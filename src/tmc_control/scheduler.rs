//! Absolute-deadline scheduler for the shared TMC control loop.
//!
//! The scheduler is pure data: it owns every deadline the control loop
//! services and returns one due unit per pick. It never touches SPI, GPIO, or
//! board types. Both boards drive it, OpenFFBoard with `N = 1` and Ouroboros
//! with `N = 2`.

use crate::commissioning::schedule::next_grid_deadline;

/// Shared slow-maintenance grid.
pub const MAINTENANCE_PERIOD_US: u64 = 1_000;
/// Per-channel `STATUS_FLAGS` audit grid.
pub const STATUS_AUDIT_PERIOD_US: u64 = 1_000_000;

/// Kind of work one scheduled unit performs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum UnitClass {
    /// One bootstrap step for a channel that is not yet `Ready`.
    Bootstrap,
    /// One terminal-restore operation for a channel with a restore pending.
    Restore,
    /// One commissioning engine poll.
    Commission,
    /// One fast control tick (feedforward).
    Fast,
    /// One channel's share of the 1 ms maintenance group.
    Maintenance,
    /// Advance shared homing state after every channel's maintenance unit.
    MaintenanceFinalize,
    /// One read-only `STATUS_FLAGS` audit against the pin level.
    StatusAudit,
}

impl UnitClass {
    /// Tie-break rank among non-commission classes; lower wins.
    const fn rank(self) -> u8 {
        match self {
            UnitClass::Commission | UnitClass::Restore => 0,
            UnitClass::Fast => 1,
            UnitClass::Maintenance => 2,
            UnitClass::MaintenanceFinalize => 3,
            UnitClass::StatusAudit => 4,
            UnitClass::Bootstrap => 5,
        }
    }
}

/// One due unit of work returned by [`ChannelScheduler::pick`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct Unit {
    /// What to do.
    pub class: UnitClass,
    /// Which channel; `0` for `MaintenanceFinalize`.
    pub channel: u8,
    /// The deadline this unit was scheduled for.
    pub deadline_us: u64,
}

/// Counters kept per deadline source and reported on the heartbeat.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct SourceStats {
    /// Units this source asked for (picked).
    pub requested: u32,
    /// Units that ran to completion.
    pub serviced: u32,
    /// Grid points skipped because the loop woke late.
    pub missed: u32,
    /// Largest deadline-to-pick delay observed.
    pub max_lateness_us: u32,
    /// Largest unit duration observed, in DWT cycles.
    pub max_unit_cycles: u32,
}

impl SourceStats {
    fn on_pick(&mut self, lateness_us: u64) {
        self.requested = self.requested.wrapping_add(1);
        let lateness = u32::try_from(lateness_us).unwrap_or(u32::MAX);
        self.max_lateness_us = self.max_lateness_us.max(lateness);
    }

    fn on_record(&mut self, unit_cycles: u32) {
        self.serviced = self.serviced.wrapping_add(1);
        self.max_unit_cycles = self.max_unit_cycles.max(unit_cycles);
    }
}

/// Where a channel is in its lifecycle, which decides which units it gets.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ChannelMode {
    /// Startup steps still running; only `Bootstrap` units.
    Bootstrap,
    /// Normal operation; `Fast`, `Maintenance`, and `StatusAudit` units.
    Ready,
    /// A commissioning engine owns the channel; `Commission` and `StatusAudit`.
    Commissioning,
    /// A terminal restore owns the channel; `Restore` and `StatusAudit`.
    Restore,
    /// Bootstrap failed; only `StatusAudit` units.
    Parked,
}

/// Scheduler-owned state of one open 1 ms maintenance group.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MaintenanceGroup {
    /// Homing state observed when the group opened; every unit uses it.
    homing_snapshot: u8,
    /// Bit `ch` set once channel `ch` has completed (or been skipped).
    done: u8,
    /// The finalize unit is the only thing left.
    finalize_pending: bool,
}

impl MaintenanceGroup {
    /// Homing state frozen for this group.
    pub const fn homing_snapshot(&self) -> u8 {
        self.homing_snapshot
    }
}

#[derive(Debug, Clone, Copy)]
struct ChannelSlot {
    mode: ChannelMode,
    mode_since_us: u64,
    fast_period_us: u32,
    fast_due_us: u64,
    audit_due_us: u64,
    fast_stats: SourceStats,
    commission_due_us: Option<u64>,
    commission_yield_pending: bool,
    commission_stats: SourceStats,
}

/// Absolute-deadline scheduler for `N` TMC channels.
pub struct ChannelScheduler<const N: usize> {
    channels: [ChannelSlot; N],
    maintenance_due_us: u64,
    group: Option<MaintenanceGroup>,
    maintenance_stats: SourceStats,
    audit_stats: SourceStats,
    bootstrap_stats: SourceStats,
    rr_cursor: u8,
    in_flight: Option<Unit>,
}

impl<const N: usize> ChannelScheduler<N> {
    /// Create a scheduler with every channel in `Bootstrap` and all periodic
    /// grids anchored at `now_us`.
    pub fn new(now_us: u64, fast_period_us: [u32; N]) -> Self {
        let mut channels = [ChannelSlot {
            mode: ChannelMode::Bootstrap,
            mode_since_us: now_us,
            fast_period_us: MAINTENANCE_PERIOD_US as u32,
            fast_due_us: now_us,
            audit_due_us: now_us + STATUS_AUDIT_PERIOD_US,
            fast_stats: SourceStats::default(),
            commission_due_us: None,
            commission_yield_pending: false,
            commission_stats: SourceStats::default(),
        }; N];
        for (slot, period) in channels.iter_mut().zip(fast_period_us) {
            slot.fast_period_us = period.max(1);
        }
        Self {
            channels,
            maintenance_due_us: now_us + MAINTENANCE_PERIOD_US,
            group: None,
            maintenance_stats: SourceStats::default(),
            audit_stats: SourceStats::default(),
            bootstrap_stats: SourceStats::default(),
            rr_cursor: 0,
            in_flight: None,
        }
    }

    /// Move a channel to `mode`. Entering `Ready` re-anchors its fast grid.
    pub fn set_mode(&mut self, channel: u8, mode: ChannelMode, now_us: u64) {
        let slot = &mut self.channels[channel as usize];
        if slot.mode == mode {
            return;
        }
        slot.mode = mode;
        slot.mode_since_us = now_us;
        if mode == ChannelMode::Ready {
            slot.fast_due_us = now_us;
        }
    }

    /// Current mode of `channel`.
    pub fn mode(&self, channel: u8) -> ChannelMode {
        self.channels[channel as usize].mode
    }

    /// Change one channel's fast period, re-anchoring only that channel.
    pub fn set_fast_period(&mut self, channel: u8, period_us: u32, now_us: u64) {
        let slot = &mut self.channels[channel as usize];
        let period_us = period_us.max(1);
        if slot.fast_period_us == period_us {
            return;
        }
        slot.fast_period_us = period_us;
        slot.fast_due_us = now_us + u64::from(period_us);
    }

    /// Set or clear the commission deadline for `channel`.
    pub fn set_commission_due(&mut self, channel: u8, due_us: Option<u64>) {
        self.channels[channel as usize].commission_due_us = due_us;
    }

    /// The engine asked for a scheduler yield: run other due classes before
    /// this channel's commission unit is considered again.
    pub fn note_commission_yield(&mut self, channel: u8) {
        self.channels[channel as usize].commission_yield_pending = true;
    }

    /// Commission-poll counters for `channel`.
    pub fn commission_stats(&self, channel: u8) -> SourceStats {
        self.channels[channel as usize].commission_stats
    }

    /// Earliest deadline across every source, or `None` when no source is armed.
    pub fn earliest_due_us(&self) -> Option<u64> {
        let mut earliest: Option<u64> = None;
        let mut consider = |deadline: u64| {
            earliest = Some(earliest.map_or(deadline, |e| e.min(deadline)));
        };
        for slot in &self.channels {
            match slot.mode {
                ChannelMode::Bootstrap => consider(slot.mode_since_us),
                ChannelMode::Ready => consider(slot.fast_due_us),
                ChannelMode::Commissioning | ChannelMode::Restore | ChannelMode::Parked => {}
            }
            if slot.mode != ChannelMode::Bootstrap {
                consider(slot.audit_due_us);
            }
            match slot.mode {
                ChannelMode::Commissioning => {
                    if let Some(due) = slot.commission_due_us {
                        consider(due);
                    }
                }
                ChannelMode::Restore => consider(slot.mode_since_us),
                _ => {}
            }
        }
        if self.group.is_some() {
            consider(0);
        } else {
            consider(self.maintenance_due_us);
        }
        earliest
    }

    /// Return the single due unit to run now, or `None` when nothing is due.
    ///
    /// A unit stays in flight until [`Self::record`] is called; picking again
    /// before that is a caller bug and returns `None`.
    pub fn pick(&mut self, now_us: u64, homing_state: u8) -> Option<Unit> {
        if self.in_flight.is_some() {
            return None;
        }
        if self.group.is_none() && self.maintenance_due_us <= now_us {
            let mut done = 0u8;
            for (idx, slot) in self.channels.iter().enumerate() {
                if slot.mode != ChannelMode::Ready {
                    done |= 1 << idx;
                }
            }
            self.group = Some(MaintenanceGroup {
                homing_snapshot: homing_state,
                done,
                finalize_pending: false,
            });
        }

        let mut priority: Option<(Unit, u8, bool)> = None;
        for idx in 0..N {
            let channel = idx as u8;
            let rr_distance = (channel + N as u8 - self.rr_cursor) % N as u8;
            let slot = &self.channels[idx];
            let candidate = match slot.mode {
                ChannelMode::Commissioning => slot
                    .commission_due_us
                    .filter(|due| *due <= now_us)
                    .map(|due| {
                        (
                            Unit {
                                class: UnitClass::Commission,
                                channel,
                                deadline_us: due,
                            },
                            slot.commission_yield_pending,
                        )
                    }),
                ChannelMode::Restore => Some((
                    Unit {
                        class: UnitClass::Restore,
                        channel,
                        deadline_us: slot.mode_since_us,
                    },
                    false,
                )),
                _ => None,
            };
            if let Some((unit, yielding)) = candidate {
                let better = match priority {
                    None => true,
                    Some((current, current_rr, _)) => {
                        (unit.deadline_us, rr_distance) < (current.deadline_us, current_rr)
                    }
                };
                if better {
                    priority = Some((unit, rr_distance, yielding));
                }
            }
        }

        let mut best: Option<(Unit, u8)> = None;
        let mut offer = |unit: Unit, rr_distance: u8| {
            let better = match best {
                None => true,
                Some((current, current_rr)) => {
                    (unit.deadline_us, unit.class.rank(), rr_distance)
                        < (current.deadline_us, current.class.rank(), current_rr)
                }
            };
            if better {
                best = Some((unit, rr_distance));
            }
        };

        for idx in 0..N {
            let channel = idx as u8;
            let rr_distance = (channel + N as u8 - self.rr_cursor) % N as u8;
            let slot = &self.channels[idx];
            match slot.mode {
                ChannelMode::Bootstrap => offer(
                    Unit {
                        class: UnitClass::Bootstrap,
                        channel,
                        deadline_us: slot.mode_since_us,
                    },
                    rr_distance,
                ),
                ChannelMode::Ready => {
                    if slot.fast_due_us <= now_us {
                        offer(
                            Unit {
                                class: UnitClass::Fast,
                                channel,
                                deadline_us: slot.fast_due_us,
                            },
                            rr_distance,
                        );
                    }
                }
                ChannelMode::Commissioning | ChannelMode::Restore | ChannelMode::Parked => {}
            }
            if slot.mode != ChannelMode::Bootstrap && slot.audit_due_us <= now_us {
                offer(
                    Unit {
                        class: UnitClass::StatusAudit,
                        channel,
                        deadline_us: slot.audit_due_us,
                    },
                    rr_distance,
                );
            }
        }

        if let Some(group) = self.group {
            let deadline_us = self.maintenance_due_us;
            let next = (0..N).find(|idx| group.done & (1 << idx) == 0);
            match next {
                Some(idx) => offer(
                    Unit {
                        class: UnitClass::Maintenance,
                        channel: idx as u8,
                        deadline_us,
                    },
                    0,
                ),
                None => offer(
                    Unit {
                        class: UnitClass::MaintenanceFinalize,
                        channel: 0,
                        deadline_us,
                    },
                    0,
                ),
            }
        }

        let unit = match (priority, best) {
            (Some((unit, _, false)), _) => unit,
            (Some((unit, _, true)), None) => {
                self.channels[unit.channel as usize].commission_yield_pending = false;
                unit
            }
            (Some((commission, _, true)), Some((other, _))) => {
                self.channels[commission.channel as usize].commission_yield_pending = false;
                other
            }
            (None, Some((other, _))) => other,
            (None, None) => return None,
        };
        let lateness = now_us.saturating_sub(unit.deadline_us);
        match unit.class {
            UnitClass::Fast => self.channels[unit.channel as usize]
                .fast_stats
                .on_pick(lateness),
            UnitClass::Maintenance | UnitClass::MaintenanceFinalize => {
                if unit.class == UnitClass::MaintenanceFinalize {
                    self.maintenance_stats.on_pick(lateness);
                }
            }
            UnitClass::StatusAudit => self.audit_stats.on_pick(lateness),
            UnitClass::Bootstrap => self.bootstrap_stats.on_pick(lateness),
            UnitClass::Commission => {
                let slot = &mut self.channels[unit.channel as usize];
                slot.commission_stats.on_pick(lateness);
                slot.commission_due_us = None;
            }
            UnitClass::Restore => {}
        }
        if matches!(unit.class, UnitClass::Fast) {
            self.rr_cursor = (unit.channel + 1) % N as u8;
        }
        self.in_flight = Some(unit);
        Some(unit)
    }

    /// Complete the in-flight unit and advance its source's deadline.
    pub fn record(&mut self, now_us: u64, unit_cycles: u32) {
        let Some(unit) = self.in_flight.take() else {
            return;
        };
        let idx = unit.channel as usize;
        match unit.class {
            UnitClass::Fast => {
                let slot = &mut self.channels[idx];
                let period = u64::from(slot.fast_period_us);
                let (next, missed) = advance_grid(slot.fast_due_us, period, now_us);
                slot.fast_due_us = next;
                slot.fast_stats.missed = slot.fast_stats.missed.wrapping_add(missed);
                slot.fast_stats.on_record(unit_cycles);
            }
            UnitClass::Maintenance => {
                if let Some(group) = self.group.as_mut() {
                    group.done |= 1 << idx;
                    group.finalize_pending = group.done == Self::all_channels_mask();
                }
                self.maintenance_stats.max_unit_cycles =
                    self.maintenance_stats.max_unit_cycles.max(unit_cycles);
            }
            UnitClass::MaintenanceFinalize => {
                self.group = None;
                let (next, missed) =
                    advance_grid(self.maintenance_due_us, MAINTENANCE_PERIOD_US, now_us);
                self.maintenance_due_us = next;
                self.maintenance_stats.missed = self.maintenance_stats.missed.wrapping_add(missed);
                self.maintenance_stats.on_record(unit_cycles);
            }
            UnitClass::StatusAudit => {
                let slot = &mut self.channels[idx];
                let (next, missed) =
                    advance_grid(slot.audit_due_us, STATUS_AUDIT_PERIOD_US, now_us);
                slot.audit_due_us = next;
                self.audit_stats.missed = self.audit_stats.missed.wrapping_add(missed);
                self.audit_stats.on_record(unit_cycles);
            }
            UnitClass::Bootstrap => {
                self.channels[idx].mode_since_us = now_us;
                self.bootstrap_stats.on_record(unit_cycles);
            }
            UnitClass::Commission => {
                self.channels[idx].commission_stats.on_record(unit_cycles);
            }
            UnitClass::Restore => {
                self.channels[idx].mode_since_us = now_us;
            }
        }
    }

    /// Fast-tick counters for `channel`.
    pub fn fast_stats(&self, channel: u8) -> SourceStats {
        self.channels[channel as usize].fast_stats
    }

    /// Shared maintenance counters (one `requested`/`serviced` per group).
    pub fn maintenance_stats(&self) -> SourceStats {
        self.maintenance_stats
    }

    /// Status-audit counters summed across channels.
    pub fn audit_stats(&self) -> SourceStats {
        self.audit_stats
    }

    /// The open maintenance group, if any.
    pub fn maintenance_group(&self) -> Option<&MaintenanceGroup> {
        self.group.as_ref()
    }

    const fn all_channels_mask() -> u8 {
        ((1u16 << N) - 1) as u8
    }
}

/// Advance a periodic deadline to the first strictly future grid point and
/// report how many grid points were skipped.
fn advance_grid(previous_deadline_us: u64, period_us: u64, now_us: u64) -> (u64, u32) {
    match next_grid_deadline(previous_deadline_us, period_us, now_us) {
        Ok(next) => {
            let slots = (next - previous_deadline_us) / period_us;
            let missed = u32::try_from(slots.saturating_sub(1)).unwrap_or(u32::MAX);
            (next, missed)
        }
        Err(_) => (now_us.saturating_add(period_us), 0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ready<const N: usize>(now: u64, periods: [u32; N]) -> ChannelScheduler<N> {
        let mut s = ChannelScheduler::new(now, periods);
        for ch in 0..N as u8 {
            s.set_mode(ch, ChannelMode::Ready, now);
        }
        s
    }

    /// Drive the scheduler over `[start, end)` with zero-duration units and
    /// count how many `Fast` units each channel received.
    fn service_counts<const N: usize>(
        s: &mut ChannelScheduler<N>,
        start: u64,
        end: u64,
    ) -> [u32; N] {
        let mut counts = [0u32; N];
        let mut now = start;
        while now < end {
            while let Some(unit) = s.pick(now, 0) {
                if unit.class == UnitClass::Fast {
                    counts[unit.channel as usize] += 1;
                }
                s.record(now, 0);
            }
            now = s
                .earliest_due_us()
                .expect("fast sources always have a deadline");
        }
        counts
    }

    #[test]
    fn ten_khz_and_two_khz_pair_service_at_five_to_one() {
        let mut s = ready(0, [100, 500]);
        let counts = service_counts(&mut s, 0, 100_000);
        assert_eq!(counts[0], 1_000);
        assert_eq!(counts[1], 200);
    }

    #[test]
    fn unit_duration_does_not_stretch_cadence() {
        let mut s = ready(0, [100]);
        // Every unit takes 40 us; the deadline grid must stay at 100 us.
        let mut now = 0;
        let mut fast_deadlines = [0u64; 5];
        let mut seen = 0;
        while seen < 5 {
            if let Some(unit) = s.pick(now, 0) {
                if unit.class == UnitClass::Fast {
                    fast_deadlines[seen] = unit.deadline_us;
                    seen += 1;
                }
                now += 40;
                s.record(now, 0);
            } else {
                now = s.earliest_due_us().unwrap();
            }
        }
        assert_eq!(fast_deadlines, [0, 100, 200, 300, 400]);
    }

    #[test]
    fn late_wake_skips_and_counts_missed_periods_without_bursting() {
        let mut s = ready(0, [100]);
        let unit = s.pick(0, 0).unwrap();
        assert_eq!(unit.class, UnitClass::Fast);
        s.record(0, 0);
        // Sleep through 4 grid points (100, 200, 300, 400); wake at 450.
        let unit = s.pick(450, 0).unwrap();
        assert_eq!(unit.class, UnitClass::Fast);
        assert_eq!(unit.deadline_us, 100);
        s.record(450, 0);
        // No burst: the next fast unit is at 500, not 200.
        assert!(s.pick(450, 0).is_none());
        assert_eq!(s.earliest_due_us(), Some(500));
        let stats = s.fast_stats(0);
        assert_eq!(stats.missed, 3);
        assert_eq!(stats.serviced, 2);
        assert_eq!(stats.max_lateness_us, 350);
    }

    #[test]
    fn period_change_reanchors_only_that_channel() {
        let mut s = ready(0, [100, 100]);
        let _ = s.pick(0, 0).unwrap();
        s.record(0, 0);
        let _ = s.pick(0, 0).unwrap();
        s.record(0, 0);
        s.set_fast_period(1, 250, 30);
        assert_eq!(s.earliest_due_us(), Some(100));
        let unit = s.pick(100, 0).unwrap();
        assert_eq!((unit.class, unit.channel), (UnitClass::Fast, 0));
        s.record(100, 0);
        assert!(s.pick(100, 0).is_none());
        assert_eq!(s.earliest_due_us(), Some(200));
        let _ = s.pick(200, 0).unwrap();
        s.record(200, 0);
        let unit = s.pick(280, 0).unwrap();
        assert_eq!(
            (unit.class, unit.channel, unit.deadline_us),
            (UnitClass::Fast, 1, 280)
        );
    }

    #[test]
    fn equal_deadlines_alternate_by_round_robin() {
        let mut s = ready(0, [100, 100]);
        let first = s.pick(0, 0).unwrap();
        s.record(0, 0);
        let second = s.pick(0, 0).unwrap();
        s.record(0, 0);
        assert_eq!((first.channel, second.channel), (0, 1));
        // The cursor advances past the last picked channel, so each grid
        // point starts with the channel that went second last time.
        let third = s.pick(100, 0).unwrap();
        s.record(100, 0);
        let fourth = s.pick(100, 0).unwrap();
        s.record(100, 0);
        assert_eq!((third.channel, fourth.channel), (0, 1));
        // Three channels make the rotation visible. Deadline still comes
        // first: at t=450 channel 1 is overdue from t=200 and goes first;
        // channels 0 and 2 then tie at 450 and the cursor, which sits just
        // past channel 1, puts channel 2 ahead of channel 0.
        let mut t = ready(0, [100, 100, 100]);
        let mut order = [0u8; 3];
        for slot in &mut order {
            *slot = t.pick(0, 0).unwrap().channel;
            t.record(0, 0);
        }
        assert_eq!(order, [0, 1, 2]);
        t.set_fast_period(0, 400, 50);
        t.set_fast_period(2, 400, 50);
        let _ = t.pick(100, 0).unwrap();
        t.record(100, 0);
        let mut later = [0u8; 3];
        for slot in &mut later {
            *slot = t.pick(450, 0).unwrap().channel;
            t.record(450, 0);
        }
        assert_eq!(later, [1, 2, 0]);
    }

    #[test]
    fn maintenance_group_runs_every_ready_channel_then_finalizes_with_one_snapshot() {
        let mut s = ready(0, [1_000, 1_000]);
        // Fast ticks at t=0 first (they are due and earliest).
        let _ = s.pick(0, 0).unwrap();
        s.record(0, 0);
        let _ = s.pick(0, 0).unwrap();
        s.record(0, 0);
        // Maintenance is due at t=1000 with homing_state = 3.
        let mut classes = [UnitClass::Fast; 5];
        let mut i = 0;
        while let Some(unit) = s.pick(1_000, 3) {
            classes[i] = unit.class;
            i += 1;
            if unit.class == UnitClass::Maintenance {
                assert_eq!(s.maintenance_group().unwrap().homing_snapshot(), 3);
            }
            // A different homing state during the group must not change the snapshot.
            s.record(1_000, 0);
        }
        assert_eq!(
            &classes[..i],
            &[
                UnitClass::Fast,
                UnitClass::Fast,
                UnitClass::Maintenance,
                UnitClass::Maintenance,
                UnitClass::MaintenanceFinalize,
            ]
        );
        assert!(s.maintenance_group().is_none());
        assert_eq!(s.maintenance_stats().serviced, 1);
    }

    #[test]
    fn maintenance_group_snapshot_is_frozen_across_interleaved_picks() {
        let mut s = ready(0, [1_000, 1_000]);
        let _ = s.pick(0, 0).unwrap();
        s.record(0, 0);
        let _ = s.pick(0, 0).unwrap();
        s.record(0, 0);
        // At t=1000 both fast ticks tie with maintenance and win by class.
        let _ = s.pick(1_000, 7).unwrap();
        s.record(1_000, 0);
        let _ = s.pick(1_000, 7).unwrap();
        s.record(1_000, 0);
        // The group opened at the first t=1000 pick with snapshot 7.
        let unit = s.pick(1_000, 7).unwrap();
        assert_eq!((unit.class, unit.channel), (UnitClass::Maintenance, 0));
        s.record(1_000, 0);
        // Homing state changes to 8 before channel 1's unit is picked.
        let unit = s.pick(1_000, 8).unwrap();
        assert_eq!((unit.class, unit.channel), (UnitClass::Maintenance, 1));
        assert_eq!(s.maintenance_group().unwrap().homing_snapshot(), 7);
        s.record(1_000, 0);
        let unit = s.pick(1_000, 8).unwrap();
        assert_eq!(unit.class, UnitClass::MaintenanceFinalize);
        s.record(1_000, 0);
    }

    #[test]
    fn maintenance_cannot_be_starved_by_fast_ticks() {
        // One channel at 100 us whose unit always takes 100 us: fast is due
        // again the moment it finishes, but strictly-future advance means it
        // actually falls behind by a full period each cycle. Maintenance must
        // still run once its own lateness makes it the earliest deadline.
        let mut s = ready(0, [100]);
        let mut now = 0;
        let mut saw_maintenance = false;
        for _ in 0..40 {
            let Some(unit) = s.pick(now, 0) else {
                now = s.earliest_due_us().expect("some source is always armed");
                continue;
            };
            if unit.class == UnitClass::Maintenance {
                saw_maintenance = true;
                break;
            }
            let finish = now + 100;
            s.record(finish, 0);
            now = finish;
        }
        assert!(saw_maintenance);
    }

    #[test]
    fn single_channel_reproduces_fast_and_slow_tick_pattern() {
        let mut s = ready(0, [1_000]);
        let mut sequence = [UnitClass::Fast; 7];
        let mut i = 0;
        for now in [0u64, 1_000, 2_000] {
            while let Some(unit) = s.pick(now, 0) {
                sequence[i] = unit.class;
                i += 1;
                s.record(now, 0);
            }
        }
        assert_eq!(
            sequence,
            [
                UnitClass::Fast,
                UnitClass::Fast,
                UnitClass::Maintenance,
                UnitClass::MaintenanceFinalize,
                UnitClass::Fast,
                UnitClass::Maintenance,
                UnitClass::MaintenanceFinalize,
            ]
        );
    }

    #[test]
    fn status_audit_is_due_every_second_for_ready_and_parked_channels() {
        let mut s = ready(0, [1_000, 1_000]);
        s.set_mode(1, ChannelMode::Parked, 0);
        let mut audits = [0u32; 2];
        let mut now = 0;
        while now <= 2_000_000 {
            while let Some(unit) = s.pick(now, 0) {
                if unit.class == UnitClass::StatusAudit {
                    audits[unit.channel as usize] += 1;
                }
                s.record(now, 0);
            }
            now = s.earliest_due_us().unwrap();
        }
        // Audit at t=1s and t=2s for both channels; t=0 is not an audit point.
        assert_eq!(audits, [2, 2]);
        assert_eq!(s.audit_stats().serviced, 4);
    }

    #[test]
    fn bootstrap_and_parked_channels_get_no_fast_or_maintenance_units() {
        let mut s = ChannelScheduler::new(0, [100, 100]);
        s.set_mode(1, ChannelMode::Parked, 0);
        // Channel 0 is in Bootstrap: its Bootstrap unit stays due until the
        // driver reports Ready, and the parked channel gets nothing.
        for _ in 0..3 {
            let unit = s.pick(0, 0).unwrap();
            assert_eq!((unit.class, unit.channel), (UnitClass::Bootstrap, 0));
            s.record(0, 0);
        }
        // Maintenance at 1 ms skips both channels and only finalizes; the
        // overdue Bootstrap unit still goes first by deadline.
        let unit = s.pick(1_000, 0).unwrap();
        assert_eq!(unit.class, UnitClass::Bootstrap);
        s.record(1_000, 0);
        let unit = s.pick(1_000, 0).unwrap();
        assert_eq!(unit.class, UnitClass::MaintenanceFinalize);
        s.record(1_000, 0);
        let unit = s.pick(1_000, 0).unwrap();
        assert_eq!(unit.class, UnitClass::Bootstrap);
        s.record(1_000, 0);
        // Only a Ready channel is ever offered Fast or Maintenance.
        s.set_mode(0, ChannelMode::Ready, 1_000);
        let unit = s.pick(1_000, 0).unwrap();
        assert_eq!((unit.class, unit.channel), (UnitClass::Fast, 0));
        s.record(1_000, 0);
    }

    #[test]
    fn record_tracks_max_unit_cycles() {
        let mut s = ready(0, [100]);
        let _ = s.pick(0, 0).unwrap();
        s.record(0, 1_234);
        let _ = s.pick(100, 0).unwrap();
        s.record(100, 99);
        assert_eq!(s.fast_stats(0).max_unit_cycles, 1_234);
        assert_eq!(s.fast_stats(0).requested, 2);
    }

    fn commissioning<const N: usize>(now: u64, periods: [u32; N], ch: u8) -> ChannelScheduler<N> {
        let mut s = ready(now, periods);
        s.set_mode(ch, ChannelMode::Commissioning, now);
        s
    }

    #[test]
    fn due_commission_unit_beats_any_number_of_overdue_units() {
        let mut s = commissioning(0, [100, 100], 0);
        // Channel 1 fast is overdue by 900 us, maintenance by 500 us,
        // channel 1 audit is overdue too; commission became due just now.
        s.set_commission_due(0, Some(1_000));
        let unit = s.pick(1_000, 0).unwrap();
        assert_eq!(
            (unit.class, unit.channel, unit.deadline_us),
            (UnitClass::Commission, 0, 1_000)
        );
        s.record(1_000, 0);
    }

    #[test]
    fn commission_due_mid_unit_waits_exactly_one_unit() {
        let mut s = commissioning(0, [100, 100], 0);
        // Channel 1's fast unit is picked at t=0 and runs until t=60.
        let unit = s.pick(0, 0).unwrap();
        assert_eq!((unit.class, unit.channel), (UnitClass::Fast, 1));
        // Commission becomes due at t=20, while that unit is in flight.
        s.set_commission_due(0, Some(20));
        assert!(
            s.pick(40, 0).is_none(),
            "nothing is picked while a unit is in flight"
        );
        s.record(60, 0);
        let unit = s.pick(60, 0).unwrap();
        assert_eq!((unit.class, unit.channel), (UnitClass::Commission, 0));
        assert_eq!(s.commission_stats(0).max_lateness_us, 40);
        s.record(60, 0);
    }

    #[test]
    fn commission_deadline_is_cleared_after_service_until_the_engine_sets_a_new_one() {
        let mut s = commissioning(0, [1_000], 0);
        s.set_commission_due(0, Some(0));
        let _ = s.pick(0, 0).unwrap();
        s.record(0, 0);
        assert!(s.pick(0, 0).is_none());
        assert_eq!(s.earliest_due_us(), Some(1_000));
        s.set_commission_due(0, Some(500));
        assert_eq!(s.earliest_due_us(), Some(500));
    }

    #[test]
    fn yield_lets_other_classes_run_before_commission_is_considered_again() {
        let mut s = commissioning(0, [100, 100], 0);
        s.set_commission_due(0, Some(0));
        s.note_commission_yield(0);
        // Channel 1 fast is due at t=0 as well; it must go first.
        let unit = s.pick(0, 0).unwrap();
        assert_eq!((unit.class, unit.channel), (UnitClass::Fast, 1));
        s.record(0, 0);
        let unit = s.pick(0, 0).unwrap();
        assert_eq!((unit.class, unit.channel), (UnitClass::Commission, 0));
        s.record(0, 0);
    }

    #[test]
    fn yield_with_nothing_else_due_runs_commission_immediately() {
        let mut s = commissioning(0, [100, 100], 0);
        // Consume channel 1's t=0 fast tick first.
        let _ = s.pick(0, 0).unwrap();
        s.record(0, 0);
        s.set_commission_due(0, Some(0));
        s.note_commission_yield(0);
        let unit = s.pick(0, 0).unwrap();
        assert_eq!(unit.class, UnitClass::Commission);
        s.record(0, 0);
    }

    #[test]
    fn fast_delay_during_ready_now_run_is_bounded_by_the_forced_yield_count() {
        use crate::tmc_control::MAX_CONSECUTIVE_READY_POLLS;
        let mut s = commissioning(0, [100, 100], 0);
        // Channel 1's fast tick is due from t=0 and stays overdue while the
        // engine answers ReadyNow forever; the poll driver forces a YieldNow
        // after MAX_CONSECUTIVE_READY_POLLS. Model that here.
        let mut commission_runs = 0u8;
        let mut now = 0u64;
        loop {
            s.set_commission_due(0, Some(now));
            if commission_runs == MAX_CONSECUTIVE_READY_POLLS {
                s.note_commission_yield(0);
            }
            let unit = s.pick(now, 0).unwrap();
            now += 10;
            s.record(now, 0);
            if unit.class == UnitClass::Fast {
                assert_eq!(unit.channel, 1);
                break;
            }
            assert_eq!(unit.class, UnitClass::Commission);
            commission_runs += 1;
            assert!(commission_runs <= MAX_CONSECUTIVE_READY_POLLS + 1);
        }
    }

    #[test]
    fn restore_units_have_commission_priority_and_stay_due_until_mode_changes() {
        let mut s = ready(0, [100, 100]);
        s.set_mode(0, ChannelMode::Restore, 0);
        let unit = s.pick(0, 0).unwrap();
        assert_eq!((unit.class, unit.channel), (UnitClass::Restore, 0));
        s.record(0, 0);
        let unit = s.pick(0, 0).unwrap();
        assert_eq!((unit.class, unit.channel), (UnitClass::Restore, 0));
        s.record(0, 0);
        s.set_mode(0, ChannelMode::Ready, 0);
        let unit = s.pick(0, 0).unwrap();
        assert_eq!(unit.class, UnitClass::Fast);
        s.record(0, 0);
    }

    #[test]
    fn commissioning_channel_gets_no_fast_or_maintenance_but_keeps_audit() {
        let mut s = commissioning(0, [100, 100], 0);
        let mut classes_for_ch0 = 0u32;
        let mut now = 0;
        while now <= 1_000_000 {
            while let Some(unit) = s.pick(now, 0) {
                if unit.channel == 0 && unit.class != UnitClass::MaintenanceFinalize {
                    assert_eq!(unit.class, UnitClass::StatusAudit);
                    classes_for_ch0 += 1;
                }
                s.record(now, 0);
            }
            now = s.earliest_due_us().unwrap();
        }
        assert_eq!(classes_for_ch0, 1);
    }
}
