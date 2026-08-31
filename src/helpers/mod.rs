//! Shared pure-logic helpers — functions with no hardware-register
//! access, no `klipper_*_from!` invocation, and no board-local type
//! dependencies. Hardware helpers live on capability traits in
//! `crate::context`.

pub mod clock;
pub mod drain;
pub mod lifecycle;
