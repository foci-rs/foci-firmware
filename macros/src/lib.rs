//! Proc-macros for `foci-firmware` host-side tests.

use proc_macro::TokenStream;
use quote::quote;
use syn::{ItemFn, parse_macro_input};

/// Turns a test function into one that serializes on the shared channel-0
/// state mutex and resets that state before running.
///
/// `cargo test` runs tests in parallel threads within one process, and the
/// TMC/stepper channel state lives in process-wide statics. `SHARED_STATE_MUTEX`
/// alone only prevents concurrent corruption between tests; it does not stop
/// state left behind by the previous lock-holder from leaking into the next
/// one. This macro closes both gaps in one place instead of relying on every
/// test author to hand-write the guard-then-reset boilerplate correctly.
///
/// Works on both plain `fn` tests (wrapped with `SHARED_STATE_MUTEX.blocking_lock()`)
/// and `async fn` tests (driven with `pollster::block_on`, guarded with
/// `SHARED_STATE_MUTEX.lock().await`). Any attributes already on the function,
/// such as `#[cfg(feature = "trace-velocity-sweep")]`, are preserved ahead of
/// the generated `#[test]`.
#[proc_macro_attribute]
pub fn fixture_test(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as ItemFn);
    let attrs = &input.attrs;
    let vis = &input.vis;
    let sig = &input.sig;
    let block = &input.block;

    let expanded = if sig.asyncness.is_some() {
        let mut sync_sig = sig.clone();
        sync_sig.asyncness = None;
        quote! {
            #(#attrs)*
            #[test]
            #vis #sync_sig {
                ::pollster::block_on(async {
                    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                        .lock()
                        .await;
                    crate::state::tmc::reset_all_tmc_channels();
                    crate::state::stepper::reset_all_stepper_channels();
                    #block
                });
            }
        }
    } else {
        quote! {
            #(#attrs)*
            #[test]
            #vis #sig {
                let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                    .blocking_lock();
                crate::state::tmc::reset_all_tmc_channels();
                crate::state::stepper::reset_all_stepper_channels();
                #block
            }
        }
    };

    expanded.into()
}
