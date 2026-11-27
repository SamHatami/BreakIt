//! Threading of the direct path's per-member loops.
//!
//! The loops here take tens of microseconds to a millisecond, many times a
//! frame, with serial work in between, so the pool's workers are asleep
//! when a loop starts, and waking one can take as long as the loop. The
//! calling thread therefore never waits for them: it works through the
//! chunks itself and the workers take chunks off the same counter as they
//! arrive.

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Mutex;

/// Run `compute(c, &mut slots[c])` for every slot, on the calling thread
/// and up to `helpers` workers of the rayon pool, as they turn up, and `consume(c, &mut
/// slots[c])` on the calling thread, in slot order, as soon as slot `c` and
/// all before it are computed.
///
/// Which thread computes a slot varies from run to run; what is computed
/// and the order it is consumed in do not, so the result is the same on any
/// number of threads.
pub(crate) fn ordered_chunks<T, C, K>(slots: &mut [T], helpers: usize, compute: C, mut consume: K)
where
    T: Send,
    C: Fn(usize, &mut T) + Sync,
    K: FnMut(usize, &mut T),
{
    let n = slots.len();
    // A lock per slot hands it from the thread that computes it to the
    // calling thread. Never contended: a slot is claimed by one thread and
    // consumed only after that thread has marked it done.
    let cells: Vec<Mutex<&mut T>> = slots.iter_mut().map(Mutex::new).collect();
    let done: Vec<AtomicBool> = (0..n).map(|_| AtomicBool::new(false)).collect();
    let next = AtomicUsize::new(0);
    let lock = |c: usize| {
        cells[c]
            .lock()
            .expect("a slot's lock is poisoned only if its kernel panicked")
    };
    // Claim and compute one slot; false when none are left.
    let work = || {
        // Checked first so the counter cannot run away while threads wait.
        if next.load(Ordering::Relaxed) >= n {
            return false;
        }
        let c = next.fetch_add(1, Ordering::Relaxed);
        if c >= n {
            return false;
        }
        compute(c, &mut lock(c));
        done[c].store(true, Ordering::Release);
        true
    };
    rayon::in_place_scope(|s| {
        for _ in 0..helpers.min(rayon::current_num_threads().saturating_sub(1)) {
            s.spawn(|_| while work() {});
        }
        let mut at = 0;
        let mut waited = 0u32;
        while at < n {
            if done[at].load(Ordering::Acquire) {
                consume(at, &mut lock(at));
                at += 1;
                waited = 0;
            } else if !work() {
                // Another thread is finishing slot `at`.
                waited += 1;
                if waited.is_multiple_of(64) {
                    std::thread::yield_now();
                } else {
                    std::hint::spin_loop();
                }
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::ordered_chunks;

    #[test]
    fn every_slot_is_computed_once_and_consumed_in_order() {
        for n in [0, 1, 2, 7, 64, 1000] {
            let mut slots = vec![0u64; n];
            let mut seen = Vec::new();
            ordered_chunks(
                &mut slots,
                3,
                |c, v| *v += 3 * c as u64 + 1,
                |c, v| seen.push((c, *v)),
            );
            let expected: Vec<_> = (0..n).map(|c| (c, 3 * c as u64 + 1)).collect();
            assert_eq!(seen, expected);
        }
    }
}
