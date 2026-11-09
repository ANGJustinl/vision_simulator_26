//! The simulator's behavioural clock. Wall watchdogs remain in the host worker.
use std::sync::atomic::{AtomicI64, AtomicBool, Ordering};
use std::sync::OnceLock;
use std::time::Duration;

pub const EPOCH_NS: i64 = 1_700_000_000_000_000_000;
static NOW: AtomicI64 = AtomicI64::new(EPOCH_NS);
static ORIGIN: AtomicI64 = AtomicI64::new(EPOCH_NS);
static OBSERVING: AtomicBool = AtomicBool::new(false);

pub fn enabled() -> bool {
    static ENABLED: OnceLock<bool> = OnceLock::new();
    *ENABLED.get_or_init(|| std::env::var("DAEDALUS_EXECUTION_MODE").as_deref()==Ok("lockstep"))
}

pub fn now_ns() -> i128 {
    if enabled() { i128::from(NOW.load(Ordering::SeqCst)) }
    else { std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default().as_nanos() as i128 }
}

pub fn set_ns(value: i64) { NOW.store(value,Ordering::SeqCst); }
pub fn reset_origin() { ORIGIN.store(NOW.load(Ordering::SeqCst),Ordering::SeqCst); }
pub fn sensor_seconds() -> f64 { (NOW.load(Ordering::SeqCst)-ORIGIN.load(Ordering::SeqCst)) as f64*1e-9 }
pub fn observing() -> bool {OBSERVING.load(Ordering::SeqCst)}
pub fn set_observing(value:bool) {OBSERVING.store(value,Ordering::SeqCst);}

#[derive(Clone, Copy, Debug)]
pub struct Instant { wall: std::time::Instant, logical: i128 }
impl Instant {
    pub fn now() -> Self { Self {wall:std::time::Instant::now(),logical:now_ns()} }
    pub fn elapsed(&self) -> Duration {
        if enabled() { Duration::from_nanos((now_ns()-self.logical).max(0).min(u64::MAX as i128) as u64) }
        else { self.wall.elapsed() }
    }
}
