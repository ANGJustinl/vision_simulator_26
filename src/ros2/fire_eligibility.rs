//! Timestamped qualification, separate from sparse fire request pulses.
#[derive(Default)]
pub struct FireEligibility {
    pub enabled: bool,
    pub issued_ns: i128,
    pub command_stamp_ns: i128,
}
impl FireEligibility {
    pub fn allows(&self, request_stamp_ns: i128, now_ns: i128) -> bool {
        self.enabled && self.command_stamp_ns >= request_stamp_ns
            && self.issued_ns <= now_ns+10_000_000 && now_ns-self.issued_ns <= 50_000_000
    }
    pub fn cancels(&self, request_stamp_ns: i128, now_ns: i128) -> bool {
        self.issued_ns > now_ns+10_000_000 || now_ns-self.issued_ns > 50_000_000
            || (!self.enabled && self.command_stamp_ns >= request_stamp_ns)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn qualification_cancels_latched_request_and_expires() {
        let mut gate=FireEligibility {enabled:true,issued_ns:1_000_000_000,command_stamp_ns:900_000_000};
        assert!(gate.allows(900_000_000,1_010_000_000));
        assert!(!gate.allows(901_000_000,1_010_000_000));
        assert!(!gate.cancels(901_000_000,1_010_000_000)); // independent topic delivery: await match
        gate.enabled=false;
        assert!(gate.cancels(900_000_000,1_010_000_000));
        gate.enabled=true;
        assert!(!gate.allows(900_000_000,1_051_000_000));
        assert!(gate.cancels(900_000_000,1_051_000_000));
        assert!(gate.cancels(900_000_000,980_000_000)); // future timestamp
    }
}
