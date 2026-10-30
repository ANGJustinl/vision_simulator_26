//! Closed-loop commanded yaw velocity -> actual yaw velocity.
//! Historical ID preserves the mis-transcribed denominator [1,43.16,3975].
//! Corrected IDs use the archived image denominator [1,34.16,975].
//! Unit DC is an explicit simulation assumption, not a motor identification.
#[derive(Clone, Copy, Debug, Default)]
pub struct YawReference { pub x: [f64;2] }
#[derive(Clone, Copy, Debug)]
pub struct Advance { pub angle: f64, pub raw_rate: f64, pub rate: f64, pub limited: bool }
impl YawReference {
    pub const DC: f64 = 1084.0/3975.0;
    // Controllable canonical realization: q''+43.16q'+3975q=u.
    pub fn output(&self, unit_dc: bool)->f64 {
        (1084.0*self.x[0]-12.65*self.x[1])/if unit_dc {Self::DC} else {1.0}
    }
    fn exact(&self,u:f64,t:f64,unit_dc:bool)->(Self,f64) {
        let a=43.16/2.0; let w=(3975.0_f64-a*a).sqrt();
        let equilibrium=u/3975.0; let q=self.x[0]-equilibrium; let v=self.x[1];
        let e=(-a*t).exp(); let c=(w*t).cos(); let s=(w*t).sin()/w;
        let next=Self{x:[equilibrium+e*((c+a*s)*q+s*v), e*(-3975.0*s*q+(c-a*s)*v)]};
        // Integrate the state equation exactly, including the angle state.
        let iq=(u*t-(next.x[1]-v)-43.16*(next.x[0]-self.x[0]))/3975.0;
        let angle=(1084.0*iq-12.65*(next.x[0]-self.x[0]))/if unit_dc {Self::DC} else {1.0};
        (next,angle)
    }
    pub fn advance(&mut self,u:f64,dt:f64,limit:f64,unit_dc:bool)->Advance {
        assert!(u.is_finite() && dt.is_finite() && dt>=0.0 && limit>0.0 && self.x.iter().all(|v|v.is_finite()),"invalid yaw model state/input");
        // Exact ZOH model; clipping is integrated on <=0.5ms slices. Simpson
        // is used only for nonlinear clipped intervals, never the linear path.
        let count=(dt/0.0005).ceil().max(1.0) as usize; let h=dt/count as f64;
        let mut angle=0.0;let mut limited=false;
        for _ in 0..count {
            let (next,linear)=self.exact(u,h,unit_dc);
            let r0=self.output(unit_dc);let rm=self.exact(u,h*0.5,unit_dc).0.output(unit_dc);let r1=next.output(unit_dc);
            // Analytic stationary points catch arbitrarily narrow peaks above
            // the limiter, even when endpoints and midpoint are below it.
            let a=21.58;let w=(3975.0_f64-a*a).sqrt();
            let scale=if unit_dc {Self::DC}else{1.0};
            let aa=r0-u*Self::DC/scale;
            let derivative=(1084.0*self.x[1]-12.65*(u-43.16*self.x[1]-3975.0*self.x[0]))/scale;
            let bb=(derivative+a*aa)/w;
            let phase=(-(w*bb-a*aa)).atan2(-w*aa-a*bb);
            let mut peak=false;
            for k in -1..=2 {let t=(phase+k as f64*std::f64::consts::PI)/w;
                if t>0.0 && t<h && self.exact(u,t,unit_dc).0.output(unit_dc).abs()>limit {peak=true;}}
            if peak || [r0,rm,r1].iter().any(|r|r.abs()>limit) {
                limited=true;
                // Adaptive integration to resolve saturation crossings.
                fn simpson(f:&impl Fn(f64)->f64,a:f64,b:f64,eps:f64,n:u32)->f64 {
                    let m=(a+b)*0.5;let l=(a+m)*0.5;let r=(m+b)*0.5;
                    let whole=(b-a)*(f(a)+4.0*f(m)+f(b))/6.0;
                    let halves=(m-a)*(f(a)+4.0*f(l)+f(m))/6.0+(b-m)*(f(m)+4.0*f(r)+f(b))/6.0;
                    if n==0 || (halves-whole).abs()<15.0*eps {halves+(halves-whole)/15.0}
                    else {simpson(f,a,m,eps*0.5,n-1)+simpson(f,m,b,eps*0.5,n-1)}
                }
                angle+=simpson(&|t|self.exact(u,t,unit_dc).0.output(unit_dc).clamp(-limit,limit),0.0,h,1e-12,16);
            } else {angle+=linear;}
            *self=next;
        }
        let raw_rate=self.output(unit_dc);
        assert!(raw_rate.is_finite() && angle.is_finite() && self.x.iter().all(|v|v.is_finite()),"nonfinite yaw reference: experiment invalid");
        Advance{angle,raw_rate,rate:raw_rate.clamp(-limit,limit),limited}
    }
}
/// Correct transcription of the archived RM2017 formula image:
/// (-12.65 s + 1084) / (s² + 34.16 s + 975).
/// Raw response is distinct from the optional unit-DC simulation assumption.
/// This kernel intentionally does not refactor the historical implementation:
/// preserving its source and floating-point order enables exact reproduction.
/// Raw and unit-DC corrected identities share this one corrected kernel.
#[derive(Clone, Copy, Debug, Default)]
pub struct CorrectedYawReference { pub x: [f64;2] }
impl CorrectedYawReference {
    pub const DC: f64 = 1084.0/975.0;
    // Controllable canonical realization: q''+34.16q'+975q=u.
    pub fn output(&self, unit_dc: bool)->f64 {
        (1084.0*self.x[0]-12.65*self.x[1])/if unit_dc {Self::DC} else {1.0}
    }
    fn exact(&self,u:f64,t:f64,unit_dc:bool)->(Self,f64) {
        let a=34.16/2.0; let w=(975.0_f64-a*a).sqrt();
        let equilibrium=u/975.0; let q=self.x[0]-equilibrium; let v=self.x[1];
        let e=(-a*t).exp(); let c=(w*t).cos(); let s=(w*t).sin()/w;
        let next=Self{x:[equilibrium+e*((c+a*s)*q+s*v), e*(-975.0*s*q+(c-a*s)*v)]};
        // Integrate the state equation exactly, including the angle state.
        let iq=(u*t-(next.x[1]-v)-34.16*(next.x[0]-self.x[0]))/975.0;
        let angle=(1084.0*iq-12.65*(next.x[0]-self.x[0]))/if unit_dc {Self::DC} else {1.0};
        (next,angle)
    }
    pub fn advance(&mut self,u:f64,dt:f64,limit:f64,unit_dc:bool)->Advance {
        assert!(u.is_finite() && dt.is_finite() && dt>=0.0 && limit.is_finite() && limit>0.0 && self.x.iter().all(|v|v.is_finite()),"invalid yaw model state/input");
        // Exact ZOH model; clipping is integrated on <=0.5ms slices. Simpson
        // is used only for nonlinear clipped intervals, never the linear path.
        let count=(dt/0.0005).ceil().max(1.0) as usize; let h=dt/count as f64;
        let mut angle=0.0;let mut limited=false;
        for _ in 0..count {
            let (next,linear)=self.exact(u,h,unit_dc);
            let r0=self.output(unit_dc);let rm=self.exact(u,h*0.5,unit_dc).0.output(unit_dc);let r1=next.output(unit_dc);
            // Analytic stationary points catch arbitrarily narrow peaks above
            // the limiter, even when endpoints and midpoint are below it.
            let a=17.08;let w=(975.0_f64-a*a).sqrt();
            let scale=if unit_dc {Self::DC}else{1.0};
            let aa=r0-u*Self::DC/scale;
            let derivative=(1084.0*self.x[1]-12.65*(u-34.16*self.x[1]-975.0*self.x[0]))/scale;
            let bb=(derivative+a*aa)/w;
            let phase=(-(w*bb-a*aa)).atan2(-w*aa-a*bb);
            let mut peak=false;
            for k in -1..=2 {let t=(phase+k as f64*std::f64::consts::PI)/w;
                if t>0.0 && t<h && self.exact(u,t,unit_dc).0.output(unit_dc).abs()>limit {peak=true;}}
            if peak || [r0,rm,r1].iter().any(|r|r.abs()>limit) {
                limited=true;
                // Adaptive integration to resolve saturation crossings.
                fn simpson(f:&impl Fn(f64)->f64,a:f64,b:f64,eps:f64,n:u32)->f64 {
                    let m=(a+b)*0.5;let l=(a+m)*0.5;let r=(m+b)*0.5;
                    let whole=(b-a)*(f(a)+4.0*f(m)+f(b))/6.0;
                    let halves=(m-a)*(f(a)+4.0*f(l)+f(m))/6.0+(b-m)*(f(m)+4.0*f(r)+f(b))/6.0;
                    if n==0 || (halves-whole).abs()<15.0*eps {halves+(halves-whole)/15.0}
                    else {simpson(f,a,m,eps*0.5,n-1)+simpson(f,m,b,eps*0.5,n-1)}
                }
                angle+=simpson(&|t|self.exact(u,t,unit_dc).0.output(unit_dc).clamp(-limit,limit),0.0,h,1e-12,16);
            } else {angle+=linear;}
            *self=next;
        }
        let raw_rate=self.output(unit_dc);
        assert!(raw_rate.is_finite() && angle.is_finite() && self.x.iter().all(|v|v.is_finite()),"nonfinite yaw reference: experiment invalid");
        Advance{angle,raw_rate,rate:raw_rate.clamp(-limit,limit),limited}
    }
}
/// Stable identities: never reinterpret the historical mis-transcription ID.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReferenceModel {
    HistoricalUnitDc,
    CorrectedRaw,
    CorrectedUnitDc,
}
impl ReferenceModel {
    pub fn from_id(id: &str) -> Option<Self> {
        match id {
            "rm2017-yaw-unitdc" => Some(Self::HistoricalUnitDc),
            "rm2017-yaw-corrected-raw" => Some(Self::CorrectedRaw),
            "rm2017-yaw-corrected-unitdc" => Some(Self::CorrectedUnitDc),
            _ => None,
        }
    }
    pub fn advance(self, state: &mut YawReference, input: f64, dt: f64, limit: f64) -> Advance {
        match self {
            Self::HistoricalUnitDc => state.advance(input,dt,limit,true),
            Self::CorrectedRaw | Self::CorrectedUnitDc => {
                let mut model=CorrectedYawReference{x:state.x};
                let result=model.advance(input,dt,limit,self==Self::CorrectedUnitDc);
                state.x=model.x;
                result
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn corrected_source_anchors_and_raw_vs_normalized_identity() {
        // Audited image formula, independent response anchors (2026-10-29).
        assert!((CorrectedYawReference::DC-1.1117948717948718).abs()<1e-15);
        let mut raw=CorrectedYawReference::default();let mut unit=CorrectedYawReference::default();
        for (time,expected) in [(0.00953,-0.0516),(0.12971,1.1350),(1.0,1.0)] {
            let mut model=CorrectedYawReference::default();let step=model.advance(1.0,time,100.0,true);
            assert!((step.rate-expected).abs()<0.0001,"{time}: {}",step.rate);
        }
        for k in 0..1000 {
            let input=if k<300 {0.7}else if k<600 {-0.3}else{0.0};
            let a=raw.advance(input,0.001,100.0,false);let b=unit.advance(input,0.001,100.0,true);
            assert_eq!(raw.x,unit.x);
            assert!((a.raw_rate-b.raw_rate*CorrectedYawReference::DC).abs()<1e-13);
            assert!((a.angle-b.angle*CorrectedYawReference::DC).abs()<1e-13);
        }
        // Correct formula's relative-DC -3 dB bandwidth is ~6.4187 Hz.
        let omega=std::f64::consts::TAU*6.4187;
        let numerator=(1084.0_f64.powi(2)+(12.65*omega).powi(2)).sqrt();
        let denominator=((975.0-omega*omega).powi(2)+(34.16*omega).powi(2)).sqrt();
        assert!((numerator/denominator/CorrectedYawReference::DC-0.5_f64.sqrt()).abs()<1e-5);
    }

    #[test]
    fn corrected_frequency_against_independent_audited_complex_oracle() {
        // Values computed independently from (-12.65*jw+1084)/
        // ((jw)^2+34.16*jw+975), not from the production state transition.
        let oracle=[(0.5,1.117034483394888,-8.4445017559102),
            (1.0,1.1324018128866664,-17.11516457499752),
            (2.0,1.1870120957186003,-36.05854020859117),
            (4.0,1.2217251875702173,-84.54855160395145),
            (6.4,0.7897042285623694,-140.19020043314427),
            (10.0,0.3665910842021264,179.57853609234994),
            (20.0,0.12473131522523258,140.4482055478175)];
        for unit_dc in [false,true] {for (hz,raw_mag,phase) in oracle {
            let omega=std::f64::consts::TAU*hz;let dt=1.0/(hz*1000.0);
            let mut model=CorrectedYawReference::default();let (mut re,mut im)=(0.0,0.0);
            // Offline midpoint-sampled sine fixture; not a communication delay.
            for k in 0..30000 {
                let t=(k+1) as f64*dt;let input=0.01*(omega*(t-0.5*dt)).cos();
                let output=model.advance(input,dt,100.0,unit_dc).rate;
                if k>=10000 {re+=output*(omega*t).cos();im-=output*(omega*t).sin();}
            }
            re/=100.0;im/=100.0;
            let expected=raw_mag/if unit_dc {CorrectedYawReference::DC}else{1.0};
            let db=20.0*((re*re+im*im).sqrt()/expected).log10();
            let error=(im.atan2(re).to_degrees()-phase+180.0).rem_euclid(360.0)-180.0;
            assert!(db.abs()<0.2 && error.abs()<2.0,"hz={} unit={} dB={} phase={}",hz,unit_dc,db,error);
        }}
    }

    #[test]
    fn corrected_zoh_matches_independent_rk4_augmented_ode() {
        // Independent numerical integration of the original differential
        // equation plus angle. It shares neither expm nor integral formula.
        fn derivative(x:[f64;3],u:f64,unit:bool)->[f64;3] {
            [x[1],u-34.16*x[1]-975.0*x[0],
                (1084.0*x[0]-12.65*x[1])/if unit {1084.0/975.0}else{1.0}]
        }
        fn add(x:[f64;3],d:[f64;3],h:f64)->[f64;3] {std::array::from_fn(|i|x[i]+h*d[i])}
        for unit in [false,true] {
            let mut model=CorrectedYawReference::default();let mut numeric=[0.0;3];let mut angle=0.0;
            for k in 0..1000 {
                let u=if k<200 {1.0}else if k<400 {(k-200) as f64*0.005}else if k<600 {-1.0}else if k<800 {2.0}else{0.0};
                angle+=model.advance(u,0.002,100.0,unit).angle;
                for _ in 0..10 {
                    let h=0.0002;let a=derivative(numeric,u,unit);let b=derivative(add(numeric,a,h/2.0),u,unit);
                    let c=derivative(add(numeric,b,h/2.0),u,unit);let d=derivative(add(numeric,c,h),u,unit);
                    numeric=std::array::from_fn(|i|numeric[i]+h/6.0*(a[i]+2.0*b[i]+2.0*c[i]+d[i]));
                }
                assert!((model.x[0]-numeric[0]).abs()<2e-11);
                assert!((model.x[1]-numeric[1]).abs()<2e-10);
                assert!((angle-numeric[2]).abs()<2e-9);
            }
        }
    }

    #[test]
    fn corrected_saturation_partitions_reset_long_gap_and_identity_selection() {
        assert!(ReferenceModel::from_id("legacy").is_none());
        assert!(ReferenceModel::from_id("rm2017-yaw-corrected").is_none());
        for id in ["rm2017-yaw-corrected-raw","rm2017-yaw-corrected-unitdc"] {
            let model=ReferenceModel::from_id(id).unwrap();let mut angles=Vec::new();
            for hz in [250,500,1000] {
                let mut state=YawReference::default();let mut angle=0.0;
                for k in 0..hz*2 {angle+=model.advance(&mut state,if k<hz {100.0}else{-100.0},1.0/hz as f64,0.7).angle;}
                angles.push(angle);
                state=YawReference::default();let a=model.advance(&mut state,1.0,5.0,100.0);
                let dc=if id.ends_with("-raw") {1.1117948717948718}else{1.0};
                assert!((a.rate-dc).abs()<1e-12);
                state=YawReference::default();assert!(model.advance(&mut state,1.0,0.001,100.0).rate<0.0);
            }
            assert!(angles.iter().all(|a|(a-angles[0]).abs()<1e-9));
        }
    }

    #[test] fn inverse_response_dc_and_integral() {
        let mut m=YawReference::default();let a=m.advance(1.0,1e-6,100.0,true);
        assert!(a.rate<0.0 && a.angle<0.0); assert!((a.rate/1e-6+12.65/YawReference::DC).abs()<0.01);
        let a=m.advance(1.0,2.0,100.0,true);assert!((a.rate-1.0).abs()<1e-12);
        let mut raw=YawReference::default(); assert!((raw.advance(1.0,2.0,100.0,false).rate-YawReference::DC).abs()<1e-12);
    }
    #[test] fn zoh_partition_and_saturation() {
        let mut a=YawReference::default(); let x=a.advance(10.0,0.143,0.7,true);
        let mut b=YawReference::default();let mut angle=0.0;
        for _ in 0..143 {angle+=b.advance(10.0,0.001,0.7,true).angle;}
        assert!((angle-x.angle).abs()<1e-9);assert!(x.limited && x.rate<=0.7);assert!(x.angle.abs()<=0.7*0.143);
        assert!((a.x[0]-b.x[0]).abs()<1e-13);
    }
    #[test] fn analytic_step_oracle_and_narrow_saturation_peak() {
        let a=21.58;let w=(3975.0_f64-a*a).sqrt();let b=-12.65/YawReference::DC;
        for t in [0.000001,0.00852,0.06155,0.1,0.5,5.0] {
            let expected=1.0+(-a*t).exp()*(-(w*t).cos()+(b-a)/w*(w*t).sin());
            let mut model=YawReference::default();let actual=model.advance(1.0,t,100.0,true);
            assert!((actual.rate-expected).abs()<1e-12);
        }
        let aa=-1.0;let bb=(b-a)/w;
        let peak_time=(-(w*bb-a*aa)).atan2(-w*aa-a*bb)/w+std::f64::consts::PI/w;
        let mut m=YawReference::default();m.advance(1.0,peak_time,100.0,true);let peak=m.output(true);
        let limit=peak-1e-9;let mut m=YawReference::default();m.advance(1.0,peak_time-0.00011,100.0,true);
        let step=m.advance(1.0,0.0003,limit,true);
        assert!(step.limited);assert!(step.angle<=limit*0.0003+1e-12);
    }
    #[test] fn reset_and_long_interval() {
        let mut model=YawReference::default();let a=model.advance(1.0,5.0,20.0,true);
        assert!((a.rate-1.0).abs()<1e-12 && a.angle.is_finite());
        model=YawReference::default();assert_eq!(model.x,[0.0,0.0]);
        assert!(model.advance(1.0,0.001,20.0,true).rate<0.0);
    }
    #[test] #[should_panic] fn invalid_is_not_legacy_fallback(){YawReference::default().advance(f64::NAN,0.01,20.0,true);}
}
