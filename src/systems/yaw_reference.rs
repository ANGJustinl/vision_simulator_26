//! RM2017 closed-loop commanded yaw velocity -> actual yaw velocity.
//! Original numerator [-12.65,1084], denominator [1,43.16,3975].
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
#[cfg(test)]
mod tests {
    use super::*;
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
