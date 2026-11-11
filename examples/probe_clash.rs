//! Where do two round-2 parts overlap? Volume and centre of each overlap, at rest and rocked.
use cortenforge::cf_design::Solid;
use cortenforge::cf_design::mechanism::mass::mass_properties;
use custom_skate::frame::{self, FrameParams, rolled};

fn main() {
    let p = FrameParams::default();
    let f = frame::build(0.0, 252.4, p);
    let l = f.layout;
    let ties = f.ties.clone().unwrap();
    let parts: [(&str, &Solid); 3] = [("upper", &f.upper), ("ties", &ties), ("bolt", &f.bolt)];
    for e in [0.0, p.travel, -p.travel] {
        for (name, s) in parts {
            let up = rolled(s, &l, p.rocker_r, e);
            if let Some(m) = mass_properties(&up.intersect(f.lower.clone()), 1000.0, 0.1) {
                let c = m.center_of_mass;
                println!(
                    "e {e:+5.0} {name:<6} x lower: {:6.2} mm3 at x {:+6.1} (from centre), y {:+5.1}, z {:+5.1} (from axle line)",
                    m.mass * 1e6,
                    c.x - l.xc,
                    c.y,
                    c.z - l.axle_z
                );
            }
        }
    }
    println!("ledge top z {:+.1}, rail top {:+.1}, web underside {:+.1} (from axle line); y: rail {:.1}-{:.1}, wall {:.1}-{:.1}",
        l.z_t - l.axle_z, l.rail_top - l.axle_z, l.z_wb - l.axle_z, l.y_in, l.y_out, l.y_wi, l.y_wo);
}
