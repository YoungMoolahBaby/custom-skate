//! The knee's bend as posed: the angle between the thigh's line (knee to hip) and the shin's
//! (ankle to knee), standing and posed, in the world and in the shin's frame.
//! `cargo run --release --example probe_knee_angle -- scans/stand-in/022_fBW_R.ply`

use custom_skate::body;
use custom_skate::pose::LegPose;

fn main() {
    let path = std::env::args().nth(1).expect("scan path");
    let b = body::build(std::path::Path::new(&path), 1250.0).unwrap();
    let shin = b.bone("lowerleg01.R").unwrap();
    let ang = |a: nalgebra::Vector3<f64>, c: nalgebra::Vector3<f64>| a.normalize().dot(&c.normalize()).clamp(-1.0, 1.0).acos().to_degrees();
    let (h0, k0, a0) = (b.joints["r hip"], b.joints["r knee"], b.joints["r ankle"]);
    println!("standing: hip {h0:.1}, knee {k0:.1}, ankle {a0:.1}; thigh line to shin line {:.1} deg", ang(h0 - k0, k0 - a0));
    for (hip, knee) in [(0.0, 0.0), (0.0, 70.0), (60.0, 70.0), (60.0, 0.0), (0.0, 90.0)] {
        let p = LegPose { hip, knee, ankle: 0.0 };
        let (_, j) = b.posed([p, p], 1250.0);
        let mv = b.moves([p, p]);
        let (h, k, a) = (j["r hip"], j["r knee"], j["r ankle"]);
        let inv = mv[shin].inverse();
        let to = |q: nalgebra::Point3<f64>| inv.apply(q.coords);
        println!(
            "hip {hip:>4} knee {knee:>4}: hip {h:.1} knee {k:.1} ankle {a:.1}; thigh to shin {:.1} deg (180 - included); in the shin's frame: knee {:.1}, ankle {:.1}, hip {:.1}; shin's motion turns {:.1} deg",
            ang(h - k, k - a),
            to(k),
            to(a),
            to(h),
            custom_skate::cup::turn_deg(&mv[shin].r)
        );
    }
}
