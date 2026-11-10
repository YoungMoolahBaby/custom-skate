//! Probe: the bottom and top bands of a scan along +z and -z of its own (pre-aligned) frame:
//! how many points each holds and how long each is, to see why the floor-side test flips.
//!
//! `cargo run --release --example probe_band -- <scan>`

use custom_skate::scan;
use nalgebra::Vector3;

fn main() {
    let path = std::env::args().nth(1).expect("scan path");
    let (mesh, _, _) = scan::load(std::path::Path::new(&path)).expect("load");
    for frac in [0.03, 0.06, 0.10] {
        for up in [Vector3::z(), -Vector3::z()] {
            let h = |v: &nalgebra::Point3<f64>| v.coords.dot(&up);
            let (lo, hi) = mesh.vertices.iter().fold((f64::MAX, f64::MIN), |a, v| (a.0.min(h(v)), a.1.max(h(v))));
            let band = frac * (hi - lo);
            let ext = |sel: &dyn Fn(f64) -> bool| {
                let pts: Vec<_> = mesh.vertices.iter().filter(|v| sel(h(v))).collect();
                let (xa, xb) = pts.iter().fold((f64::MAX, f64::MIN), |a, v| (a.0.min(v.x), a.1.max(v.x)));
                let (ya, yb) = pts.iter().fold((f64::MAX, f64::MIN), |a, v| (a.0.min(v.y), a.1.max(v.y)));
                (pts.len(), xb - xa, yb - ya)
            };
            let b = ext(&|x| x < lo + band);
            let t = ext(&|x| x > hi - band);
            println!(
                "band {:>4.0}%  up {:+}z: bottom {:>5} pts {:5.1} x {:5.1} mm | top {:>5} pts {:5.1} x {:5.1} mm",
                frac * 100.0,
                up.z,
                b.0,
                b.1,
                b.2,
                t.0,
                t.1,
                t.2
            );
        }
    }
}
