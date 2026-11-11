//! Probe: the ankle region of a scan, extreme y per height band (to find the malleoli).
use custom_skate::{foot, scan};
fn main() {
    let path = std::env::args().nth(1).expect("scan");
    let (raw, _, _) = scan::load(std::path::Path::new(&path)).unwrap();
    let m = scan::measure(&raw, 20);
    let mesh = foot::canonical(&raw, &m);
    let l = m.length;
    println!("length {l:.1}");
    for z0 in (20..=120).step_by(5) {
        let z0 = z0 as f64;
        let (mut ymax, mut ymin) = ((f64::MIN, 0.0), (f64::MAX, 0.0));
        let (mut xmin, mut xmax) = (f64::MAX, f64::MIN);
        for v in &mesh.vertices {
            if v.z >= z0 && v.z < z0 + 5.0 && v.x < 0.5 * l {
                if v.y > ymax.0 { ymax = (v.y, v.x); }
                if v.y < ymin.0 { ymin = (v.y, v.x); }
                xmin = xmin.min(v.x);
                xmax = xmax.max(v.x);
            }
        }
        println!("z {z0:5.0}-{:3.0}: +y {:6.1} at x {:5.1} | -y {:6.1} at x {:5.1} | x {:5.1}..{:5.1}", z0 + 5.0, ymax.0, ymax.1, ymin.0, ymin.1, xmin, xmax);
    }
}
