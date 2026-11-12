//! Where the boot's top parts are, for the shin pad (track D2): extents of the tongue, laces,
//! eyelets and tendon guard, and the shell's top edge round the leg.
//! `cargo run --release --example probe_boot_top [out/boot]`

use cortenforge::mesh::io::load_mesh;

fn main() {
    let dir = std::env::args().nth(1).unwrap_or_else(|| "out/boot".into());
    for name in ["tongue_plate", "tongue_lattice", "laces", "eyelets", "tendon_guard", "shell", "liner"] {
        let m = load_mesh(format!("{dir}/{name}.stl")).expect("load");
        let (lo, hi) = m.vertices.iter().fold(([f64::MAX; 3], [f64::MIN; 3]), |(mut lo, mut hi), v| {
            for k in 0..3 {
                lo[k] = lo[k].min(v[k]);
                hi[k] = hi[k].max(v[k]);
            }
            (lo, hi)
        });
        println!(
            "{name:<15} {:>7} tris  x {:>6.1}..{:>6.1}  y {:>6.1}..{:>6.1}  z {:>6.1}..{:>6.1}",
            m.faces.len(),
            lo[0],
            hi[0],
            lo[1],
            hi[1],
            lo[2],
            hi[2]
        );
        if name == "shell" || name == "laces" || name == "tongue_plate" {
            // The top edge round the leg: the highest vertex in 20-degree bins about the leg's
            // centre at the cuff (58, 11), leaned 6 deg (x moves +0.105 mm per mm above ~100).
            let mut top = [f64::MIN; 18];
            let mut at = [(0.0, 0.0); 18];
            for v in &m.vertices {
                if v.z < 90.0 {
                    continue;
                }
                let cx = 58.0 + (v.z - 81.0).max(0.0) * 0.105;
                let (dx, dy) = (v.x - cx, v.y - 11.0);
                let th = dy.atan2(dx).to_degrees();
                let b = (((th + 180.0) / 20.0) as usize).min(17);
                if v.z > top[b] {
                    top[b] = v.z;
                    at[b] = (v.x, v.y);
                }
            }
            let s: Vec<String> = (0..18)
                .filter(|&b| top[b] > f64::MIN)
                .map(|b| format!("{:+}: z {:.0} at ({:.0}, {:.0})", -180 + 20 * b as i32 + 10, top[b], at[b].0, at[b].1))
                .collect();
            println!("  top by angle (0 = front, + = medial/+y): {}", s.join("; "));
        }
    }
}
