//! Where the cup's knee cap comes closest to the top over it, as built (`out/pad`, the leaned shin
//! frame at the build pose): the cap's vertices nearest to or inside the top.
//! `cargo run --release --example probe_shell_touch`

use cortenforge::mesh::io::load_mesh;
use custom_skate::pad::{NearMesh, Part};

fn main() {
    let load = |p: Part| load_mesh(format!("out/pad/{}", custom_skate::cup::stl_name(p))).unwrap();
    let (cap, top) = (load(Part::KneeCap), load(Part::KneeTop));
    let f = NearMesh::new(&top, "top").unwrap();
    let mut d: Vec<(f64, nalgebra::Point3<f64>)> = cap.vertices.iter().map(|v| (f.eval(*v), *v)).collect();
    d.sort_by(|a, b| a.0.total_cmp(&b.0));
    let inside = d.iter().filter(|x| x.0 < 0.0).count();
    println!("{} cap vertices, {inside} inside the top; the closest:", cap.vertices.len());
    // The knee joint in this frame is (103, 18, 466); +x forward, +y the inside of the right leg.
    for (e, v) in d.iter().take(12) {
        let th = (v.y - 18.0).atan2(v.x - 103.0).to_degrees();
        println!("  {e:+.2} mm at ({:.0}, {:.0}, {:.0}): {:+.0} mm from the knee joint's height, {th:+.0} deg round from the front", v.x, v.y, v.z, v.z - 466.0);
    }
}
