//! Probe: why does `mesh_measure::cross_section` split a cf-design ellipsoid's section into
//! many loops? Checks vertex sharing, duplicate faces, edge manifoldness, and the section count
//! before and after welding.

use std::collections::HashMap;

use cortenforge::cf_design::Solid;
use cortenforge::mesh::measure::cross_section;
use cortenforge::mesh::repair::weld_vertices;
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Vector3};

fn report(name: &str, mesh: &IndexedMesh) {
    let mut edges: HashMap<(u32, u32), usize> = HashMap::new();
    for f in &mesh.faces {
        for (a, b) in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])] {
            *edges.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    let mut by_count: HashMap<usize, usize> = HashMap::new();
    for c in edges.values() {
        *by_count.entry(*c).or_default() += 1;
    }
    let degenerate = mesh.faces.iter().filter(|f| f[0] == f[1] || f[1] == f[2] || f[0] == f[2]).count();
    let mut counts: Vec<_> = by_count.into_iter().collect();
    counts.sort();
    println!(
        "{name}: {} vertices, {} faces, degenerate faces {degenerate}, edges by face count {:?}",
        mesh.vertices.len(),
        mesh.faces.len(),
        counts
    );
    for (label, normal) in [("x = 0", Vector3::x()), ("x = 0.37", Vector3::x()), ("tilted", Vector3::new(1.0, 0.2, 0.1))] {
        let p = if label == "x = 0.37" { Point3::new(0.37, 0.0, 0.0) } else { Point3::origin() };
        let cs = cross_section(mesh, p, normal);
        println!(
            "  section {label}: {} loop(s), perimeter {:.2}, area {:.0}, {} points",
            cs.contour_count,
            cs.perimeter,
            cs.area,
            cs.points.len()
        );
    }
}

fn main() {
    let raw = Solid::ellipsoid(Vector3::new(130.0, 50.0, 40.0)).mesh(1.0).geometry;
    report("cf-design mesh(1.0)", &raw);
    let mut welded = raw.clone();
    let merged = weld_vertices(&mut welded, 1e-6);
    println!("weld_vertices(1e-6) merged {merged}");
    report("welded", &welded);
    let exact = {
        let (b, c) = (50.0_f64, 40.0_f64);
        let h = ((b - c) / (b + c)).powi(2);
        std::f64::consts::PI * (b + c) * (1.0 + 3.0 * h / (10.0 + (4.0 - 3.0 * h).sqrt()))
    };
    println!("exact perimeter at x = 0: {exact:.2}, area {:.0}", std::f64::consts::PI * 50.0 * 40.0);
}
