//! Why does sim-soft's Newton need several iterations (or fail) on a near-linear aluminium
//! problem? Thin strip (60 x 20 x 2 mm) on sim-soft's own hand-built mesh vs the SDF mesh, and
//! the SDF stocky bar with a soft modulus. `FEM_MAX_ITER` caps Newton.
use cortenforge::cf_design::Solid;
use cortenforge::sim::soft::mesh::HandBuiltTetMesh;
use cortenforge::sim::soft::MaterialField;
use custom_skate::fem::{self, pick, spread};
use nalgebra::Vector3;

fn run(name: &str, m: &fem::Model, l: f64, p: f64, d_exact: f64) {
    let clamp = pick(m, |q| q.x < 1e-6);
    let tip = pick(m, |q| q.x > l - 1e-6);
    match fem::solve(m, clamp, vec![], &spread(&tip, Vector3::new(0.0, 0.0, -p))) {
        Ok(x) => {
            let d = tip.iter().map(|&v| -fem::disp(m, &x, v).z).sum::<f64>() / tip.len() as f64;
            println!("{name}: tip {d:.5} mm = {:.4} of beam theory", d / d_exact);
        }
        Err(e) => println!("{name}: {e}"),
    }
}

fn main() {
    let (e, nu) = (71_700.0, 0.33);
    let (mu, lambda) = fem::lame(e, nu);
    // Thin strip, hand-built (sim-soft's structured mesh), 60 x 20 x 2, 2 N.
    let (l, b, h, p): (f64, f64, f64, f64) = (60.0, 20.0, 2.0, 2.0);
    let i = b * h * h * h / 12.0;
    let d_thin = p * l.powi(3) / (3.0 * e * i);
    let field = MaterialField::uniform(mu, lambda);
    let hb = HandBuiltTetMesh::cantilever_bilayer_beam(60, 20, 2, l, b, h, &field);
    let m = fem::from_tet4(&hb, mu, lambda);
    run("thin, hand-built 60x20x2 cells", &m, l, p, d_thin);
    // Same strip from a cf-design solid.
    let strip = Solid::cuboid(Vector3::new(0.5 * l, 0.5 * b, 0.5 * h)).translate(Vector3::new(0.5 * l, 0.0, 0.0));
    let m = fem::mesh_solid(&strip, 1.0, e, nu).expect("mesh");
    run("thin, SDF cell 1.0", &m, l, p, d_thin);
    // Stocky bar from a solid at a 1000x softer modulus (same displacements at 1000x less load).
    let (l2, h2): (f64, f64) = (100.0, 10.0);
    let bar = Solid::cuboid(Vector3::new(0.5 * l2, 5.0, 5.0)).translate(Vector3::new(0.5 * l2, 0.0, 0.0));
    let d_bar = 20.0 * l2.powi(3) / (3.0 * e * h2.powi(4) / 12.0);
    let m = fem::mesh_solid(&bar, 2.5, e / 1000.0, nu).expect("mesh");
    run("stocky, SDF cell 2.5, E / 1000, load / 1000", &m, l2, 0.02, d_bar);
}
