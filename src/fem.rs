//! Static FEM on chassis parts with sim-soft, CortenForge's soft-body FEM.
//!
//! sim-soft is hyperelastic (Neo-Hookean); at the small strains of an aluminium frame
//! (~1e-3) Neo-Hookean is linear elasticity, so it serves as a linear-elastic solver here.
//! The pipeline, all CortenForge:
//! - a cf-design `Solid` (it implements the shared `Sdf` trait) -> `SdfMeshedTetMesh` (BCC
//!   lattice + isosurface stuffing) -> `Tet10Mesh` (quadratic tets: linear tets lock in bending,
//!   quadratic ones don't; sim-soft's own `slender_bending_matches_analytic` test);
//! - `CpuNewtonSolver` with density 0, so the backward-Euler step is a static solve (that test's
//!   lesson: a mass term at dt = 1 stiffens a compliant structure);
//! - stresses recovered here (sim-soft's `StressField` observable is still a stub): per
//!   element, F from the Tet10 shape gradients, P from sim-soft's own `NeoHookean::first_piola`,
//!   Cauchy stress = P F^T / det F, evaluated at the element's four corners (the strain is linear
//!   in a Tet10, so the corners carry the surface values).
//!
//! Units: mm, N, MPa (consistent; density is 0, so no mass unit is needed).

use std::time::Instant;

use cortenforge::cf_design::Solid;
use cortenforge::sim::ml_chassis::Tensor;
use cortenforge::sim::soft::element::{Element, Tet10};
use cortenforge::sim::soft::material::Material;
use cortenforge::sim::soft::{
    Aabb3, BoundaryConditions, CpuTet10NHSolver, LoadAxis, MaterialField, Mesh, MeshingHints, NeoHookean, NullContact,
    SdfMeshedTetMesh, Solver, SolverConfig, SolverFailure, Tet10Mesh, Vec3,
};
use nalgebra::{Matrix3, SMatrix, Vector3};

/// Lamé parameters from Young's modulus and Poisson's ratio.
pub fn lame(e: f64, nu: f64) -> (f64, f64) {
    (e / (2.0 * (1.0 + nu)), e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu)))
}

/// A Tet10 mesh of a solid, with what stress recovery needs kept outside the solver.
pub struct Model {
    pub mesh: Tet10Mesh,
    pub rest: Vec<Vec3>,
    /// Ten node ids per element: corners, then midsides in sim-soft's TET10_EDGE_NODES order.
    pub elems: Vec<[u32; 10]>,
    /// Vertices some element uses (the BCC lattice leaves unused corners, which sim-soft pins).
    pub used: Vec<bool>,
    pub mu: f64,
    pub lambda: f64,
}

/// Mesh `solid` with lattice spacing `cell` (mm) in a uniform material.
pub fn mesh_solid(solid: &Solid, cell: f64, e: f64, nu: f64) -> Result<Model, String> {
    let t = Instant::now();
    let b = solid.bounds().ok_or("solid has no bounds")?;
    let pad = 2.0 * cell;
    let (mu, lambda) = lame(e, nu);
    let hints = MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(b.min.x - pad, b.min.y - pad, b.min.z - pad),
            Vec3::new(b.max.x + pad, b.max.y + pad, b.max.z + pad),
        ),
        cell_size: cell,
        material_field: Some(MaterialField::uniform(mu, lambda)),
    };
    let m4: SdfMeshedTetMesh = SdfMeshedTetMesh::from_sdf(solid, &hints).map_err(|e| format!("meshing: {e:?}"))?;
    let m4 = m4.largest_component();
    let model = from_tet4(&m4, mu, lambda);
    println!("  (meshed in {:.1} s)", t.elapsed().as_secs_f64());
    Ok(model)
}

/// A Tet10 model from any linear tet mesh (an SDF mesh, or sim-soft's hand-built ones).
pub fn from_tet4(m4: &dyn Mesh, mu: f64, lambda: f64) -> Model {
    let t = Instant::now();
    let mesh = Tet10Mesh::from_tet4(m4);
    let rest = mesh.positions().to_vec();
    let mut used = vec![false; rest.len()];
    let elems: Vec<[u32; 10]> = (0..mesh.n_tets())
        .map(|i| {
            let c = mesh.tet_vertices(i as u32);
            let m = mesh.tet_midside_nodes(i as u32).expect("Tet10 midsides");
            let mut n = [0u32; 10];
            n[..4].copy_from_slice(&c);
            n[4..].copy_from_slice(&m);
            for &v in &n {
                used[v as usize] = true;
            }
            n
        })
        .collect();
    let q = mesh.quality();
    let min_aspect = q.aspect_ratio.iter().cloned().fold(f64::INFINITY, f64::min);
    let min_dihedral = q.dihedral_min.iter().cloned().fold(f64::INFINITY, f64::min).to_degrees();
    let slivers = q.aspect_ratio.iter().filter(|&&a| a < 0.02).count();
    println!(
        "  fem mesh: {} Tet10, {} nodes ({} used); worst tet: aspect {min_aspect:.3} (regular 0.333), dihedral {min_dihedral:.1} deg, {slivers} below 0.02 ({:.1} s)",
        elems.len(),
        rest.len(),
        used.iter().filter(|&&u| u).count(),
        t.elapsed().as_secs_f64()
    );
    Model { mesh, rest, elems, used, mu, lambda }
}

/// Load fraction the solve runs at; the displacements are scaled back by its inverse.
///
/// sim-soft solves finite strain. Its first Newton step from rest is the linear solution, which
/// moves points along straight lines rather than arcs, so a slender part also stretches by
/// ~theta^2 / 2 (theta its slope). On a thin aluminium part that stretch, times the large
/// in-plane stiffness, makes a residual several times the load, the residual-norm line search
/// rejects the full step, and Newton crawls (a 60 x 20 x 2 mm strip at 2 N: no convergence in
/// 50 iterations; a 100 x 10 x 10 bar at 20 N: 5 iterations). That residual grows with the
/// load squared while the answer is linear in it, so solving at a small fraction of the load
/// and scaling back gives the linear-elastic answer, which is the one wanted for a metal frame
/// at strains ~1e-3.
pub const LOAD_SCALE: f64 = 1e-3;

/// Static linear-elastic solve: `pinned` vertices fixed, `rollers` fixed on the masked axes,
/// `loads` as force vectors (N) on free vertices. Runs at `LOAD_SCALE` of the load and returns
/// the deformed positions at the full load (rest + displacement / LOAD_SCALE), flattened xyz.
pub fn solve(
    model: &Model,
    pinned: Vec<u32>,
    rollers: Vec<(u32, [bool; 3])>,
    loads: &[(u32, Vector3<f64>)],
) -> Result<Vec<f64>, String> {
    solve_with_reactions(model, pinned, rollers, loads).map(|(x, _)| x)
}

/// As `solve`, plus the force the part exerts on each support, per DOF (N, at the full load):
/// sim-soft's `nodal_reaction_forces` (-f_int, exact for a static solve with no load on the
/// supported nodes). Only the held components of a roller mean anything.
pub fn solve_with_reactions(
    model: &Model,
    pinned: Vec<u32>,
    rollers: Vec<(u32, [bool; 3])>,
    loads: &[(u32, Vector3<f64>)],
) -> Result<(Vec<f64>, Vec<f64>), String> {
    let t = Instant::now();
    let n_dof = 3 * model.rest.len();
    let mut x0 = vec![0.0; n_dof];
    for (v, p) in model.rest.iter().enumerate() {
        x0[3 * v] = p.x;
        x0[3 * v + 1] = p.y;
        x0[3 * v + 2] = p.z;
    }
    let scale = std::env::var("FEM_LOAD_SCALE").ok().and_then(|v| v.parse().ok()).unwrap_or(LOAD_SCALE);
    let total: f64 = scale * loads.iter().map(|(_, f)| f.norm()).sum::<f64>();
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = 1.0;
    cfg.density = 0.0;
    // Linear at the scaled load: Newton should take one or two steps.
    cfg.max_newton_iter = std::env::var("FEM_MAX_ITER").ok().and_then(|s| s.parse().ok()).unwrap_or(12);
    let rel_tol: f64 = std::env::var("FEM_TOL").ok().and_then(|v| v.parse().ok()).unwrap_or(1e-6);
    // The residual has an absolute round-off floor near 1e-7 N (measured on the cantilever).
    cfg.tol = (rel_tol * total).max(1e-6);
    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: rollers,
        loaded_vertices: loads.iter().map(|&(v, _)| (v, LoadAxis::FullVector)).collect(),
    };
    let theta: Vec<f64> = loads.iter().flat_map(|(_, f)| [scale * f.x, scale * f.y, scale * f.z]).collect();
    let n_theta = theta.len();
    let solver: CpuTet10NHSolver<Tet10Mesh> =
        cortenforge::sim::soft::solver::CpuNewtonSolver::new(Tet10, model.mesh.clone(), NullContact, cfg, bc);
    let step = solver
        .try_replay_step(
            &Tensor::from_slice(&x0, &[n_dof]),
            &Tensor::zeros(&[n_dof]),
            &Tensor::from_slice(&theta, &[n_theta]),
            cfg.dt,
        )
        .map_err(|e| match e {
            SolverFailure::ArmijoStall { last_iter, last_r_norm, .. } => {
                format!("solve: line search stalled at iteration {last_iter}, residual {last_r_norm:.3e} N (tol {:.1e})", cfg.tol)
            }
            SolverFailure::NewtonIterCap { max_iter, last_r_norm, .. } => {
                format!("solve: {max_iter} Newton iterations, residual {last_r_norm:.3e} N (tol {:.1e})", cfg.tol)
            }
            other => format!("solve: {}", format!("{other:?}").chars().take(300).collect::<String>()),
        })?;
    println!(
        "  fem solve: {} dof, load x {scale:e}, {} Newton iterations, residual {:.1e} of the load ({:.1} s)",
        n_dof,
        step.iter_count,
        step.final_residual_norm / total,
        t.elapsed().as_secs_f64()
    );
    let reactions: Vec<f64> = solver.nodal_reaction_forces(&step.x_final, &x0, cfg.dt).iter().map(|r| r / scale).collect();
    Ok((step.x_final.iter().zip(&x0).map(|(x, r)| r + (x - r) / scale).collect(), reactions))
}

/// Cauchy stress at the four corners of every element.
pub fn corner_stresses(model: &Model, x: &[f64]) -> Vec<[Matrix3<f64>; 4]> {
    let mat = NeoHookean::from_lame(model.mu, model.lambda);
    let corners = [Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)];
    let grads: Vec<SMatrix<f64, 10, 3>> = corners.iter().map(|&xi| Tet10.shape_gradients(xi)).collect();
    model
        .elems
        .iter()
        .map(|n| {
            let xr = SMatrix::<f64, 10, 3>::from_fn(|a, k| model.rest[n[a] as usize][k]);
            let xd = SMatrix::<f64, 10, 3>::from_fn(|a, k| x[3 * n[a] as usize + k]);
            std::array::from_fn(|c| {
                let g = &grads[c];
                let j_rest = xr.transpose() * g;
                let j_def = xd.transpose() * g;
                let f = j_def * j_rest.try_inverse().unwrap_or_else(Matrix3::identity);
                let p = mat.first_piola(&f);
                p * f.transpose() / f.determinant()
            })
        })
        .collect()
}

pub fn von_mises(s: &Matrix3<f64>) -> f64 {
    let (a, b, c) = (s[(0, 0)], s[(1, 1)], s[(2, 2)]);
    (0.5 * ((a - b).powi(2) + (b - c).powi(2) + (c - a).powi(2))
        + 3.0 * (s[(0, 1)].powi(2) + s[(1, 2)].powi(2) + s[(0, 2)].powi(2)))
    .sqrt()
}

/// Vertices (used by some element) whose rest position satisfies `pred`.
pub fn pick(model: &Model, pred: impl Fn(&Vec3) -> bool) -> Vec<u32> {
    model.rest.iter().enumerate().filter(|(v, p)| model.used[*v] && pred(p)).map(|(v, _)| v as u32).collect()
}

/// Spread a force over vertices in proportion to equal shares (a resultant, Saint-Venant).
pub fn spread(vertices: &[u32], force: Vector3<f64>) -> Vec<(u32, Vector3<f64>)> {
    let n = vertices.len().max(1) as f64;
    vertices.iter().map(|&v| (v, force / n)).collect()
}

/// Displacement of vertex `v`.
pub fn disp(model: &Model, x: &[f64], v: u32) -> Vector3<f64> {
    let i = 3 * v as usize;
    Vector3::new(x[i], x[i + 1], x[i + 2]) - model.rest[v as usize]
}

/// Stress at each corner vertex, averaged over the elements that share it (nodal averaging
/// smooths the element-by-element jumps of a Tet10's linear strain). `None` for vertices that
/// are no element's corner (midsides, unused lattice points).
pub fn nodal_stresses(model: &Model, x: &[f64]) -> Vec<Option<Matrix3<f64>>> {
    let st = corner_stresses(model, x);
    let mut sum = vec![Matrix3::zeros(); model.rest.len()];
    let mut count = vec![0u32; model.rest.len()];
    for (e, s4) in model.elems.iter().zip(&st) {
        for c in 0..4 {
            sum[e[c] as usize] += s4[c];
            count[e[c] as usize] += 1;
        }
    }
    sum.into_iter().zip(count).map(|(s, n)| (n > 0).then(|| s / n as f64)).collect()
}

/// Volume of the meshed solid (mm3), from the elements' corner tetrahedra.
pub fn volume(model: &Model) -> f64 {
    model
        .elems
        .iter()
        .map(|n| {
            let [a, b, c, d] = [0, 1, 2, 3].map(|k| model.rest[n[k] as usize]);
            ((b - a).cross(&(c - a)).dot(&(d - a)) / 6.0).abs()
        })
        .sum()
}
