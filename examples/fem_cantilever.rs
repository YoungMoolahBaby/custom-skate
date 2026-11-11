//! sim-soft on an aluminium cantilever meshed from a cf-design solid, against beam theory:
//! tip deflection vs P L^3 / 3EI + P L / (k G A), mid-span bending stress vs M c / I.
use cortenforge::cf_design::Solid;
use custom_skate::fem::{self, corner_stresses, pick, spread, von_mises};
use custom_skate::materials::AL_7075_T6_STRENGTH as AL;
use nalgebra::Vector3;

fn main() {
    // `CANTILEVER=thin`: a 2 mm strip bent across its thickness, like a wall pushed sideways.
    let thin = std::env::var("CANTILEVER").is_ok_and(|v| v == "thin");
    let (l, b, h) = if thin { (60.0, 20.0, 2.0) } else { (100.0, 10.0, 10.0) };
    let bar = Solid::cuboid(Vector3::new(0.5 * l, 0.5 * b, 0.5 * h)).translate(Vector3::new(0.5 * l, 0.0, 0.0));
    let i = b * h * h * h / 12.0;
    let g = AL.e / (2.0 * (1.0 + AL.nu));
    let p = if thin { 2.0 } else { 20.0 };
    let d_eb = p * l.powi(3) / (3.0 * AL.e * i);
    let d_t = d_eb + p * l / (5.0 / 6.0 * g * b * h);
    let s_mid = p * 0.5 * l * 0.5 * h / i;
    println!("analytic: tip {d_eb:.5} mm (Euler-Bernoulli), {d_t:.5} mm (+ shear); mid-span sigma_xx {s_mid:.3} MPa");
    let cells: Vec<f64> = std::env::args().skip(1).filter_map(|s| s.parse().ok()).collect();
    for cell in if cells.is_empty() { vec![2.5, 2.0, 1.5] } else { cells } {
        let m = fem::mesh_solid(&bar, cell, AL.e, AL.nu).expect("mesh");
        let clamp = pick(&m, |q| q.x < 1e-3);
        let tip = pick(&m, |q| q.x > l - 1e-3);
        let x = match fem::solve(&m, clamp.clone(), vec![], &spread(&tip, Vector3::new(0.0, 0.0, -p))) {
            Ok(x) => x,
            Err(e) => {
                println!("cell {cell}: {e}");
                continue;
            }
        };
        let d: f64 = tip.iter().map(|&v| -fem::disp(&m, &x, v).z).sum::<f64>() / tip.len() as f64;
        let ns = fem::nodal_stresses(&m, &x);
        // Surface vertices at mid-span, top and bottom faces: mean and spread of |sigma_xx|.
        let mid: Vec<f64> = m
            .rest
            .iter()
            .zip(&ns)
            .filter(|(q, s)| s.is_some() && (q.x - 0.5 * l).abs() < 0.6 * cell && q.z.abs() > 0.5 * h - 1e-3)
            .map(|(_, s)| s.unwrap()[(0, 0)].abs())
            .collect();
        let s_top = mid.iter().sum::<f64>() / mid.len().max(1) as f64;
        let s_hi = mid.iter().cloned().fold(0.0, f64::max);
        let vm_max = m
            .rest
            .iter()
            .zip(&ns)
            .filter(|(q, s)| s.is_some() && q.x > 2.0 * cell)
            .map(|(_, s)| von_mises(&s.unwrap()))
            .fold(0.0, f64::max);
        let vol = fem::volume(&m);
        let _ = corner_stresses;
        println!(
            "cell {cell}: clamp {} / tip {} vertices; tip {d:.5} mm = {:.4} of EB+shear; volume {vol:.0} mm3 (exact {:.0}); mid-span surface |sigma_xx| mean {s_top:.3} MPa = {:.4} of Mc/I (max {s_hi:.3}, n {}); max von Mises away from the clamp {vm_max:.2} MPa (root Mc/I {:.2})",
            clamp.len(), tip.len(), d / d_t, l * b * h, s_top / s_mid, mid.len(), p * l * 0.5 * h / i
        );
    }
}
