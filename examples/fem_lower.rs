//! The round-2 lower frame in sim-soft under the stride load with a sideways push (half model,
//! symmetric about the contact at the centre). `cargo run --release --example fem_lower -- <cell> <lateral ratio>`
//!
//! Loads: four wheels share the stride load (rigid frame, contact at the centre: N/4 each); each
//! wheel's sideways ground force (ratio x its load) acts 36 mm below the axle, so it reaches
//! the rails at the axle as a sideways force (half per rail) and a roll couple (up on one rail,
//! down on the other). Supports: the rocker contact on both ledges (a 5 mm patch, vertical),
//! the upper's wall bearing on the +y rail's outer face just above the ledge (sideways), and
//! the symmetry plane at the centre (along x).
use std::collections::BTreeMap;

use cortenforge::cf_design::Solid;
use custom_skate::fem::{self, nodal_stresses, von_mises};
use custom_skate::frame::{self, FrameParams};
use custom_skate::materials::AL_7075_T6_STRENGTH as AL;
use custom_skate::round2::STAND_IN_LENGTH;
use custom_skate::strength::LOADS;
use nalgebra::Vector3;

fn main() {
    let args: Vec<f64> = std::env::args().skip(1).filter_map(|s| s.parse().ok()).collect();
    let cell = args.first().copied().unwrap_or(1.5);
    let lam = args.get(1).copied().unwrap_or(LOADS.lateral);
    let mut p = FrameParams::default();
    if let Some(t) = std::env::var("FEM_RAIL_T").ok().and_then(|v| v.parse().ok()) {
        p.rail_t = t;
    }
    let f = frame::build(0.0, STAND_IN_LENGTH, p);
    let l = f.layout;
    // The axles are clamped through both rails and tie them together: bonded in, as one body.
    // FEM_FULL=1: the whole lower (no symmetry cut), held along x at the contact patch instead.
    let full = std::env::var("FEM_FULL").is_ok();
    // FEM_PART=onepiece: the one-piece from the same members, clamped at its two mount tops.
    let onepiece = std::env::var("FEM_PART").is_ok_and(|v| v == "onepiece");
    let both = if onepiece {
        frame::one_piece(0.0, STAND_IN_LENGTH, &p, 165.0, 36.0, 4.0).union(f.axles.clone())
    } else {
        f.lower.clone().union(f.axles.clone())
    };
    let half = if full {
        both
    } else {
        both.intersect(Solid::cuboid(Vector3::new(100.0, 50.0, 100.0)).translate(Vector3::new(l.xc + 100.0, 0.0, l.axle_z)))
    };
    let m = fem::mesh_solid(&half, cell, AL.e, AL.nu).expect("mesh");
    let vol = fem::volume(&m);
    let exact = cortenforge::cf_design::mechanism::mass::mass_properties(&half, 1000.0, 0.25).map_or(0.0, |q| q.mass * 1e6);
    println!("  meshed volume {vol:.0} mm3 vs the solid {exact:.0} ({:+.1}%)", (vol / exact - 1.0) * 100.0);

    let n = LOADS.stride * LOADS.bw();
    let r_w = 0.5 * p.wheel_d;
    let tol = 1e-3;
    // Rollers, merged per vertex.
    let mut mask: BTreeMap<u32, [bool; 3]> = BTreeMap::new();
    let mut add = |vs: Vec<u32>, axis: usize| {
        for v in vs {
            mask.entry(v).or_insert([false; 3])[axis] = true;
        }
    };
    let sym = if full { Vec::new() } else { fem::pick(&m, |q| (q.x - l.xc).abs() < tol) };
    let patch = fem::pick(&m, |q| q.z > l.z_t - tol && q.y.abs() > l.y_wi && q.y.abs() < l.y_wo && (q.x - l.xc).abs() < 5.0);
    if full {
        add(patch.clone(), 0);
    }
    // FEM_BEAR=one: only the wall the push drives the frame into bears (+y). Default: both walls
    // snug on both rails along the upper's length (the no-play design: a preloaded liner).
    let one = std::env::var("FEM_BEAR").is_ok_and(|v| v == "one");
    let bearing = fem::pick(&m, |q| {
        let face = if one { (q.y - l.y_out).abs() < tol } else { (q.y.abs() - l.y_out).abs() < tol };
        face && q.z > l.z_t - tol && q.z < l.z_t + p.wall_bot_h && q.x < l.xu1 && q.x > l.xu0
    });
    println!("  supports: symmetry {} nodes, contact patch {}, wall bearing {}", sym.len(), patch.len(), bearing.len());
    let mut pinned: Vec<u32> = Vec::new();
    if onepiece {
        let mounts = fem::pick(&m, |q| q.z > l.z_s - tol && ((q.x - l.xc).abs() - 82.5).abs() < 18.0);
        println!("  one-piece: {} nodes clamped on the mount tops", mounts.len());
        pinned = mounts;
    } else {
        add(sym.clone(), 0);
        add(patch.clone(), 2);
        add(bearing.clone(), 1);
    }
    // Loads on the two axles of this half, at the wheel's two bearings (608, 7 mm wide, centres
    // +-(wheel width / 2 - 3.5) mm): the wheel's ground force (up N/4, sideways ratio x that)
    // acting 36 mm below the axle is, at the axle, the same forces plus a roll couple, carried
    // by the two bearings as unequal vertical loads.
    let mut loads: Vec<(u32, Vector3<f64>)> = Vec::new();
    let r_axle = 0.5 * p.axle_d;
    let y_b = 0.5 * p.wheel_w - 3.5;
    let axles: Vec<f64> = if full { l.axle_x.to_vec() } else { l.axle_x[2..].to_vec() };
    for &ax in &axles {
        let w = n / 4.0;
        let side = lam * w;
        let dv = side * r_w / (2.0 * y_b);
        for (sign, fz) in [(1.0, 0.5 * w + dv), (-1.0, 0.5 * w - dv)] {
            let seat: Vec<u32> = fem::pick(&m, |q| {
                ((q.x - ax).hypot(q.z - l.axle_z) - r_axle).abs() < 0.05 && (q.y - sign * y_b).abs() < 3.5
            })
            .into_iter()
            .filter(|v| !mask.contains_key(v))
            .collect();
            println!("  axle x {:+.0}, bearing at y {:+.1}: {} nodes, force (0, {:.0}, {:.0}) N", ax - l.xc, sign * y_b, seat.len(), 0.5 * side, fz);
            loads.extend(fem::spread(&seat, Vector3::new(0.0, 0.5 * side, fz)));
        }
    }
    let rollers: Vec<(u32, [bool; 3])> = mask.into_iter().collect();
    let pinned_set: std::collections::BTreeSet<u32> = pinned.iter().copied().collect();
    let loads: Vec<(u32, Vector3<f64>)> = loads.into_iter().filter(|(v, _)| !pinned_set.contains(v)).collect();
    let x = fem::solve(&m, pinned, rollers, &loads).expect("solve");

    // Stress: nodal von Mises; the worst away from the load and support patches.
    let ns = nodal_stresses(&m, &x);
    let near_bc = |q: &nalgebra::Vector3<f64>| {
        l.axle_x.iter().any(|&ax| (q.x - ax).hypot(q.z - l.axle_z) < r_axle + 1.5 && q.y.abs() < l.y_in) || (q.z > l.z_t - 1.5 && (q.x - l.xc).abs() < 7.0 && q.y.abs() > l.y_out)
    };
    let mut vm: Vec<(f64, nalgebra::Vector3<f64>)> =
        m.rest.iter().zip(&ns).filter_map(|(q, s)| s.map(|s| (von_mises(&s), *q))).collect();
    vm.sort_by(|a, b| b.0.total_cmp(&a.0));
    println!("  top von Mises (MPa) at the stride, x from the centre, z from the axle line:");
    for (s, q) in vm.iter().filter(|(_, q)| !near_bc(q)).take(8) {
        println!("    {s:6.1} at x {:+6.1}, y {:+5.1}, z {:+5.1}", q.x - l.xc, q.y, q.z - l.axle_z);
    }
    if std::env::var("FEM_DEBUG").is_ok() {
        let (s0, q0) = vm[0];
        let v0 = m.rest.iter().position(|q| (q - q0).norm() < 1e-9).unwrap() as u32;
        let shared = m.elems.iter().filter(|e| e[..4].contains(&v0)).count();
        println!("  DEBUG worst node {v0} vm {s0:.1}: displacement {:?}, shared by {shared} elements as a corner", fem::disp(&m, &x, v0).as_slice());
        println!("  stress {:?}", ns[v0 as usize].unwrap().as_slice());
        let st = fem::corner_stresses(&m, &x);
        for (e, s4) in m.elems.iter().zip(&st) {
            if let Some(c) = e[..4].iter().position(|&v| v == v0) {
                let vol = { let [a, b, c2, d] = [0, 1, 2, 3].map(|k| m.rest[e[k] as usize]); ((b - a).cross(&(c2 - a)).dot(&(d - a)) / 6.0).abs() };
                println!("    element corner vm {:.1}, volume {vol:.4} mm3", von_mises(&s4[c]));
            }
        }
        // Neighbours within 2 mm: their displacement z.
        for (v, q) in m.rest.iter().enumerate() {
            if m.used[v] && (q - q0).norm() < 1.6 && ns[v].is_some() {
                println!("    nbr {:+.2} {:+.2} {:+.2}: dz {:+.5} dy {:+.5}, vm {:.1}", q.x - q0.x, q.y - q0.y, q.z - q0.z, fem::disp(&m, &x, v as u32).z, fem::disp(&m, &x, v as u32).y, von_mises(&ns[v].unwrap()));
            }
        }
    }
    let worst_any = vm.first().map_or(0.0, |v| v.0);
    println!("  (worst including the load/support patches: {worst_any:.1} MPa)");
    // Sideways deflection at the axle line, the rail bottom.
    let maxdy = m.rest.iter().enumerate().filter(|(v, q)| m.used[*v] && (q.z - l.axle_z).abs() < 1.0).map(|(v, _)| fem::disp(&m, &x, v as u32).y).fold(0.0f64, |a, b| a.max(b.abs()));
    println!("  largest sideways deflection along the axle line: {maxdy:.3} mm");
}
