//! `check`: every tool this project builds on, tested against known results before use.
//!
//! - The pitch model (`pitch.rs`) against the numbers Daggfeldt & Petré (2025) state in
//!   their own text, and against itself (two routes, limits, slope).
//! - cf-design's `mass_properties` against exact volumes, because this project is a contest
//!   in grams: it has to resolve ~1 g on a ~180 g chassis with 2-3 mm walls.

use std::f64::consts::{FRAC_PI_2, PI};
use std::time::Instant;

use cortenforge::cf_design::Solid;
use cortenforge::cf_design::mechanism::mass::mass_properties;
use nalgebra::{UnitQuaternion, Vector3};

use crate::materials::AL_7075_T6;
use crate::pitch::{Rocker, fixed_alpha, roller_alpha};

/// Pass/fail tally.
#[derive(Default)]
pub struct Tally {
    pub pass: usize,
    pub fail: usize,
}

impl Tally {
    pub fn item(&mut self, name: &str, ok: bool, detail: impl AsRef<str>) {
        if ok {
            self.pass += 1;
        } else {
            self.fail += 1;
        }
        println!("  [{}] {name}: {}", if ok { " ok " } else { "FAIL" }, detail.as_ref());
    }
}

pub fn run() -> Tally {
    let mut t = Tally::default();
    println!("pitch model (Daggfeldt & Petre 2025)");
    pitch_checks(&mut t);
    println!("cf-design mass_properties vs exact volumes ({})", AL_7075_T6.name);
    mass_checks(&mut t);
    println!("scan measuring (mesh-io, mesh-measure, scan-prep-core) on a tilted ellipsoid");
    scan_checks(&mut t);
    println!("strength tools (round 2): sections, wheel loads, the rolled rocker");
    strength_checks(&mut t);
    println!("sim-soft as a linear-elastic solver (round 2, step 2)");
    fem_checks(&mut t);
    println!("hockey boot geometry: the dorsal line's nearest point, mesh volumes");
    boot_checks(&mut t);
    println!("scan frame on real stand-in scans (Schuster et al. 2024, pre-aligned: x heel to toe, z up)");
    stand_in_checks(&mut t);
    println!("stand-in leg tools (track D1): Catmull-Clark, the tape girth");
    leg_checks(&mut t);
    println!("impacts (track D3): sim-soft-explicit with a free impactor, against a spring-mass impact");
    impact_checks(&mut t);
    println!("posing (track G3): motions, and reading them across to the foot frame");
    pose_checks(&mut t);
    t
}

/// Posing's algebra: a turn about a joint keeps the joint put; a motion after its inverse is no
/// motion; a motion read across from MakeHuman's frame (dm, y up) to the foot frame (mm, z up)
/// moves points as the original does, read across (what carries the pads with the body).
fn pose_checks(t: &mut Tally) {
    use crate::body::{motion_to_mm, to_mm};
    use crate::pose::Motion;
    let axis = nalgebra::Unit::new_normalize(Vector3::new(0.3, -0.5, 0.8));
    let r = nalgebra::Rotation3::from_axis_angle(&axis, 0.7);
    let h = Vector3::new(0.4, 5.2, -0.3);
    let m = Motion { r, t: h - r * h };
    let moved = (m.apply(h) - h).norm();
    t.item("a turn about a joint keeps the joint put", moved < 1e-12, format!("moved {moved:.1e} dm"));
    let pts = [Vector3::new(1.0, 2.0, 3.0), h, Vector3::new(-2.0, 0.5, 7.0)];
    let back = m.after(&m.inverse());
    let err = pts.iter().map(|&x| (back.apply(x) - x).norm()).fold(0.0, f64::max);
    t.item("a motion after its inverse is no motion", err < 1e-12, format!("worst {err:.1e} dm"));
    let (floor, shift) = (-8.3, (12.0, -95.0));
    let mm = motion_to_mm(&m, floor, shift);
    let worst = pts.iter().map(|&v| (mm.apply(to_mm(v, floor, shift).coords) - to_mm(m.apply(v), floor, shift).coords).norm()).fold(0.0, f64::max);
    let det = mm.r.matrix().determinant();
    t.item(
        "a motion read across to the foot frame moves points alike, and stays a turn",
        worst < 1e-9 && (det - 1.0).abs() < 1e-12,
        format!("worst {worst:.1e} mm, determinant {det:.12}"),
    );
    kneecap_checks(t);
    cup_checks(t);
}

/// The kneecap (`pose::Patella`, RESEARCH.md 24) on a made-up knee (dm, MakeHuman's axes: the
/// shin turns about +x through the origin): its three rules hold through the range (it turns
/// 0.7 times the knee, its centre stays on its circle, its tendon keeps its length), and in a rig
/// the skin it takes over moves with it.
fn kneecap_checks(t: &mut Tally) {
    use crate::pose::{Bone, PATELLA_FLEX, Patella, Rig};
    use nalgebra::Rotation3;
    use std::collections::HashMap;
    let rx = |deg: f64| Rotation3::from_axis_angle(&Vector3::x_axis(), deg.to_radians());
    let (knee, centre, low, tub) = (Vector3::zeros(), Vector3::new(0.0, 0.17, 0.42), Vector3::new(0.0, -0.03, 0.42), Vector3::new(0.0, -0.47, 0.36));
    let p = Patella { bone: 2, femur: 0, shin: 1, knee, centre, low, tub };
    let (mut worst, mut shares) = ([0.0f64; 4], Vec::new());
    for deg in [5.0, 20.0, 45.0, 70.0, 90.0, 135.0] {
        let Some((a, k)) = p.on_thigh(&rx(deg)) else {
            t.item("a kneecap: a turn round the knee keeps its tendon", false, format!("none at knee {deg}"));
            return;
        };
        let c = a.apply(centre);
        let errs = [
            (a.r.matrix() - rx(PATELLA_FLEX * deg).matrix()).norm(),
            ((c - knee).norm() - (centre - knee).norm()).abs(),
            ((a.apply(low) - rx(deg) * tub).norm() - (low - tub).norm()).abs(),
            c.x.abs(),
        ];
        for (w, e) in worst.iter_mut().zip(errs) {
            *w = w.max(e);
        }
        shares.push(format!("{deg:.0}: {k:.3}"));
    }
    t.item(
        "a kneecap turns 0.7 times the knee, its centre on its circle, its tendon its length, in the knee's plane",
        worst.iter().all(|&e| e < 1e-9),
        format!("worst: turn {:.1e}, radius {:.1e} dm, tendon {:.1e} dm, out of plane {:.1e} dm; round the knee per degree of it: {}", worst[0], worst[1], worst[2], worst[3], shares.join(", ")),
    );
    // In a rig: thigh, shin, and a skin vertex over the kneecap shared 0.6 / 0.4 between them,
    // all of it handed to the kneecap.
    let raw = vec![Vector3::new(0.0, 1.0, 0.0), knee, Vector3::new(0.0, 0.17, 0.5)];
    let mut rig = Rig {
        bones: vec![
            Bone { name: "upperleg02.R".into(), head: "hip".into(), tail: "knee".into(), parent: None },
            Bone { name: "lowerleg01.R".into(), head: "knee".into(), tail: String::new(), parent: Some(0) },
        ],
        index: HashMap::from([("upperleg02.R".to_string(), 0), ("lowerleg01.R".to_string(), 1)]),
        joints: HashMap::from([("hip".to_string(), vec![0]), ("knee".to_string(), vec![1])]),
        weights: vec![vec![(2, 0.6)], vec![(2, 0.4)]],
        patellae: Vec::new(),
    };
    let ok = rig.add_patella(&raw, "R", centre, low, tub, &[(2, 1.0)]).is_ok();
    let moves = rig.motions(&raw, &HashMap::from([("lowerleg01.R".to_string(), rx(70.0))]));
    let skinned = rig.skin(&raw, &moves)[2];
    let want = p.on_thigh(&rx(70.0)).map_or(Vector3::repeat(f64::NAN), |(a, _)| a.apply(raw[2]));
    let left: f64 = rig.weights[..2].iter().flatten().map(|w| w.1).sum();
    let err = (skinned - want).norm();
    t.item(
        "a kneecap in a rig: the skin it takes over moves with it, the thigh and shin left none of it",
        ok && err < 1e-12 && left.abs() < 1e-12,
        format!("skin {err:.1e} dm from the kneecap's motion; thigh and shin keep {left:.1e}"),
    );
}

/// The cup's tools (G3, round 2): the rigid fit gives back a known motion; the warp reads a
/// cylinder's angle and height at points off its surface as those of the foot of the normal.
fn cup_checks(t: &mut Tally) {
    use crate::cup::kabsch;
    use crate::pad::Warp;
    use cortenforge::mesh::types::IndexedMesh;
    use nalgebra::Point3;
    let axis = nalgebra::Unit::new_normalize(Vector3::new(-0.2, 0.9, 0.4));
    let r = nalgebra::Rotation3::from_axis_angle(&axis, 1.1);
    let tr = Vector3::new(30.0, -12.0, 85.0);
    let a: Vec<Point3<f64>> = (0..40).map(|i| Point3::new((i as f64 * 7.3).sin() * 60.0, (i as f64 * 3.1).cos() * 45.0, i as f64 * 4.0)).collect();
    let b: Vec<Point3<f64>> = a.iter().map(|p| Point3::from(r * p.coords + tr)).collect();
    let m = kabsch(&a, &b);
    let err = a.iter().zip(&b).map(|(p, q)| (m.apply(p.coords) - q.coords).norm()).fold(0.0, f64::max);
    let dang = crate::cup::turn_deg(&(m.r.inverse() * r));
    t.item("the rigid fit (Kabsch) gives back a known turn and shift", err < 1e-9 && dang < 1e-7, format!("worst {err:.1e} mm, turn off by {dang:.1e} deg"));

    // A cylinder r = 50, z 0..200, 5-degree facets every 5 mm; the values (cos, sin, z).
    let (nt, nz) = (72usize, 41usize);
    let mut cyl = IndexedMesh::default();
    let mut vals = Vec::new();
    for j in 0..nz {
        for i in 0..nt {
            let th = (i as f64 * 5.0).to_radians();
            let z = j as f64 * 5.0;
            cyl.vertices.push(Point3::new(50.0 * th.cos(), 50.0 * th.sin(), z));
            vals.push([th.cos(), th.sin(), z]);
        }
    }
    for j in 0..nz - 1 {
        for i in 0..nt {
            let (a0, a1) = ((j * nt + i) as u32, (j * nt + (i + 1) % nt) as u32);
            let (b0, b1) = (a0 + nt as u32, a1 + nt as u32);
            cyl.faces.push([a0, a1, b1]);
            cyl.faces.push([a0, b1, b0]);
        }
    }
    let w = Warp::build(&cyl, &vals, &cyl.faces, Point3::new(-80.0, -80.0, 20.0), Point3::new(80.0, 80.0, 180.0), 1.5, 30.0, crate::cup::WARP_SMOOTH, 4);
    let mut worst = (0.0f64, 0.0f64);
    for (th, z, r) in [(37.0, 113.0, 62.0), (-150.0, 41.3, 55.0), (178.0, 160.0, 70.0), (-2.5, 99.9, 58.0)] {
        let tr = f64::to_radians(th);
        let (got_t, got_z) = w.at(Point3::new(r * tr.cos(), r * tr.sin(), z));
        let dt = ((got_t - th + 540.0).rem_euclid(360.0) - 180.0).abs();
        worst = (worst.0.max(dt), worst.1.max((got_z - z).abs()));
    }
    // The resting fit: a sphere patch moved rigidly comes back exactly; a piece built on a dome
    // (r 80, 30 deg round) set on flat skin rests on its rim (the dome's 10.7 mm stands off)
    // instead of splitting the difference as least squares does (5.4 in, 5.4 off).
    let patch: Vec<Point3<f64>> = (0..15)
        .flat_map(|i| (0..24).map(move |j| (i, j)))
        .map(|(i, j)| {
            let (a, b) = ((2.0 + 2.0 * i as f64).to_radians(), (15.0 * j as f64).to_radians());
            Point3::new(80.0 * a.sin() * b.cos(), 80.0 * a.sin() * b.sin(), 80.0 * a.cos() - 80.0)
        })
        .collect();
    let idx: Vec<usize> = (0..patch.len()).collect();
    let from = IndexedMesh { vertices: patch.clone(), ..Default::default() };
    let moved = IndexedMesh { vertices: patch.iter().map(|p| Point3::from(r * p.coords + tr)).collect(), ..Default::default() };
    let n_moved: Vec<Vector3<f64>> = patch.iter().map(|p| r * (p.coords - Vector3::new(0.0, 0.0, -80.0)).normalize()).collect();
    let m = crate::cup::fit(&idx, &from, &moved, &n_moved, &[]);
    let err = patch.iter().zip(&moved.vertices).map(|(p, q)| (m.apply(p.coords) - q.coords).norm()).fold(0.0, f64::max);
    let flat = IndexedMesh { vertices: patch.iter().map(|p| Point3::new(p.x, p.y, 0.0)).collect(), ..Default::default() };
    let up = vec![Vector3::z(); patch.len()];
    let rest = crate::cup::fit(&idx, &from, &flat, &up, &[]);
    let (into, off, _) = crate::cup::sit(&idx, &from, &flat, &rest, &up);
    let (k_in, k_off, _) = crate::cup::sit(&idx, &from, &flat, &kabsch(&patch, &flat.vertices), &up);
    // By hand: it rises until only the rim ring presses in, by d where the rings balance along
    // z: PUSH_OVER_PULL x d (the rim) = sum over the other rings of (rim depth - depth - d).
    let depth = |deg: f64| 80.0 * (1.0 - f64::to_radians(deg).cos());
    let rim = depth(30.0);
    let others: Vec<f64> = (0..14).map(|i| depth(2.0 + 2.0 * i as f64)).collect();
    let d = others.iter().map(|x| rim - x).sum::<f64>() / (crate::cup::PUSH_OVER_PULL + others.len() as f64);
    let off_hand = rim - depth(2.0) - d;
    t.item(
        "the resting fit gives back a rigid motion, and rests a dome on flat skin by its rim as the balance by hand says",
        err < 1e-6 && (into - d).abs() < 0.01 && (off - off_hand).abs() < 0.01,
        format!("worst {err:.1e} mm; on flat skin {into:.3} mm in, {off:.2} off (by hand {d:.3}, {off_hand:.2}; least squares {k_in:.1} in, {k_off:.1} off)"),
    );
    // Hinges: ties alone give back a motion; a turn about the hinge line reads as that turn,
    // with no stretch.
    let tie_pts = [Point3::new(10.0, -40.0, 5.0), Point3::new(-20.0, 30.0, 12.0), Point3::new(35.0, 15.0, -8.0), Point3::new(0.0, 0.0, 40.0)];
    let ties: Vec<(Point3<f64>, Point3<f64>)> = tie_pts.iter().map(|p| (*p, Point3::from(r * p.coords + tr))).collect();
    let none = IndexedMesh::default();
    let mt = crate::cup::fit(&[], &none, &none, &[], &ties);
    let tie_err = ties.iter().map(|(a, b)| (mt.apply(a.coords) - b.coords).norm()).fold(0.0, f64::max);
    let line: Vec<Point3<f64>> = (0..9).map(|k| Point3::new(20.0, -40.0 + 10.0 * k as f64, 7.0)).collect();
    let turn = nalgebra::Rotation3::from_axis_angle(&Vector3::y_axis(), f64::to_radians(30.0));
    let about = crate::pose::Motion { r: turn, t: Vector3::new(20.0, 0.0, 7.0) - turn * Vector3::new(20.0, 0.0, 7.0) };
    let (h_turn, h_stretch) = crate::cup::hinge(&line, &about, &crate::pose::Motion::identity());
    // Its give, across the hinge (+z) and out of the surface (+x): none for the turn; a 2 mm
    // shift across reads as 2 mm opening, back the other way as closing.
    let dirs = vec![(Vector3::z(), Vector3::x()); line.len()];
    let id = crate::pose::Motion::identity();
    let g0 = crate::cup::give(&line, &dirs, &about, &id);
    let shift = |d: f64| crate::pose::Motion { r: nalgebra::Rotation3::identity(), t: Vector3::new(0.0, 0.0, d) };
    let (g1, g2) = (crate::cup::give(&line, &dirs, &shift(2.0), &id), crate::cup::give(&line, &dirs, &shift(-2.0), &id));
    let give_ok = [g0.open, g0.close, g0.lift, g0.side].iter().all(|x| x.abs() < 1e-9)
        && (g1.open - 2.0).abs() < 1e-12
        && g1.close.abs() < 1e-12
        && (g2.close - 2.0).abs() < 1e-12
        && g1.lift.abs() + g1.side.abs() < 1e-12;
    t.item(
        "a hinge: ties alone give back a motion; a 30 deg turn about the hinge line reads 30 deg, no stretch, no give; a shift across it reads as opening or closing",
        tie_err < 1e-6 && (h_turn - 30.0).abs() < 1e-9 && h_stretch < 1e-9 && give_ok,
        format!("ties worst {tie_err:.1e} mm; hinge {h_turn:.6} deg, stretch {h_stretch:.1e} mm; give for the turn {:.1e} mm, for +2 mm open {:.3} close {:.3}, for -2 mm close {:.3}", g0.open.max(g0.close).max(g0.lift).max(g0.side), g1.open, g1.close, g2.close),
    );
    t.item(
        "the warp reads a cylinder's angle and height 5-20 mm off its surface",
        worst.0 < 0.3 && worst.1 < 0.2,
        format!("worst {:.3} deg, {:.3} mm", worst.0, worst.1),
    );

    // Off a hollow: a V valley (sides at 35 deg, its floor along y), carrying the distance along
    // the skin from the floor. 18 mm above the floor, the closest point jumps from one side to
    // the other at the middle (by hand 2 x 18 sin 35 = 20.7 mm along the skin); the blend runs
    // through it smoothly and never turns back.
    let tn = f64::to_radians(35.0).tan();
    let (nx, ny) = (61usize, 61usize);
    let mut vee = IndexedMesh::default();
    let mut vv = Vec::new();
    for j in 0..ny {
        for i in 0..nx {
            let (x, y) = (-60.0 + 2.0 * i as f64, -60.0 + 2.0 * j as f64);
            vee.vertices.push(Point3::new(x, y, x.abs() * tn));
            vv.push([x / f64::to_radians(35.0).cos(), 0.0, 0.0]);
        }
    }
    for j in 0..ny - 1 {
        for i in 0..nx - 1 {
            let a = (j * nx + i) as u32;
            let (b, c, d) = (a + 1, a + nx as u32 + 1, a + nx as u32);
            vee.faces.push([a, b, c]);
            vee.faces.push([a, c, d]);
        }
    }
    let (lo, hi) = (Point3::new(-30.0, -10.0, 0.0), Point3::new(30.0, 10.0, 40.0));
    let along = |w: &Warp| -> (f64, f64) {
        let us: Vec<f64> = (0..=120).map(|k| w.raw(Point3::new(-15.0 + 0.25 * k as f64, 0.0, 18.0)).map_or(f64::NAN, |a| a[0])).collect();
        let steps: Vec<f64> = us.windows(2).map(|p| p[1] - p[0]).collect();
        (steps.iter().cloned().fold(f64::MIN, f64::max), steps.iter().cloned().fold(f64::MAX, f64::min))
    };
    let (cp_max, _) = along(&Warp::build(&vee, &vv, &vee.faces, lo, hi, 1.5, 30.0, 0.0, 4));
    let (bl_max, bl_min) = along(&Warp::build(&vee, &vv, &vee.faces, lo, hi, 1.5, 30.0, crate::cup::WARP_SMOOTH, 4));
    let jump = 2.0 * 18.0 * f64::to_radians(35.0).sin();
    t.item(
        "the warp's blend runs smoothly off a hollow where the closest point jumps",
        cp_max > 3.0 && bl_max < 1.0 && bl_min >= -1e-9,
        format!("largest step along 0.25 mm: closest point {cp_max:.1} mm (its jump {jump:.1} over a 1.5 mm cell), blended {bl_max:.2} mm, never back ({bl_min:.3})"),
    );
}

/// A heavy flat impactor on a soft block (a spring, k = E A / L): deepest compression
/// v sqrt(m/k), peak force v sqrt(k m) (averaged over the block's own wave period, which rings
/// on it), contact time pi sqrt(m/k), rebound at the impact speed. The bending study (a thin PA
/// 12 strip's first frequency with 2-4 elements through it) is `cargo run --release -- impact`:
/// minutes, not seconds.
fn impact_checks(t: &mut Tally) {
    match crate::impact::spring_mass_check() {
        Ok((got, want)) => {
            let e = |k: usize| got[k] / want[k] - 1.0;
            t.item(
                "spring-mass impact: compression, averaged peak, contact time, rebound, force on the base within 3%",
                e(2).abs() < 0.03 && e(1).abs() < 0.03 && e(3).abs() < 0.03 && e(4).abs() < 0.03 && e(5).abs() < 0.03,
                format!(
                    "deepest {:.3} mm ({:.3}), peak {:.2} N averaged ({:.2}; raw {:.2}), contact {:.2} ms ({:.2}), rebound {:.3} m/s ({:.3}); force on the held base {:.2} N averaged ({:.2})",
                    got[2] * 1e3,
                    want[2] * 1e3,
                    got[1],
                    want[1],
                    got[0],
                    got[3] * 1e3,
                    want[3] * 1e3,
                    got[4],
                    want[4]
                    , got[5], want[5]
                ),
            );
        }
        Err(e) => t.item("spring-mass impact", false, e),
    }
    // The puck model, fitted to Anderson 2008's contact time and restitution for a rim impact on
    // a rigid plate at 24.6 m/s, predicts the peak force, which the fit left out: 14.2 kN warm
    // (22 C), 37 kN cold (-4 C, an average of three pucks).
    for (cold, measured) in [(false, 14.2e3), (true, 37.0e3)] {
        let p = crate::impact::ice_puck(cold);
        let (peak, tc, e) = p.on_wall(24.6);
        t.item(
            &format!("puck model, {}: predicted peak force within 20% of Anderson 2008's", if cold { "-4 C" } else { "22 C" }),
            (peak / measured - 1.0).abs() < 0.20,
            format!(
                "peak {:.1} kN (measured {:.1}, {:+.0}%); fitted: contact {:.2} ms, restitution {:.3}; k {:.2} MN/m, alpha {:.4} s/m",
                peak / 1e3,
                measured / 1e3,
                100.0 * (peak / measured - 1.0),
                tc * 1e3,
                e,
                p.k / 1e6,
                p.alpha
            ),
        );
    }
}

/// The leg's own tools on cases with answers by hand.
/// - Catmull-Clark, one step on the cube [-1, 1]^3: a corner (valence 3) goes to (F + 2R) / 3 =
///   (5/9, 5/9, 5/9); an edge's point is (two ends + two face centres) / 4, (3/4, 3/4, 0) for the
///   edge from (1, 1, -1) to (1, 1, 1). Two steps: 96 quads, still closed (every edge in two).
/// - On a flat regular grid, Catmull-Clark leaves interior vertices where they are.
/// - The tape girth (convex hull perimeter) of a meshed cylinder r 50 is 2 pi r; of two
///   overlapping cylinders r 30 with centres 40 apart (a peanut) it is 2 pi r + 2 x 40 = 268.50,
///   the tape bridging the waist.
fn leg_checks(t: &mut Tally) {
    use crate::leg::{Subdivision, girth_at, open_edges};
    let p = [
        [-1.0, -1.0, -1.0], [1.0, -1.0, -1.0], [1.0, 1.0, -1.0], [-1.0, 1.0, -1.0],
        [-1.0, -1.0, 1.0], [1.0, -1.0, 1.0], [1.0, 1.0, 1.0], [-1.0, 1.0, 1.0],
    ];
    let v: Vec<Vector3<f64>> = p.iter().map(|c| Vector3::new(c[0], c[1], c[2])).collect();
    let quads = vec![[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [2, 3, 7, 6], [1, 2, 6, 5], [0, 4, 7, 3]];
    let s1 = Subdivision::new(&quads, 8);
    let v1 = s1.apply(&v);
    let corner = v1[6];
    // The edge 2-6 is (1, 1, -1) to (1, 1, 1): find its edge point (the new vertex at (x, x, 0)).
    let edge = v1.iter().skip(8 + 6).copied().find(|q| q.z.abs() < 1e-12 && q.x > 0.0 && q.y > 0.0).unwrap_or(Vector3::zeros());
    let s2 = Subdivision::new(&s1.quads, v1.len());
    let v2 = s2.apply(&v1);
    let mut mesh = cortenforge::mesh::types::IndexedMesh::default();
    mesh.vertices = v2.iter().map(|q| nalgebra::Point3::from(*q)).collect();
    for q in &s2.quads {
        mesh.faces.push([q[0], q[1], q[2]]);
        mesh.faces.push([q[0], q[2], q[3]]);
    }
    t.item(
        "Catmull-Clark on a cube: corner 5/9, edge point 3/4, closed after two steps",
        (corner - Vector3::repeat(5.0 / 9.0)).norm() < 1e-12 && (edge - Vector3::new(0.75, 0.75, 0.0)).norm() < 1e-12 && s2.quads.len() == 96 && open_edges(&mesh) == 0,
        format!(
            "corner ({:.6}, {:.6}, {:.6}), edge point ({:.6}, {:.6}, {:.6}), {} quads, {} open edges",
            corner.x, corner.y, corner.z, edge.x, edge.y, edge.z, s2.quads.len(), open_edges(&mesh)
        ),
    );
    // A 4 x 4 flat grid of unit quads: interior vertices stay put, everything stays flat.
    let n = 5;
    let grid: Vec<Vector3<f64>> = (0..n * n).map(|i| Vector3::new((i % n) as f64, (i / n) as f64, 0.0)).collect();
    let gq: Vec<[u32; 4]> = (0..(n - 1) * (n - 1))
        .map(|k| {
            let (i, j) = ((k % (n - 1)) as u32, (k / (n - 1)) as u32);
            let id = |a: u32, b: u32| b * n as u32 + a;
            [id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)]
        })
        .collect();
    let g1 = Subdivision::new(&gq, n * n).apply(&grid);
    let interior_moved = (0..n * n)
        .filter(|i| i % n != 0 && i % n != n - 1 && i / n != 0 && i / n != n - 1)
        .map(|i| (g1[i] - grid[i]).norm())
        .fold(0.0, f64::max);
    let off_plane = g1.iter().map(|q| q.z.abs()).fold(0.0, f64::max);
    t.item(
        "Catmull-Clark on a flat grid: interior vertices stay, all flat",
        interior_moved < 1e-12 && off_plane < 1e-12,
        format!("interior moved {interior_moved:.1e}, off the plane {off_plane:.1e}"),
    );
    // The tape: a cylinder, and a peanut whose waist the tape bridges.
    let cyl = Solid::cylinder(50.0, 60.0).mesh(0.5).geometry;
    let (g_cyl, _) = girth_at(&cyl, 7.3).unwrap_or((0.0, (0.0, 0.0)));
    let peanut = Solid::cylinder(30.0, 60.0).translate(Vector3::new(-20.0, 0.0, 0.0)).union(Solid::cylinder(30.0, 60.0).translate(Vector3::new(20.0, 0.0, 0.0)));
    let pm = peanut.mesh(0.5).geometry;
    let (g_pea, _) = girth_at(&pm, 7.3).unwrap_or((0.0, (0.0, 0.0)));
    let (e_cyl, e_pea) = (g_cyl / (2.0 * PI * 50.0) - 1.0, g_pea / (2.0 * PI * 30.0 + 80.0) - 1.0);
    t.item(
        "tape girth: cylinder 2 pi r, peanut 2 pi r + 2 d (within 0.2%)",
        e_cyl.abs() < 0.002 && e_pea.abs() < 0.002,
        format!("cylinder {g_cyl:.2} mm vs {:.2} ({:+.3}%), peanut {g_pea:.2} vs {:.2} ({:+.3}%)", 2.0 * PI * 50.0, e_cyl * 100.0, 2.0 * PI * 30.0 + 80.0, e_pea * 100.0),
    );
}

/// The boot's own tools, on cases with answers by hand: the dorsal line's nearest point (an L of
/// two 100 mm legs, as the instep turns up the shin), and the volume of a closed mesh (a meshed
/// sphere against 4/3 pi r^3).
fn boot_checks(t: &mut Tally) {
    use crate::hockey_boot::{Profile, mesh_volume};
    use nalgebra::Point3;
    let prof = Profile::new(vec![Point3::new(200.0, 5.0, 0.0), Point3::new(100.0, 5.0, 0.0), Point3::new(100.0, 25.0, 100.0)]);
    // Above the first leg: nearest at x = 150 -> s = 50, 10 mm away; y from the first leg.
    let a = prof.closest(150.0, 10.0);
    t.item(
        "dorsal line: nearest on the first leg",
        (a.s - 50.0).abs() < 1e-9 && (a.d - 10.0).abs() < 1e-9 && (a.y - 5.0).abs() < 1e-9,
        format!("s {:.3}, d {:.3}, y {:.3}", a.s, a.d, a.y),
    );
    // In front of the second leg at z = 50: nearest at its middle -> s = 150, y halfway (15).
    let b = prof.closest(112.0, 50.0);
    t.item(
        "dorsal line: nearest on the second leg",
        (b.s - 150.0).abs() < 1e-9 && (b.d - 12.0).abs() < 1e-9 && (b.y - 15.0).abs() < 1e-9,
        format!("s {:.3}, d {:.3}, y {:.3}", b.s, b.d, b.y),
    );
    // Outside the corner: nearest is the corner itself.
    let c = prof.closest(90.0, -10.0);
    t.item(
        "dorsal line: nearest at the corner",
        (c.s - 100.0).abs() < 1e-9 && (c.d - 200f64.sqrt()).abs() < 1e-9,
        format!("s {:.3}, d {:.3}", c.s, c.d),
    );
    let n = prof.normal(150.0);
    t.item(
        "dorsal line: outward normal on the shin points forward",
        (n.x - 1.0).abs() < 1e-9 && n.z.abs() < 1e-9,
        format!("({:.3}, {:.3})", n.x, n.z),
    );
    let r = 20.0;
    let mesh = crate::round1::mesh_of(&Solid::sphere(r), 0.5);
    let v = mesh_volume(&mesh);
    let exact = 4.0 / 3.0 * PI * r.powi(3);
    let err = v / exact - 1.0;
    t.item("mesh volume: sphere r 20 at 0.5 mm", err.abs() < 0.01, format!("{v:.0} vs {exact:.0} mm3 ({:+.2}%)", err * 100.0));
    // The viewer's decimation on a thin wall coarser than its cells: a spherical shell, r 20,
    // 2 mm wall, meshed at 0.4 mm and clustered at 2.5 mm. Both faces must survive (volume),
    // every vertex must sit on the surface, and no sliver may span the part.
    let shell = Solid::sphere(20.0).subtract(Solid::sphere(18.0));
    let fine = crate::round1::mesh_of(&shell, 0.4);
    let coarse = crate::hockey_boot::cluster_decimate(&fine, 2.5, Some(&|q| shell.evaluate(&q)));
    let exact = 4.0 / 3.0 * PI * (20f64.powi(3) - 18f64.powi(3));
    let err = mesh_volume(&coarse) / exact - 1.0;
    let off = coarse.vertices.iter().map(|v| shell.evaluate(v).abs()).fold(0.0, f64::max);
    let edge = coarse.faces.iter().flat_map(|f| [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])]).map(|(a, b)| (coarse.vertices[a as usize] - coarse.vertices[b as usize]).norm()).fold(0.0, f64::max);
    t.item(
        "viewer decimation: 2 mm shell at 2.5 mm cells keeps both faces, on the surface",
        err.abs() < 0.03 && off < 0.05 && edge < 8.0,
        format!("{} -> {} triangles, volume {:+.2}%, worst vertex {off:.3} mm off, longest edge {edge:.1} mm", fine.faces.len(), coarse.faces.len(), err * 100.0),
    );
    // The plane-section girth (for girths of the foot and boot): a sphere r 20 cut through its
    // centre, level and tilted 30 deg,
    // is a great circle (2 pi r = 125.66 mm); off-centre by 12 mm, a circle of radius 16.
    let ball = Solid::sphere(20.0);
    let fb = |q: nalgebra::Point3<f64>| ball.evaluate(&q);
    let level = crate::hockey_boot::plane_girth(&fb, Point3::new(-10.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0));
    let tilted = crate::hockey_boot::plane_girth(&fb, Point3::new(-10.0, 0.0, -5.77), Point3::new(10.0, 0.0, 5.77));
    let off = crate::hockey_boot::plane_girth(&fb, Point3::new(-10.0, 0.0, 12.0), Point3::new(10.0, 0.0, 12.0));
    let (gc, sc) = (2.0 * PI * 20.0, 2.0 * PI * 16.0);
    t.item(
        "plane-section girth: a sphere, through the centre and off it",
        (level / gc - 1.0).abs() < 1e-3 && (tilted / gc - 1.0).abs() < 1e-3 && (off / sc - 1.0).abs() < 1e-3,
        format!("{level:.2} / {tilted:.2} vs {gc:.2}; {off:.2} vs {sc:.2} mm"),
    );
    // The viewer's mesh file: base64 against RFC 4648's test vectors.
    let vectors = [("", ""), ("f", "Zg=="), ("fo", "Zm8="), ("foo", "Zm9v"), ("foob", "Zm9vYg=="), ("fooba", "Zm9vYmE="), ("foobar", "Zm9vYmFy")];
    let bad: Vec<&str> = vectors.iter().filter(|(i, o)| crate::hockey_boot::base64(i.as_bytes()) != *o).map(|(i, _)| *i).collect();
    t.item("base64: RFC 4648 test vectors", bad.is_empty(), format!("{} of {} match", vectors.len() - bad.len(), vectors.len()));
}

/// The three stand-in scans that broke earlier versions of `scan::measure`: a loaded foot with a
/// flat sole, a light one whose bands flipped at a 7 deg tilt, and one whose toes lift off the
/// floor. Skipped (not failed) when the dataset isn't on this machine (it isn't in git).
fn stand_in_checks(t: &mut Tally) {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("scans").join("stand-in");
    for name in ["001_fBW_L.ply", "002_mBW_R.ply", "051_mBW_R.ply"] {
        let path = dir.join(name);
        if !path.exists() {
            println!("  [skip] {name}: not here (fetch it, see docs/RESEARCH.md 12)");
            continue;
        }
        let (mesh, _, _) = crate::scan::load(&path).expect("load scan");
        let m = crate::scan::measure(&mesh, 20);
        let (xmin, xmax) = mesh.vertices.iter().fold((f64::MAX, f64::MIN), |a, v| (a.0.min(v.x), a.1.max(v.x)));
        let ax = m.along.dot(&Vector3::x());
        let uz = m.up.dot(&Vector3::z());
        let e = (m.length - (xmax - xmin)) / (xmax - xmin);
        t.item(
            &format!("{name}: up within 3 deg, heel right, length within 1.5%"),
            uz > 3f64.to_radians().cos() && ax > 0.98 && e.abs() < 0.015,
            format!(
                "up.z {uz:.4}, heel-to-toe.x {ax:+.4}, length {:.1} vs {:.1} mm ({:+.2}%), up from {}",
                m.length,
                xmax - xmin,
                e * 100.0,
                m.up_from
            ),
        );
    }
}

/// Ramanujan's second approximation to an ellipse's perimeter (error ~1e-10 relative here).
fn ellipse_perimeter(a: f64, b: f64) -> f64 {
    let h = ((a - b) / (a + b)).powi(2);
    PI * (a + b) * (1.0 + 3.0 * h / (10.0 + (4.0 - 3.0 * h).sqrt()))
}

/// A foot-sized ellipsoid (260 x 100 x 80 mm), meshed by cf-design, tilted and moved, written to
/// PLY and read back the way a scan is, then measured: extents and girths have exact answers.
fn scan_checks(t: &mut Tally) {
    let (a, b, c) = (130.0, 50.0, 40.0);
    let t0 = Instant::now();
    let mut mesh = Solid::ellipsoid(Vector3::new(a, b, c)).mesh(1.0).geometry;
    let tilt = UnitQuaternion::from_euler_angles(0.3, -0.5, 1.1);
    let shift = Vector3::new(40.0, -25.0, 310.0);
    for v in &mut mesh.vertices {
        *v = tilt * *v + shift;
    }
    let path = std::env::temp_dir().join("custom-skate-check-ellipsoid.ply");
    cortenforge::mesh::io::save_mesh(&mesh, &path).expect("save ply");
    let raw = cortenforge::mesh::io::load_mesh(&path).expect("load ply");
    let (back, scale, merged) = crate::scan::load(&path).expect("load ply");
    let _ = std::fs::remove_file(&path);
    let worst_move = raw
        .vertices
        .iter()
        .zip(&mesh.vertices)
        .map(|(a, b)| (a - b).norm())
        .fold(0.0_f64, f64::max);
    t.item(
        "PLY round trip: same triangles, vertices within 1e-4 mm",
        raw.faces.len() == mesh.faces.len() && raw.vertices.len() == mesh.vertices.len() && worst_move < 1e-4,
        format!(
            "{} triangles, worst vertex move {worst_move:.1e} mm (PLY stores f32); then units x{scale}, \
             welding merged {merged} vertices, {} triangles left",
            raw.faces.len(),
            back.faces.len()
        ),
    );

    let m = crate::scan::measure(&back, 20);
    let err = |x: f64, exact: f64| (x - exact) / exact;
    let (el, ew, eh) = (err(m.length, 2.0 * a), err(m.width, 2.0 * b), err(m.height, 2.0 * c));
    t.item(
        "principal-axis extents within 0.5%",
        el.abs() < 0.005 && ew.abs() < 0.005 && eh.abs() < 0.005,
        format!(
            "{:.2} / {:.2} / {:.2} mm vs 260 / 100 / 80 ({:+.3}% / {:+.3}% / {:+.3}%)",
            m.length,
            m.width,
            m.height,
            el * 100.0,
            ew * 100.0,
            eh * 100.0
        ),
    );
    // Stations at 1/4 and 1/2 of the length: ellipses with semi-axes scaled by sqrt(1 - (x/a)^2).
    let mut worst: f64 = 0.0;
    let mut line = String::new();
    for frac in [0.25, 0.5] {
        let s = m.stations.iter().find(|s| (s.frac - frac).abs() < 1e-9).expect("station");
        let x = (frac - 0.5) * 2.0 * a;
        let k = (1.0 - (x / a).powi(2)).sqrt();
        let exact = ellipse_perimeter(b * k, c * k);
        let e = err(s.perimeter, exact);
        worst = worst.max(if s.contours == 1 { e.abs() } else { f64::INFINITY });
        line += &format!(
            "{frac}: {:.2} vs {:.2} mm ({:+.3}%), {} loop(s), area {:.0} vs {:.0} mm2; ",
            s.perimeter,
            exact,
            e * 100.0,
            s.contours,
            s.area,
            PI * b * c * k * k
        );
    }
    t.item("girths: one loop each, perimeter within 0.5%", worst < 0.005, line);
    t.item(
        "closed mesh has no open edges",
        m.open_loops.is_empty(),
        format!("{} open loops, {:.2} s", m.open_loops.len(), t0.elapsed().as_secs_f64()),
    );

    // A foot with a leg stump: the ellipsoid plus a vertical cylinder over its -x end, cut flat
    // at z = 100 (scan-prep-core's plane clip), then tilted. The cut is the only open edge; it
    // must be found, and must point the heel to -x and up to +z.
    let stump = Solid::cylinder(35.0, 70.0).translate(Vector3::new(-80.0, 0.0, 60.0));
    let solid = Solid::ellipsoid(Vector3::new(a, b, c)).union(stump);
    let mut leg = solid.mesh(1.0).geometry;
    cortenforge::mesh::repair::weld_vertices(&mut leg, 1e-5);
    let mut cut = cortenforge::cf_scan_prep_core::clip_mesh_against_plane(
        &leg,
        nalgebra::Point3::new(0.0, 0.0, 100.0),
        -Vector3::z(),
    );
    let before = crate::scan::measure(&cut, 4).open_loops.len();
    let merged = cortenforge::mesh::repair::weld_vertices(&mut cut, 1e-5);
    for v in &mut cut.vertices {
        *v = tilt * *v + shift;
    }
    let m = crate::scan::measure(&cut, 20);
    let heel_to_toe = tilt * Vector3::x();
    let up = tilt * Vector3::z();
    let (d_along, d_up) = (m.along.dot(&heel_to_toe), m.up.dot(&up));
    let ring = m.open_loops.iter().map(|l| l.length).fold(0.0_f64, f64::max);
    t.item(
        "leg stump: one open edge, heel and up found",
        m.open_loops.len() == 1 && d_along > 0.999 && d_up > 0.999 && (ring - 2.0 * PI * 35.0).abs() < 1.0,
        format!(
            "{} open edge(s) after welding the clip ({before} before, {merged} vertices merged), the cut \
             {ring:.2} mm round (circle 219.91); heel-to-toe . true {d_along:.3}, up . true {d_up:.3}",
            m.open_loops.len()
        ),
    );
}

fn pitch_checks(t: &mut Tally) {
    // The paper: "moving the contact point from the back to the front of the blade, a
    // distance of not more than 30 cm, on a blade with a radius of curvature of 2.7 m
    // results in less than 0.12 rad (~7 deg) of skate rotation".
    let a = fixed_alpha(0.30, 2.7);
    t.item(
        "eq.1: 30 cm on a 2.7 m blade < 0.12 rad",
        a < 0.12,
        format!("{a:.4} rad = {:.2} deg", a.to_degrees()),
    );

    // The paper: "For a beta of 0.12 rad, the linear approximation in equation (9) would
    // result in an error in phi of less than one tenth of a degree (s = 2.7 m, h = 0.03 m,
    // r = 1 m)". beta = y / s (eq. 5).
    let p = Rocker { r: 1.0, s: 2.7, h: 0.03, theta: 0.0 };
    let y = 0.12 * p.s;
    let err = (p.phi_of_y(y) - p.phi_of_y_linear(y)).abs().to_degrees();
    t.item("eq.9 vs eq.7 at beta 0.12 rad: error < 0.1 deg", err < 0.1, format!("{err:.4} deg"));

    // Eq. 4 + eq. 5 (in alpha) and eq. 7 (in y) are the same curve.
    let mut worst: f64 = 0.0;
    for i in -300..=300 {
        let alpha = f64::from(i) * 1e-3;
        let y = p.y_of_alpha(alpha);
        worst = worst.max((p.phi_of_alpha(alpha) - p.phi_of_y(y)).abs());
    }
    t.item("eq.4+5 and eq.7 agree (alpha +-0.3 rad)", worst < 1e-12, format!("max diff {worst:.1e} rad"));

    // Eq. 11 is the slope of eq. 7 at y = 0 over the slope 1/s of a flat rocker.
    let dy = 1e-6;
    let slope = (p.phi_of_y(dy) - p.phi_of_y(-dy)) / (2.0 * dy);
    let a_num = slope * p.s;
    let a_eq = p.amplification();
    t.item(
        "eq.11 amplification = slope of eq.7 at y = 0",
        (a_num - a_eq).abs() < 1e-6,
        format!("{a_num:.6} vs {a_eq:.6} (r 1 m, s 2.7 m, h 3 cm)"),
    );

    // Limits: a flat rocker is a fixed holder (r_eff -> s); a flat blade is a roller chassis
    // (r_eff -> r, the foot pitches x / r).
    let fixed = Rocker { r: 1e9, ..p }.effective_radius();
    let roller = Rocker { s: 1e9, ..p }.effective_radius();
    t.item(
        "eq.10 limits: r -> inf gives s, s -> inf gives r",
        (fixed - p.s).abs() < 1e-6 && (roller - p.r).abs() < 1e-6,
        format!("{fixed:.6} m, {roller:.6} m"),
    );
    let x = 0.05;
    let via_limit = Rocker { s: 1e9, ..p }.phi_of_y_linear(x);
    t.item(
        "roller: pitch x / r is eq.9 with s -> inf",
        (via_limit - roller_alpha(x, p.r)).abs() < 1e-9,
        format!("{:.6} vs {:.6} rad at x 5 cm", via_limit, roller_alpha(x, p.r)),
    );
}

/// Relative error of cf-design's grid mass against an exact volume, at three grid sizes.
/// Returns the error at the finest grid.
fn mass_case(t: &mut Tally, name: &str, solid: &Solid, exact_mm3: f64) -> f64 {
    let rho = AL_7075_T6.density;
    let exact_g = exact_mm3 * 1e-9 * rho * 1000.0;
    let mut line = format!("exact {exact_g:.3} g;");
    let mut last = f64::NAN;
    for cell in [1.0, 0.5, 0.25] {
        let t0 = Instant::now();
        let m = mass_properties(solid, rho, cell).expect("solid has mass");
        let g = m.mass * 1000.0;
        last = (g - exact_g) / exact_g;
        line += &format!(
            " cell {cell} mm: {g:.3} g ({:+.3}%, {:.2} s);",
            last * 100.0,
            t0.elapsed().as_secs_f64()
        );
    }
    t.item(&format!("{name}: within 0.5% at 0.25 mm"), last.abs() < 0.005, line);
    last
}

fn mass_checks(t: &mut Tally) {
    // A rail-like plate: 100 x 3 x 30 mm.
    let plate = Solid::cuboid(Vector3::new(50.0, 1.5, 15.0));
    mass_case(t, "plate 100x3x30", &plate, 100.0 * 3.0 * 30.0);

    // A 2 mm wall with a 20 mm lightening hole through it.
    let wall = Solid::cuboid(Vector3::new(50.0, 1.0, 15.0));
    let x_to_y = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), FRAC_PI_2);
    let hole = Solid::cylinder(10.0, 5.0).rotate(x_to_y);
    let holed = wall.subtract(hole);
    mass_case(t, "2 mm wall, 20 mm hole", &holed, 100.0 * 2.0 * 30.0 - PI * 100.0 * 2.0);

    // An axle-like rod: 8 mm diameter, 60 mm long, off-axis to test the centre of mass too.
    let offset = Vector3::new(12.0, -7.0, 30.0);
    let rod = Solid::cylinder(4.0, 30.0).translate(offset);
    mass_case(t, "rod 8 x 60 mm", &rod, PI * 16.0 * 60.0);
    let m = mass_properties(&rod, AL_7075_T6.density, 0.25).expect("rod has mass");
    let com_err = (m.center_of_mass.coords - offset).norm();
    t.item("rod centre of mass at its offset", com_err < 0.01, format!("off by {com_err:.4} mm"));
}

/// Round 2's strength tools: the section sampler against closed-form sections, the wheel-load
/// model against statics, and the rolled rocker against where it must touch.
fn strength_checks(t: &mut Tally) {
    use crate::strength::{lower_moment, section, wheel_reactions};
    let x_to_y = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), FRAC_PI_2);
    let along_x = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), FRAC_PI_2);
    let close = |a: f64, b: f64, tol: f64| ((a - b) / b).abs() < tol;

    // A 30 (y) x 6 (z) bar along x: A = 180, I = b h^3 / 12 = 540, c = 3.
    let bar = Solid::cuboid(Vector3::new(50.0, 15.0, 3.0));
    let s = section(&bar, 0.0, 0.1, None).expect("bar section");
    t.item(
        "section of a 30 x 6 bar: A 180, I 540 mm4, c 3 (within 1%)",
        close(s.area, 180.0, 0.01) && close(s.i, 540.0, 0.01) && close(s.c(), 3.0, 0.02),
        format!("A {:.2}, I {:.1}, c {:.3}", s.area, s.i, s.c()),
    );
    // An I-section 30 wide, 20 deep, 3 mm flanges and web: I = 30*20^3/12 - 27*14^3/12.
    let i_exact = 30.0 * 8000.0 / 12.0 - 27.0 * 2744.0 / 12.0;
    let ibeam = Solid::cuboid(Vector3::new(50.0, 15.0, 1.5))
        .translate(Vector3::new(0.0, 0.0, 8.5))
        .union(Solid::cuboid(Vector3::new(50.0, 15.0, 1.5)).translate(Vector3::new(0.0, 0.0, -8.5)))
        .union(Solid::cuboid(Vector3::new(50.0, 1.5, 7.5)))
        .translate(Vector3::new(0.0, 4.0, 11.0));
    let s = section(&ibeam, 0.0, 0.1, None).expect("I section");
    t.item(
        &format!("section of an I-beam (offset): I {i_exact:.0} mm4, centroid at z 11 (within 1%)"),
        close(s.i, i_exact, 0.01) && (s.zc - 11.0).abs() < 0.05,
        format!("A {:.1} (exact 222), I {:.0}, zc {:.3}", s.area, s.i, s.zc),
    );
    // A tube along x, 20 / 14 mm: I = pi (D^4 - d^4) / 64.
    let i_tube = PI * (20f64.powi(4) - 14f64.powi(4)) / 64.0;
    let tube = Solid::cylinder(10.0, 40.0).subtract(Solid::cylinder(7.0, 41.0)).rotate(along_x);
    let s = section(&tube, 0.0, 0.1, None).expect("tube section");
    t.item(
        &format!("section of a 20/14 tube: I {i_tube:.0} mm4 (within 1%)"),
        close(s.i, i_tube, 0.01),
        format!("A {:.1} (exact {:.1}), I {:.0}", s.area, PI * (100.0 - 49.0), s.i),
    );
    let _ = x_to_y;

    // Four wheels 76 mm apart, load at the centre: R = P/4 each, M(centre) = P (38 + 114) / 4 = 38 P.
    let ax = [-114.0, -38.0, 38.0, 114.0];
    let r = wheel_reactions(&ax, 0.0, 1000.0);
    let m0 = lower_moment(0.0, &ax, 0.0, 1000.0);
    // Off centre: statics must close (sum R = P, sum R x = P e) and the moment past the last wheel is 0.
    let e = 40.0;
    let r2 = wheel_reactions(&ax, e, 1000.0);
    let fsum: f64 = r2.iter().sum();
    let msum: f64 = r2.iter().zip(&ax).map(|(ri, a)| ri * a).sum();
    let m_end = lower_moment(200.0, &ax, e, 1000.0);
    t.item(
        "rigid frame on 4 wheels: P/4 each and 38 P mm at the centre; statics close off centre",
        r.iter().all(|ri| (ri - 250.0).abs() < 1e-9)
            && (m0 - 38_000.0).abs() < 1e-6
            && (fsum - 1000.0).abs() < 1e-9
            && (msum - 1000.0 * e).abs() < 1e-6
            && m_end.abs() < 1e-6,
        format!("R {r:.1?}, M(0) {m0:.1} N mm; off centre sum R {fsum:.3}, sum R x {msum:.3}, M past the end {m_end:.2e}"),
    );

    // The rolled rocker: at e = +50 mm the wall's arc must sit 0.1 mm above the ledge at
    // x = xc + 50 and lift clear of it 30 mm either side by (30^2 / 2r).
    let p = crate::frame::FrameParams::default();
    let f = crate::frame::build(0.0, 252.4, p);
    let l = f.layout;
    let up = crate::frame::rolled(&f.upper, &l, p.rocker_r, 50.0);
    let y = 0.5 * (l.y_wi + l.y_wo);
    let bottom_at = |x: f64| {
        // Walk up from the ledge until inside the wall.
        let mut z = l.z_t - 0.5;
        while up.evaluate(&nalgebra::Point3::new(x, y, z)) > 0.0 && z < l.z_t + 10.0 {
            z += 0.005;
        }
        z - l.z_t
    };
    let (b0, bm, bp) = (bottom_at(l.xc + 50.0), bottom_at(l.xc + 20.0), bottom_at(l.xc + 80.0));
    let rise = 900.0 / (2.0 * p.rocker_r);
    t.item(
        "rolled rocker touches its ledge at the new contact (0.1 mm gap) and rises x^2/2r beside it",
        (b0 - 0.1).abs() < 0.02 && (bm - 0.1 - rise).abs() < 0.03 && (bp - 0.1 - rise).abs() < 0.03,
        format!("gap at +50 {b0:.3} mm; at +20 {bm:.3} and +80 {bp:.3} (expected {:.3})", 0.1 + rise),
    );
}

/// sim-soft as a linear-elastic solver for aluminium (round 2, step 2): a 100 x 10 x 10 mm
/// 7075 cantilever meshed from a cf-design solid (SDF -> Tet10), tip load 20 N, static (density
/// 0). Against beam theory: tip deflection vs P L^3 / 3EI + P L / (k G A), and the mid-span
/// surface bending stress (nodal-averaged, `fem::nodal_stresses`) vs M c / I.
fn fem_checks(t: &mut Tally) {
    use crate::fem::{self, pick, spread};
    use crate::materials::AL_7075_T6_STRENGTH as AL;
    let (l, b, h, p) = (100.0, 10.0, 10.0, 20.0);
    let bar = Solid::cuboid(Vector3::new(0.5 * l, 0.5 * b, 0.5 * h)).translate(Vector3::new(0.5 * l, 0.0, 0.0));
    let i = b * h * h * h / 12.0;
    let g = AL.e / (2.0 * (1.0 + AL.nu));
    let d_exact = p * l.powi(3) / (3.0 * AL.e * i) + p * l / (5.0 / 6.0 * g * b * h);
    let s_exact = p * 0.5 * l * 0.5 * h / i;
    let t0 = Instant::now();
    let m = match fem::mesh_solid(&bar, 2.5, AL.e, AL.nu) {
        Ok(m) => m,
        Err(e) => {
            t.item("sim-soft cantilever", false, e);
            return;
        }
    };
    let clamp = pick(&m, |q| q.x < 1e-3);
    let tip = pick(&m, |q| q.x > l - 1e-3);
    let (x, r) = match fem::solve_with_reactions(&m, clamp.clone(), vec![], &spread(&tip, Vector3::new(0.0, 0.0, -p))) {
        Ok(x) => x,
        Err(e) => {
            t.item("sim-soft cantilever", false, e);
            return;
        }
    };
    let d = tip.iter().map(|&v| -fem::disp(&m, &x, v).z).sum::<f64>() / tip.len() as f64;
    let ns = fem::nodal_stresses(&m, &x);
    let mid: Vec<f64> = m
        .rest
        .iter()
        .zip(&ns)
        .filter(|(q, s)| s.is_some() && (q.x - 0.5 * l).abs() < 1.5 && q.z.abs() > 0.5 * h - 1e-3)
        .map(|(_, s)| s.expect("filtered")[(0, 0)].abs())
        .collect();
    let s_mid = mid.iter().sum::<f64>() / mid.len().max(1) as f64;
    t.item(
        "sim-soft Tet10 cantilever in 7075: tip deflection within 3% of beam theory",
        ((d - d_exact) / d_exact).abs() < 0.03,
        format!("{d:.5} mm vs {d_exact:.5} ({:+.2}%), {:.1} s", (d / d_exact - 1.0) * 100.0, t0.elapsed().as_secs_f64()),
    );
    t.item(
        "... and mid-span surface bending stress within 2% of M c / I",
        ((s_mid - s_exact) / s_exact).abs() < 0.02 && !mid.is_empty(),
        format!("{s_mid:.3} MPa vs {s_exact:.3} ({:+.2}%), mean of {} surface nodes", (s_mid / s_exact - 1.0) * 100.0, mid.len()),
    );
    // The reaction readout (what the rocker's two-model coupling rests on): the force the bar puts
    // on its clamp is the tip load, and its moment about the clamp is the tip load x the length.
    let (mut f, mut my) = (Vector3::zeros(), 0.0);
    for &v in &clamp {
        let k = 3 * v as usize;
        let rv = Vector3::new(r[k], r[k + 1], r[k + 2]);
        f += rv;
        my += m.rest[v as usize].z * rv.x - m.rest[v as usize].x * rv.z;
    }
    t.item(
        "sim-soft reaction readout: clamp force = tip load within 0.1%, clamp moment = P L within 1%",
        (f.z + p).abs() < 1e-3 * p && f.x.abs() < 1e-3 * p && ((my.abs() - p * l) / (p * l)).abs() < 0.01,
        format!("force ({:.3}, {:.3}, {:.3}) N vs (0, 0, {:.1}); moment {:.1} N mm vs {:.1}", f.x, f.y, f.z, -p, my.abs(), p * l),
    );
}
