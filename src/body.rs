//! Track G1: the stand-in body, waist to feet, for the girdle and everything worn with it
//! (docs/PLAN.md, track D: the gear).
//!
//! MakeHuman's male body (CC0; RESEARCH.md 18), smoothed by two levels of Catmull-Clark like the
//! stand-in leg (`leg.rs`), and set by seven of MakeHuman's own measure sliders, fitted together
//! (Newton, Jacobian by central differences) to ANSUR II's medians for men 1700-1800 mm tall, the
//! stand-in foot 022's height:
//! - the three the leg uses: the knee joint at the tibial height (466), the calf (391), the
//!   girth 115 mm up equal to the foot scan's own;
//! - thigh length to the crotch height (842), thigh girth (625, just below the crotch), hip girth
//!   (the buttocks' largest, 1016), and the waist girth at the waist's height (938 at 1052).
//!
//! **Posed standing**: MakeHuman stands with its feet ~44 cm apart, so each shin leans in. The
//! stand-in leg stood its shin upright by turning it about the ankle, which is fine for a leg
//! alone but would tip the pelvis. Here each leg turns about its own hip joint by the same
//! rotation (knee joint straight above the ankle joint), blended in over the hip so the groin
//! stays smooth: the leg's shape is the same, the pelvis stays level.
//!
//! Placed in the foot's standard frame (foot.rs: x forward, y left, z up, floor 0, mm) as the leg
//! is: the right leg's section 115 mm up centred on the scan's.

use std::collections::HashMap;
use std::path::Path;
use std::time::Instant;

use cortenforge::mesh::io::save_mesh;
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Rotation3, Unit, Vector3};

use crate::leg::{Subdivision, load_base, load_target, makehuman_dir, tape_perimeter};
use crate::render::{Layer, Style, View, render_styled};
use crate::scan;

/// ANSUR II, men 1700-1800 mm tall (2,222 of 4,082), medians in mm (RESEARCH.md 18, 21).
pub mod ansur {
    pub const TIBIAL_HEIGHT: f64 = 466.0;
    pub const CALF_CIRC: f64 = 391.0;
    pub const CROTCH_HEIGHT: f64 = 842.0;
    pub const THIGH_CIRC: f64 = 625.0;
    pub const LOWER_THIGH_CIRC: f64 = 408.0;
    pub const BUTTOCK_CIRC: f64 = 1016.0;
    pub const BUTTOCK_HEIGHT: f64 = 882.0;
    pub const WAIST_CIRC: f64 = 938.0;
    pub const WAIST_HEIGHT: f64 = 1052.0;
    pub const TROCHANTER_HEIGHT: f64 = 896.0;
    pub const ILIAC_CREST_HEIGHT: f64 = 1058.0;
    pub const HIP_BREADTH: f64 = 344.0;
}

/// The sliders: MakeHuman's measure targets (`-decr` below 0, `-incr` above), and its body
/// weight (min to max weight for a young male of average muscle): its average man is slimmer
/// than ANSUR's median soldier, and the waist slider alone ran out at +1 still 58 mm short.
const TARGETS: [&str; 8] = [
    "measure-lowerleg-height",
    "measure-calf-circ",
    "measure-ankle-circ",
    "measure-upperleg-height",
    "measure-thigh-circ",
    "measure-hips-circ",
    "measure-waist-circ",
    "weight",
];
const N: usize = TARGETS.len();

fn target_files(name: &str) -> (String, String) {
    if name == "weight" {
        let w = |s: &str| format!("macrodetails/universal-male-young-averagemuscle-{s}.target");
        (w("minweight"), w("maxweight"))
    } else {
        (format!("measure/{name}-decr.target"), format!("measure/{name}-incr.target"))
    }
}

/// MakeHuman's male body, subdivided, with the sliders ready to add.
struct Model {
    base: Vec<Vector3<f64>>,
    targets: Vec<(Vec<Vector3<f64>>, Vec<Vector3<f64>>)>,
    raw: Vec<Vector3<f64>>,
    raw_targets: Vec<(Vec<Vector3<f64>>, Vec<Vector3<f64>>)>,
    joints: HashMap<String, Vec<u32>>,
    quads: Vec<[u32; 4]>,
}

impl Model {
    fn load(dir: &Path) -> Result<Self, String> {
        let base = load_base(&dir.join("base.obj"))?;
        let n = base.verts.len();
        let mut raw = base.verts.clone();
        for e in ["caucasian", "african", "asian"] {
            let t = load_target(&dir.join("targets/macrodetails").join(format!("{e}-male-young.target")), n)?;
            for (r, d) in raw.iter_mut().zip(&t) {
                *r += d / 3.0;
            }
        }
        let s1 = Subdivision::new(&base.quads, base.n_body);
        let s2 = Subdivision::new(&s1.quads, s1.stencils.len());
        let sub = |x: &[Vector3<f64>]| s2.apply(&s1.apply(&x[..base.n_body]));
        let mut raw_targets = Vec::new();
        for name in TARGETS {
            let (d, i) = target_files(name);
            raw_targets.push((load_target(&dir.join("targets").join(d), n)?, load_target(&dir.join("targets").join(i), n)?));
        }
        let targets = raw_targets.iter().map(|(d, i)| (sub(d), sub(i))).collect();
        Ok(Model { base: sub(&raw), targets, raw, raw_targets, joints: base.joints, quads: s2.quads })
    }

    fn add(x: &mut [Vector3<f64>], t: &(Vec<Vector3<f64>>, Vec<Vector3<f64>>), w: f64) {
        let (d, s) = if w < 0.0 { (&t.0, -w) } else { (&t.1, w) };
        for (a, b) in x.iter_mut().zip(d) {
            *a += b * s;
        }
    }

    fn at(&self, w: &[f64; N]) -> (Vec<Vector3<f64>>, Vec<Vector3<f64>>) {
        let (mut skin, mut raw) = (self.base.clone(), self.raw.clone());
        for k in 0..N {
            Self::add(&mut skin, &self.targets[k], w[k]);
            Self::add(&mut raw, &self.raw_targets[k], w[k]);
        }
        (skin, raw)
    }

    fn joint(&self, raw: &[Vector3<f64>], name: &str) -> Vector3<f64> {
        let idx = &self.joints[name];
        idx.iter().fold(Vector3::zeros(), |a, &i| a + raw[i as usize]) / idx.len() as f64
    }
}

fn smoothstep(e0: f64, e1: f64, x: f64) -> f64 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Stand the legs: each turns about its hip joint until its knee joint is above its ankle
/// joint, fully below the hip and blended to nothing 30 mm above it and across the midline.
/// MakeHuman units (dm, y up); applied to skin and joint markers alike.
fn pose(m: &Model, skin: &mut [Vector3<f64>], raw: &mut [Vector3<f64>]) -> [(f64, f64); 2] {
    let mut tilts = [(0.0, 0.0); 2];
    for (k, side) in ["r", "l"].iter().enumerate() {
        let sign = if *side == "r" { -1.0 } else { 1.0 };
        let hip = m.joint(raw, &format!("joint-{side}-upper-leg"));
        let knee = m.joint(raw, &format!("joint-{side}-knee"));
        let ankle = m.joint(raw, &format!("joint-{side}-ankle"));
        let d = knee - ankle;
        let Some(rot) = Rotation3::rotation_between(&d, &Vector3::y()) else { continue };
        tilts[k] = (d.x.atan2(d.y).to_degrees(), d.z.atan2(d.y).to_degrees());
        let (axis, angle) = rot.axis_angle().map_or((Vector3::x_axis(), 0.0), |(a, t)| (a, t));
        let weight = |v: &Vector3<f64>| smoothstep(hip.y + 0.3, hip.y - 0.7, v.y) * smoothstep(-0.15, 0.15, sign * v.x);
        let turn = |v: &mut Vector3<f64>, axis: &Unit<Vector3<f64>>| {
            let w = weight(v);
            if w > 0.0 {
                *v = hip + Rotation3::from_axis_angle(axis, w * angle) * (*v - hip);
            }
        };
        for v in skin.iter_mut() {
            turn(v, &axis);
        }
        for v in raw.iter_mut() {
            turn(v, &axis);
        }
    }
    tilts
}

/// MakeHuman (dm) to the foot frame (mm), floor at 0, then `shift` in x and y.
fn to_mm(v: Vector3<f64>, floor: f64, shift: (f64, f64)) -> Point3<f64> {
    Point3::new(100.0 * v.z + shift.0, 100.0 * v.x + shift.1, 100.0 * (v.y - floor))
}

/// The body below `z_top` mm (its connected piece holding the pelvis: hands hang free of it),
/// as triangles in the foot frame.
fn lower_body(m: &Model, skin: &[Vector3<f64>], floor: f64, shift: (f64, f64), z_top: f64) -> IndexedMesh {
    let h = |i: u32| 100.0 * (skin[i as usize].y - floor);
    let cand: Vec<[u32; 4]> = m.quads.iter().copied().filter(|q| q.iter().all(|&i| h(i) < z_top)).collect();
    let mut parent: Vec<u32> = (0..skin.len() as u32).collect();
    fn find(p: &mut [u32], mut i: u32) -> u32 {
        while p[i as usize] != i {
            p[i as usize] = p[p[i as usize] as usize];
            i = p[i as usize];
        }
        i
    }
    for q in &cand {
        for k in 1..4 {
            let (a, b) = (find(&mut parent, q[0]), find(&mut parent, q[k]));
            if a != b {
                parent[a as usize] = b;
            }
        }
    }
    // The seed: the skin point nearest the pelvis's middle (the midline, at crotch height + ~0.5 dm).
    let pelvis = Vector3::new(0.0, floor + 0.01 * (ansur::CROTCH_HEIGHT + 60.0), 0.0);
    let seed = cand
        .iter()
        .flatten()
        .copied()
        .min_by(|&a, &b| (skin[a as usize] - pelvis).norm().total_cmp(&(skin[b as usize] - pelvis).norm()))
        .unwrap_or(0);
    let root = find(&mut parent, seed);
    let keep: Vec<[u32; 4]> = cand.into_iter().filter(|q| find(&mut parent, q[0]) == root).collect();
    let mut map = vec![u32::MAX; skin.len()];
    let mut mesh = IndexedMesh::default();
    for q in &keep {
        let ids = q.map(|i| {
            if map[i as usize] == u32::MAX {
                map[i as usize] = mesh.vertices.len() as u32;
                mesh.vertices.push(to_mm(skin[i as usize], floor, shift));
            }
            map[i as usize]
        });
        mesh.faces.push([ids[0], ids[1], ids[2]]);
        mesh.faces.push([ids[0], ids[2], ids[3]]);
    }
    mesh
}

/// The horizontal section at `z`: its loops' (tape girth, centroid).
fn loops_at(mesh: &IndexedMesh, z: f64) -> Vec<(f64, Point3<f64>)> {
    let cs = scan::section(mesh, Point3::new(0.0, 0.0, z), Vector3::z());
    cs.contours
        .iter()
        .filter(|c| c.points.len() >= 3)
        .map(|c| (tape_perimeter(&c.points.iter().map(|p| (p.x, p.y)).collect::<Vec<_>>()), c.centroid))
        .collect()
}

/// The right leg's loop at `z` (the one furthest to -y, wherever the body sits): its tape girth
/// and centroid.
fn right_girth(mesh: &IndexedMesh, z: f64) -> Option<(f64, Point3<f64>)> {
    loops_at(mesh, z).into_iter().min_by(|a, b| a.1.y.total_cmp(&b.1.y))
}

/// What the fit measures, mm. Lengthwise on the posed body (the knee joint's height, and the
/// right lower leg's calf and girth at `join_z`, which the leg was fitted on upright); girths
/// round the thighs, hips and waist on the body as MakeHuman stands it, legs apart: posed, the
/// thighs press into each other at the top and the sections jump between one loop and two.
/// [knee, calf, join, crotch height, thigh girth just below the crotch, hip girth (the largest
/// single loop between crotch and waist), waist girth at ANSUR's waist height, lower thigh 80
/// mm above the knee joint].
fn measures(m: &Model, w: &[f64; N], join_z: f64) -> ([f64; N], IndexedMesh, f64) {
    let (mut skin, mut raw) = m.at(w);
    let floor_u = skin.iter().map(|v| v.y).fold(f64::MAX, f64::min);
    let as_is = lower_body(m, &skin, floor_u, (0.0, 0.0), 1250.0);
    let knee_u = 100.0 * (m.joint(&raw, "joint-r-knee").y - floor_u);
    let crotch = as_is.vertices.iter().filter(|v| v.y.abs() < 8.0 && v.z > 600.0).map(|v| v.z).fold(f64::MAX, f64::min);
    let thigh = right_girth(&as_is, crotch - 25.0).map_or(f64::NAN, |g| g.0);
    let hips = (0..)
        .map(|k| crotch + 20.0 + 4.0 * k as f64)
        .take_while(|&z| z < ansur::WAIST_HEIGHT - 40.0)
        .filter_map(|z| {
            let l = loops_at(&as_is, z);
            (l.len() == 1).then(|| l[0].0)
        })
        .fold(0.0, f64::max);
    let waist = loops_at(&as_is, ansur::WAIST_HEIGHT).into_iter().map(|l| l.0).fold(0.0, f64::max);
    let lower_thigh = right_girth(&as_is, knee_u + 80.0).map_or(f64::NAN, |g| g.0);
    pose(m, &mut skin, &mut raw);
    let floor = skin.iter().map(|v| v.y).fold(f64::MAX, f64::min);
    let mesh = lower_body(m, &skin, floor, (0.0, 0.0), 1250.0);
    let knee = 100.0 * (m.joint(&raw, "joint-r-knee").y - floor);
    let calf = (0..).map(|k| 150.0 + 4.0 * k as f64).take_while(|&z| z < knee - 40.0).filter_map(|z| right_girth(&mesh, z).map(|g| g.0)).fold(0.0, f64::max);
    let join = right_girth(&mesh, join_z).map_or(f64::NAN, |g| g.0);
    ([knee, calf, join, crotch, thigh, hips, waist, lower_thigh], mesh, floor)
}

pub struct Body {
    /// The body from the floor to `top` mm, in the foot frame, open at the top (for drawing and
    /// for building parts on).
    pub mesh: IndexedMesh,
    pub weights: [f64; N],
    pub measured: [f64; N],
    /// Joint centres, mm: right and left hip, knee, ankle.
    pub joints: HashMap<String, Point3<f64>>,
    /// The legs' tilt taken out at the hips, degrees (sideways, forward), right then left.
    pub tilts: [(f64, f64); 2],
}

pub fn build(scan_path: &Path, top: f64) -> Result<Body, String> {
    let t0 = Instant::now();
    // The foot scan only for the join: its girth and centre 115 mm up.
    let (raw_scan, _, _) = scan::load(scan_path)?;
    let fm = scan::measure(&raw_scan, 20);
    let foot = crate::foot::canonical(&raw_scan, &fm);
    let join_z = crate::leg::JOIN_Z;
    let (foot_girth, foot_c) = crate::leg::girth_at(&foot, join_z).ok_or("no scan section at the join")?;
    let m = Model::load(&makehuman_dir())?;
    println!("MakeHuman: {} skin quads, {} sliders ({:.1} s)", m.quads.len(), N, t0.elapsed().as_secs_f64());
    let goal = [ansur::TIBIAL_HEIGHT, ansur::CALF_CIRC, foot_girth, ansur::CROTCH_HEIGHT, ansur::THIGH_CIRC, ansur::BUTTOCK_CIRC, ansur::WAIST_CIRC, ansur::LOWER_THIGH_CIRC];
    let names = ["knee joint", "calf", "girth at 115", "crotch", "thigh", "hips", "waist", "lower thigh"];
    let mut w = [0.0; N];
    let (m0, _, _) = measures(&m, &w, join_z);
    println!(
        "  unfitted: {}",
        (0..N).map(|i| format!("{} {:.1} ({:.0})", names[i], m0[i], goal[i])).collect::<Vec<_>>().join(", ")
    );
    for it in 0..8 {
        let (f, _, _) = measures(&m, &w, join_z);
        let r: Vec<f64> = (0..N).map(|i| f[i] - goal[i]).collect();
        if r.iter().all(|x| x.abs() < 0.5) {
            break;
        }
        let h = 0.05;
        let mut jac = nalgebra::DMatrix::<f64>::zeros(N, N);
        for j in 0..N {
            let (mut wp, mut wm) = (w, w);
            wp[j] += h;
            wm[j] -= h;
            let (fp, _, _) = measures(&m, &wp, join_z);
            let (fq, _, _) = measures(&m, &wm, join_z);
            for i in 0..N {
                jac[(i, j)] = (fp[i] - fq[i]) / (2.0 * h);
            }
        }
        let dw = jac.lu().solve(&nalgebra::DVector::from_vec(r.clone())).ok_or("fit: singular Jacobian")?;
        for j in 0..N {
            w[j] = (w[j] - dw[j]).clamp(-1.0, 1.0);
        }
        println!("  fit {it}: residuals {} mm", r.iter().map(|x| format!("{x:+.1}")).collect::<Vec<_>>().join(" "));
    }
    let (f, unshifted, floor) = measures(&m, &w, join_z);
    let (_, c) = right_girth(&unshifted, join_z).ok_or("no right leg at the join")?;
    let shift = (foot_c.0 - c.x, foot_c.1 - c.y);
    let (mut skin, mut raw) = m.at(&w);
    let tilts = pose(&m, &mut skin, &mut raw);
    let mesh = lower_body(&m, &skin, floor, shift, top);
    let mut joints = HashMap::new();
    for side in ["r", "l"] {
        for (part, key) in [("upper-leg", "hip"), ("knee", "knee"), ("ankle", "ankle")] {
            joints.insert(format!("{side} {key}"), to_mm(m.joint(&raw, &format!("joint-{side}-{part}")), floor, shift));
        }
    }
    println!(
        "fitted ({:.0} s): {}; sliders {}",
        t0.elapsed().as_secs_f64(),
        (0..N).map(|i| format!("{} {:.1} ({:.0})", names[i], f[i], goal[i])).collect::<Vec<_>>().join(", "),
        TARGETS.iter().zip(w).map(|(n, x)| format!("{} {x:+.2}", n.trim_start_matches("measure-"))).collect::<Vec<_>>().join(", ")
    );
    Ok(Body { mesh, weights: w, measured: f, joints, tilts })
}

/// `cargo run --release -- body <scan> [out]`: the stand-in body, measured against ANSUR II and
/// against the stand-in leg (`out/leg/leg.stl`), drawn.
pub fn run(scan_path: &Path, out: &Path) -> Result<(), String> {
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let docs = Path::new("docs/body");
    std::fs::create_dir_all(docs).map_err(|e| e.to_string())?;
    let b = build(scan_path, 1250.0)?;
    let mut lines = Vec::new();
    let mut say = |s: String| {
        println!("{s}");
        lines.push(s);
    };
    say(format!("stand-in body, waist to feet (track G1), on {}", scan_path.display()));
    say(format!(
        "legs stood at the hips: right {:.1} deg in, {:.1} deg forward; left {:.1}, {:.1}",
        b.tilts[0].0.abs(),
        b.tilts[0].1,
        b.tilts[1].0.abs(),
        b.tilts[1].1
    ));
    for k in ["r hip", "l hip", "r knee", "r ankle"] {
        let p = b.joints[k];
        say(format!("  {k} joint ({:.0}, {:.0}, {:.0}) mm", p.x, p.y, p.z));
    }
    let names = ["knee joint", "calf", "girth at 115", "crotch height", "thigh girth", "hip girth", "waist girth", "lower thigh girth"];
    for (i, n) in names.iter().enumerate() {
        say(format!("  {n}: {:.1} mm", b.measured[i]));
    }
    // The girths were fitted on the body standing as MakeHuman stands it; on the standing pose
    // the lower thigh should read the same. Not fitted, compared: the hip breadth.
    let lower_thigh = right_girth(&b.mesh, b.joints["r knee"].z + 80.0).map_or(f64::NAN, |g| g.0);
    say(format!("  lower thigh girth 80 mm above the knee joint, legs stood: {lower_thigh:.0} mm (fitted with legs apart; ANSUR {:.0})", ansur::LOWER_THIGH_CIRC));
    let (y0, y1) = b.mesh.vertices.iter().filter(|v| (v.z - ansur::TROCHANTER_HEIGHT).abs() < 15.0).fold((f64::MAX, f64::MIN), |a, v| (a.0.min(v.y), a.1.max(v.y)));
    say(format!("  hip breadth at the trochanters' height ({:.0} mm): {:.0} mm (ANSUR {:.0})", ansur::TROCHANTER_HEIGHT, y1 - y0, ansur::HIP_BREADTH));
    // Against the stand-in leg: the right leg's girths at the same heights.
    if let Ok(leg) = cortenforge::mesh::io::load_mesh("out/leg/leg.stl") {
        let rows: Vec<String> = [150.0, 250.0, 360.0, 450.0, 550.0]
            .iter()
            .map(|&z| {
                let a = right_girth(&b.mesh, z).map_or(f64::NAN, |g| g.0);
                let c = crate::leg::girth_at(&leg, z).map_or(f64::NAN, |g| g.0);
                format!("z {z:.0}: body {a:.0}, leg {c:.0}")
            })
            .collect();
        say(format!("  right leg against the stand-in leg (out/leg/leg.stl): {}", rows.join("; ")));
    }
    save_mesh(&b.mesh, out.join("body.stl")).map_err(|e| e.to_string())?;
    let skin = [0.85, 0.70, 0.60];
    let views = [
        View::looking(-Vector3::x(), Vector3::z()),
        View::looking(Vector3::y(), Vector3::z()),
        View::looking(Vector3::x(), Vector3::z()),
        View::looking(Vector3::new(-0.6, 0.65, -0.25), Vector3::z()),
    ];
    render_styled(&[Layer::solid(&b.mesh, skin)], &views, 0.8, &docs.join("body.png"), Style::pretty()).map_err(|e| e.to_string())?;
    std::fs::write(docs.join("summary.txt"), lines.join("\n") + "\n").map_err(|e| e.to_string())?;
    Ok(())
}
