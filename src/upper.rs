//! Track U1: the stand-in body above the waist (arms, hands, head), for the gloves, the elbow
//! pads and the helmet (docs/PLAN.md, track D: the gear).
//!
//! G1 (`body.rs`) fitted MakeHuman's man from the waist down; its arms, hands and head kept
//! MakeHuman's average shape, changed only by the body-weight slider. Here eight more of
//! MakeHuman's sliders are fitted together (Newton, as G1) to ANSUR II's medians for the same men
//! (1700-1800 mm tall, n = 2,222; RESEARCH.md 26), each measure taken as ANSUR's measurer's
//! handbook describes it (Hotzman et al. 2011), as near as a mesh without drawn landmarks allows:
//!
//! - **acromion-radiale length** (334): arm hanging, the shoulder's tip to the elbow, along the
//!   arm. The tip (acromion) is where the shoulder's top turns down into the arm: on the frontal
//!   section through the shoulder joint, the point furthest up-and-out at 45 deg. Radiale (the
//!   top of the radius) is taken at the elbow joint's level.
//! - **radiale-stylion length** (266): the elbow joint to the wrist joint (stylion, the tip of the
//!   radius, taken at the wrist joint's level).
//! - **wrist circumference** (175): at the wrist joint, across the forearm.
//! - **hand breadth** (88) between the knuckles of the index and little fingers (metacarpale II
//!   and V), and **hand length** (193) from the wrist joint's level to the tip of the middle
//!   finger, the hand in line with the forearm and the middle finger straight (MakeHuman's rest
//!   hand is relaxed: fingers curled ~20 deg).
//! - **head length** (200), glabella (the brow's middle) to the furthest point behind;
//!   **head breadth** (154), widest above the ears; **tragion-top of head** (131), the ear's
//!   tragus to the top of the head, upright.
//!
//! Not fitted, compared: hand circumference, head circumference, acromial height, biacromial
//! breadth, stature. The arms' girths aren't fitted: ANSUR took them flexed ("making a muscle").
//!
//! The sliders act above the waist only: the body from the waist down, and so every part built on
//! it (G1-G3), stays as it was (`run` checks this).

use std::collections::HashMap;
use std::path::Path;
use std::time::Instant;

use cortenforge::mesh::io::save_mesh;
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Rotation3, Vector3};

use crate::body::{self, Body};
use crate::leg::{load_target, makehuman_dir, tape_perimeter};
use crate::pose::Motion;
use crate::render::{Layer, Style, View, render_styled};

/// ANSUR II, men 1700-1800 mm tall (2,222 of 4,082), medians in mm (RESEARCH.md 26).
pub mod ansur {
    pub const ACROMION_RADIALE: f64 = 334.0;
    pub const RADIALE_STYLION: f64 = 266.0;
    pub const WRIST_CIRC: f64 = 175.0;
    pub const HAND_BREADTH: f64 = 88.0;
    pub const HAND_LENGTH: f64 = 193.0;
    pub const HEAD_LENGTH: f64 = 200.0;
    pub const HEAD_BREADTH: f64 = 154.0;
    pub const TRAGION_TOP: f64 = 131.0;
    pub const HAND_CIRC: f64 = 212.0;
    pub const HEAD_CIRC: f64 = 574.5;
    pub const ACROMIAL_HEIGHT: f64 = 1435.0;
    pub const BIACROMIAL: f64 = 415.0;
    pub const STATURE: f64 = 1750.0;
    pub const FOREARM_HAND: f64 = 478.0;
    pub const SHOULDER_ELBOW: f64 = 362.0;
    pub const GRIP_LENGTH: f64 = 348.0;
    /// Per man (not from the medians above): forearm-center of grip length minus forearm-hand
    /// length plus hand length, stylion to the gripped dowel's centre; p5, median, p95.
    pub const STYLION_GRIP_RANGE: [f64; 3] = [51.0, 62.0, 74.0];
    /// Per man: forearm-hand length minus hand length minus radiale-stylion length, the elbow's
    /// back (bent 90 deg) to radiale; median.
    pub const OLECRANON_RADIALE: f64 = 19.0;
}

/// The sliders, each a MakeHuman target pair (`-decr` below 0, `-incr` above) or, for the hands,
/// the right and left ones together.
const SLIDERS: [(&str, &[&str]); M] = [
    ("upperarm-length", &["measure/measure-upperarm-length"]),
    ("lowerarm-length", &["measure/measure-lowerarm-length"]),
    ("wrist-circ", &["measure/measure-wrist-circ"]),
    ("hand-scale", &["armslegs/r-hand-scale", "armslegs/l-hand-scale"]),
    ("fingers-length", &["armslegs/r-hand-fingers-length", "armslegs/l-hand-fingers-length"]),
    ("head-depth", &["head/head-scale-depth"]),
    ("head-width", &["head/head-scale-horiz"]),
    ("head-height", &["head/head-scale-vert"]),
];
pub const M: usize = 8;
const NAMES: [&str; M] = ["acromion-radiale", "radiale-stylion", "wrist girth", "hand breadth", "hand length", "head length", "head breadth", "tragion-top of head"];
const GOAL: [f64; M] = [
    ansur::ACROMION_RADIALE,
    ansur::RADIALE_STYLION,
    ansur::WRIST_CIRC,
    ansur::HAND_BREADTH,
    ansur::HAND_LENGTH,
    ansur::HEAD_LENGTH,
    ansur::HEAD_BREADTH,
    ansur::TRAGION_TOP,
];

/// Tragion, the top of the tragus where it joins the head (just under the front of the helix's
/// crus): a vertex of MakeHuman's base mesh, picked by eye on G1's body's right ear (2026-11-13,
/// side and three-quarter views at 14 px/mm).
const TRAGION: usize = 5499;

/// The fitted body above the waist.
pub struct Upper {
    pub weights: [f64; M],
    pub measured: [f64; M],
    pub checks: Checks,
    /// The control mesh with every slider (G1's and these), rest frame (dm).
    pub raw: Vec<Vector3<f64>>,
}

/// Something measured, for drawing: its skin, and the points and sections it was measured by (mm,
/// foot frame).
pub struct Drawn {
    pub mesh: IndexedMesh,
    pub marks: Vec<Point3<f64>>,
    pub loops: Vec<Vec<Point3<f64>>>,
}

/// What the fit measures, and what it measured on: the body with the arms hung and the right hand
/// in the measuring pose; the right hand and lower forearm; the head.
pub struct Measured {
    pub values: [f64; M],
    pub checks: Checks,
    pub body: Drawn,
    pub hand: Drawn,
    pub head: Drawn,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Checks {
    pub hand_circ: f64,
    pub head_circ: f64,
    pub acromial_height: f64,
    pub biacromial: f64,
    pub stature: f64,
    /// How far the tip found for the hand length is from the middle finger's end joint, mm (it
    /// should be the middle finger's tip: ~20 mm).
    pub tip_off: f64,
    /// The widest section above the ears: its height above tragion, mm.
    pub euryon_up: f64,
    /// The ear's top, above tragion, mm.
    pub ear_top_up: f64,
}

/// The sliders' targets, loaded.
struct Targets {
    t: Vec<(Vec<Vector3<f64>>, Vec<Vector3<f64>>)>,
}

impl Targets {
    fn load(n: usize) -> Result<Self, String> {
        let dir = makehuman_dir().join("targets");
        let mut t = Vec::new();
        for (_, files) in SLIDERS {
            let mut d = vec![Vector3::zeros(); n];
            let mut i = vec![Vector3::zeros(); n];
            for f in files {
                for (acc, s) in [(&mut d, "decr"), (&mut i, "incr")] {
                    let x = load_target(&dir.join(format!("{f}-{s}.target")), n)?;
                    for (a, b) in acc.iter_mut().zip(&x) {
                        *a += b;
                    }
                }
            }
            t.push((d, i));
        }
        Ok(Targets { t })
    }

    fn apply(&self, raw: &[Vector3<f64>], w: &[f64; M]) -> Vec<Vector3<f64>> {
        let mut x = raw.to_vec();
        for (k, (d, i)) in self.t.iter().enumerate() {
            let (t, s) = if w[k] < 0.0 { (d, -w[k]) } else { (i, w[k]) };
            for (a, b) in x.iter_mut().zip(t) {
                *a += b * s;
            }
        }
        x
    }
}

/// The parts of the smoothed skin each measure looks at (by index: the same in any pose and for
/// any slider).
struct Regions {
    hand: Vec<[u32; 4]>,
    shoulder: [Vec<[u32; 4]>; 2],
    head: Vec<[u32; 4]>,
}

fn quads_where(b: &Body, keep: impl Fn(usize) -> bool) -> Vec<[u32; 4]> {
    b.model.quads.iter().copied().filter(|q| q.iter().all(|&i| keep(i as usize))).collect()
}

impl Regions {
    fn new(b: &Body, raw: &[Vector3<f64>]) -> Self {
        let m = &b.model;
        let skin = m.s2.apply(&m.s1.apply(&raw[..m.n_body]));
        let mm = |v: Vector3<f64>| body::to_mm(v, b.floor, b.shift);
        let j = |name: &str| mm(m.rig.joint(raw, name).unwrap_or_default());
        let (el, wr) = (j("lowerarm01.R____head"), j("wrist.R____head"));
        let a = (wr - el).normalize();
        let len = (wr - el).norm();
        let hand = quads_where(b, |i| {
            let d = mm(skin[i]) - el;
            let t = d.dot(&a);
            t > 0.45 * len && (d - a * t).norm() < 130.0
        });
        let shoulder = [("R", -1.0), ("L", 1.0)].map(|(s, out)| {
            let gh = j(&format!("upperarm01.{s}____head"));
            quads_where(b, |i| {
                let v = mm(skin[i]);
                (v - gh).norm() < 160.0 && out * (v.y - gh.y) > -50.0
            })
        });
        let tz = mm(skin[TRAGION]).z;
        let head = quads_where(b, |i| mm(skin[i]).z > tz - 60.0);
        Regions { hand, shoulder, head }
    }
}

/// The loop of a section nearest `near`.
fn loop_near(mesh: &IndexedMesh, point: Point3<f64>, normal: Vector3<f64>, near: Point3<f64>) -> Option<Vec<Point3<f64>>> {
    let cs = crate::scan::section(mesh, point, normal);
    cs.contours
        .into_iter()
        .filter(|c| c.points.len() >= 3)
        .min_by(|a, b| (a.centroid - near).norm().total_cmp(&(b.centroid - near).norm()))
        .map(|c| c.points)
}

/// The tape girth of a loop lying in a plane with in-plane axes `e1`, `e2`.
fn tape(points: &[Point3<f64>], e1: &Vector3<f64>, e2: &Vector3<f64>) -> f64 {
    tape_perimeter(&points.iter().map(|p| (p.coords.dot(e1), p.coords.dot(e2))).collect::<Vec<_>>())
}

/// The rotations that hang both arms straight down at the sides, straighten the wrists (the
/// middle knuckle on the forearm's line) and the right middle finger, all in the rest frame
/// (`pose::Rig::motions`).
fn measuring_pose(b: &Body, raw: &[Vector3<f64>]) -> std::collections::HashMap<String, Rotation3<f64>> {
    let rig = &b.model.rig;
    let j = |name: &str| rig.joint(raw, name).unwrap_or_default();
    let between = |a: Vector3<f64>, c: Vector3<f64>| Rotation3::rotation_between(&a, &c).unwrap_or_else(Rotation3::identity);
    let mut m = std::collections::HashMap::new();
    for s in ["R", "L"] {
        let (sh, el, wr) = (j(&format!("upperarm01.{s}____head")), j(&format!("lowerarm01.{s}____head")), j(&format!("wrist.{s}____head")));
        let mcp3 = j(&format!("finger3-1.{s}____head"));
        // The upper arm straight down (MakeHuman: y up).
        m.insert(format!("upperarm01.{s}"), between(el - sh, -Vector3::y()));
        // The forearm on the upper arm's line, as it is in the rest frame (the bone's turn is
        // carried by the upper arm's).
        m.insert(format!("lowerarm01.{s}"), between(wr - el, el - sh));
        // The hand on the forearm's line.
        m.insert(format!("wrist.{s}"), between(mcp3 - wr, wr - el));
        // The middle finger straight, on the hand's line: each joint on its parent's.
        let f = |k: u32, e: &str| j(&format!("finger3-{k}.{s}____{e}"));
        let d = [1, 2, 3].map(|k| f(k, "tail") - f(k, "head"));
        m.insert(format!("finger3-1.{s}"), between(d[0], mcp3 - wr));
        m.insert(format!("finger3-2.{s}"), between(d[1], d[0]));
        m.insert(format!("finger3-3.{s}"), between(d[2], d[1]));
    }
    m
}

fn measure(b: &Body, raw: &[Vector3<f64>], reg: &Regions) -> Result<Measured, String> {
    let m = &b.model;
    let rig = &m.rig;
    let motions = rig.motions(raw, &measuring_pose(b, raw));
    let praw = rig.skin(raw, &motions);
    let skin = m.s2.apply(&m.s1.apply(&praw[..m.n_body]));
    let mm = |v: Vector3<f64>| body::to_mm(v, b.floor, b.shift);
    // A joint where it is posed: moved with the bone it heads.
    let at = |bone: &str| -> Point3<f64> {
        let i = rig.index[bone];
        let h = rig.joint(raw, &rig.bones[i].head).unwrap_or_default();
        mm(motions[i].apply(h))
    };
    let mesh_of = |quads: &[[u32; 4]]| body::mesh_of_quads(&skin, b.floor, b.shift, quads);

    // The arms (right; the left only for the shoulders' breadth). Hung, the arm's axis is -z.
    let (el, wr) = (at("lowerarm01.R"), at("wrist.R"));
    let radiale_stylion = (wr - el).norm();
    let mut acromion = [Point3::origin(); 2];
    for (k, (s, out)) in [("R", -1.0), ("L", 1.0)].into_iter().enumerate() {
        let gh = at(&format!("upperarm01.{s}"));
        let sh = mesh_of(&reg.shoulder[k]);
        let cs = crate::scan::section(&sh, gh, Vector3::x());
        let d = Vector3::new(0.0, out, 1.0).normalize();
        acromion[k] = cs
            .points
            .iter()
            .filter(|p| p.z > gh.z - 30.0)
            .copied()
            .max_by(|a, c| a.coords.dot(&d).total_cmp(&c.coords.dot(&d)))
            .ok_or(format!("no shoulder section on the {s} side"))?;
    }
    let acromion_radiale = acromion[0].z - el.z;
    let hand = mesh_of(&reg.hand);
    let wrist = loop_near(&hand, wr, Vector3::z(), wr).ok_or("no wrist section")?;
    let wrist_circ = tape(&wrist, &Vector3::x(), &Vector3::y());
    // The hand length: the wrist joint's level to the lowest point of the hung hand.
    let tip = hand.vertices.iter().copied().min_by(|a, c| a.z.total_cmp(&c.z)).ok_or("no hand")?;
    let hand_length = wr.z - tip.z;
    let tip_off = (tip - at("finger3-3.R")).norm();
    // Across the knuckles: the plane through the index and little fingers' knuckles that holds
    // the palm's normal.
    let (mcp2, mcp5) = (at("finger2-1.R"), at("finger5-1.R"));
    let e = (mcp2 - mcp5).normalize();
    let a = -Vector3::z();
    let n = (a - e * a.dot(&e)).normalize();
    let mid = Point3::from((mcp2.coords + mcp5.coords) / 2.0);
    let knuckles = loop_near(&hand, mid, n, mid).ok_or("no section across the knuckles")?;
    let along: Vec<f64> = knuckles.iter().map(|p| (p - mid).dot(&e)).collect();
    let hand_breadth = along.iter().cloned().fold(f64::MIN, f64::max) - along.iter().cloned().fold(f64::MAX, f64::min);
    let hand_circ = tape(&knuckles, &e, &n.cross(&e));

    // The head (upright as MakeHuman holds it: taken as the Frankfurt plane).
    let head = mesh_of(&reg.head);
    let tragion = mm(skin[TRAGION]);
    let top = head.vertices.iter().copied().max_by(|a, c| a.z.total_cmp(&c.z)).ok_or("no head")?;
    let tragion_top = top.z - tragion.z;
    let mid_y = at("head").y;
    let eye = mm(rig.joint(raw, "eye.R____head").unwrap_or_default());
    let midline = crate::scan::section(&head, Point3::new(0.0, mid_y, top.z - 60.0), Vector3::y());
    let glabella = midline
        .points
        .iter()
        .filter(|p| p.z > eye.z - 5.0 && p.z < eye.z + 35.0)
        .copied()
        .max_by(|a, c| a.x.total_cmp(&c.x))
        .ok_or("no glabella")?;
    let back = midline
        .points
        .iter()
        .filter(|p| p.x < glabella.x - 100.0)
        .copied()
        .max_by(|a, c| (*a - glabella).norm().total_cmp(&(*c - glabella).norm()))
        .ok_or("no back of the head")?;
    let head_length = (back - glabella).norm();
    // The widest section above the ears' tops. The ear's top: going up from tragion, where the
    // right side's outermost skin beside the ear steps in from the ear's rim to the head (by over
    // 5 mm in 1 mm of height).
    let outermost = |z: f64| {
        head.vertices.iter().filter(|v| (v.z - z).abs() < 1.0 && (v.x - tragion.x).abs() < 40.0 && v.y < mid_y).map(|v| v.y).fold(f64::MAX, f64::min)
    };
    let mut ear_top = None;
    let mut prev = outermost(tragion.z);
    for k in 1..60 {
        let z = tragion.z + k as f64;
        let y = outermost(z);
        if y - prev > 5.0 {
            ear_top = Some(z - 1.0);
            break;
        }
        prev = y;
    }
    let ear_top = ear_top.ok_or("no ear top within 60 mm above tragion")?;
    let mut breadth = (0.0, 0.0, Vec::new());
    let mut z = ear_top + 3.0;
    while z < top.z - 10.0 {
        let cs = crate::scan::section(&head, Point3::new(0.0, mid_y, z), Vector3::z());
        if let Some(c) = cs.contours.iter().max_by(|a, c| a.area.total_cmp(&c.area)) {
            let w = c.bounds.1.y - c.bounds.0.y;
            if w > breadth.0 {
                breadth = (w, z, c.points.clone());
            }
        }
        z += 2.0;
    }
    let head_breadth = breadth.0;
    // The head's girth: the largest over planes just above the brow (10 mm over glabella, in
    // front) and round the back (through the back's furthest point, +-20 mm), level side to side.
    let front = Point3::new(glabella.x, mid_y, glabella.z + 10.0);
    let mut head_circ = (0.0, Vec::new());
    for dz in (-20..=20).step_by(4) {
        let b2 = Point3::new(back.x, mid_y, back.z + dz as f64);
        let along = (b2 - front).normalize();
        let nrm = along.cross(&Vector3::y()).normalize();
        if let Some(l) = loop_near(&head, front, nrm, Point3::from((front.coords + b2.coords) / 2.0)) {
            let g = tape(&l, &along, &Vector3::y());
            if g > head_circ.0 {
                head_circ = (g, l);
            }
        }
    }
    let checks = Checks {
        hand_circ,
        head_circ: head_circ.0,
        acromial_height: acromion[0].z,
        biacromial: (acromion[1].y - acromion[0].y).abs(),
        stature: top.z,
        tip_off,
        euryon_up: breadth.1 - tragion.z,
        ear_top_up: ear_top - tragion.z,
    };
    let values = [acromion_radiale, radiale_stylion, wrist_circ, hand_breadth, hand_length, head_length, head_breadth, tragion_top];
    let hand = Drawn { mesh: hand, marks: vec![wr, tip, mcp2, mcp5], loops: vec![wrist, knuckles] };
    let head = Drawn { mesh: head, marks: vec![tragion, top, glabella, back], loops: vec![head_circ.1, breadth.2] };
    let mut marks = vec![acromion[0], acromion[1], el];
    marks.extend(hand.marks.iter().chain(&head.marks));
    let loops = hand.loops.iter().chain(&head.loops).cloned().collect();
    let body = Drawn { mesh: body::mesh_of_quads(&skin, b.floor, b.shift, &m.quads), marks, loops };
    Ok(Measured { values, checks, body, hand, head })
}

/// Fits the sliders above the waist on G1's body.
pub fn fit(b: &Body) -> Result<(Upper, Measured), String> {
    let t0 = Instant::now();
    let targets = Targets::load(b.raw.len())?;
    let reg = Regions::new(b, &b.raw);
    let mut w = [0.0; M];
    let eval = |w: &[f64; M]| measure(b, &targets.apply(&b.raw, w), &reg);
    let first = eval(&w)?;
    println!("  upper body, unfitted: {}", (0..M).map(|i| format!("{} {:.1} ({:.0})", NAMES[i], first.values[i], GOAL[i])).collect::<Vec<_>>().join(", "));
    for it in 0..10 {
        let f = eval(&w)?.values;
        let r: Vec<f64> = (0..M).map(|i| f[i] - GOAL[i]).collect();
        println!("  upper fit {it}: residuals {} mm", r.iter().map(|x| format!("{x:+.1}")).collect::<Vec<_>>().join(" "));
        if r.iter().all(|x| x.abs() < 0.3) {
            break;
        }
        let h = 0.05;
        let mut jac = nalgebra::DMatrix::<f64>::zeros(M, M);
        for k in 0..M {
            let (mut wp, mut wm) = (w, w);
            wp[k] += h;
            wm[k] -= h;
            let (fp, fq) = (eval(&wp)?.values, eval(&wm)?.values);
            for i in 0..M {
                jac[(i, k)] = (fp[i] - fq[i]) / (2.0 * h);
            }
        }
        let dw = jac.lu().solve(&nalgebra::DVector::from_vec(r)).ok_or("upper fit: singular Jacobian")?;
        for k in 0..M {
            w[k] = (w[k] - dw[k]).clamp(-1.0, 1.0);
        }
    }
    let raw = targets.apply(&b.raw, &w);
    let out = measure(b, &raw, &reg)?;
    println!("  upper body fitted ({:.0} s)", t0.elapsed().as_secs_f64());
    Ok((Upper { weights: w, measured: out.values, checks: out.checks, raw }, out))
}

// ---- U2: the arm bent at the elbow, the hand gripping (2026-11-13) ----

/// A straight round bar held in the hand (mm, foot frame).
#[derive(Clone, Copy, Debug)]
pub struct Bar {
    pub centre: Point3<f64>,
    pub axis: Vector3<f64>,
    pub radius: f64,
}

impl Bar {
    fn dist(&self, p: &Point3<f64>) -> f64 {
        let d = p - self.centre;
        (d - self.axis * d.dot(&self.axis)).norm()
    }
}

/// ANSUR II's dowel for the forearm-center of grip length: 1-1/4" across (RESEARCH.md 26).
pub const DOWEL_DIAMETER: f64 = 31.75;

type Pose = HashMap<String, Rotation3<f64>>;

/// A direction in the foot frame (x forward, y left, z up) in MakeHuman's (x left, y up, z
/// forward). The map is a turn (det +1), so angles about an axis carry over.
fn to_mh(v: Vector3<f64>) -> Vector3<f64> {
    Vector3::new(v.y, v.z, v.x)
}

/// The right hand where it is posed (mm): the wrist joint, the hand's axis (wrist to the middle
/// knuckle), the palm's normal (out of the palm) and the knuckles' line across it, and the
/// middle of the knuckles.
struct HandFrame {
    wr: Point3<f64>,
    axis: Vector3<f64>,
    palm: Vector3<f64>,
    across: Vector3<f64>,
    knuckles: Point3<f64>,
}

/// A bone's end where it is posed, mm.
fn posed_end(b: &Body, raw: &[Vector3<f64>], mo: &[Motion], bone: &str, tail: bool) -> Point3<f64> {
    let rig = &b.model.rig;
    let i = rig.index[bone];
    let name = if tail { &rig.bones[i].tail } else { &rig.bones[i].head };
    body::to_mm(mo[i].apply(rig.joint(raw, name).unwrap_or_default()), b.floor, b.shift)
}

fn hand_frame(b: &Body, raw: &[Vector3<f64>], mo: &[Motion]) -> HandFrame {
    // The frame from the wrist and the knuckles, which move with the hand as one; which way is
    // out of the palm from the rest pose, where the ring finger curls toward it (MakeHuman's rest
    // hand is relaxed; a posed hand's fingers may be straight).
    let frame = |at: &dyn Fn(&str, bool) -> Point3<f64>| {
        let wr = at("wrist.R", false);
        let axis = (at("finger3-1.R", false) - wr).normalize();
        let (m2, m5) = (at("finger2-1.R", false), at("finger5-1.R", false));
        let a = m2 - m5;
        let across = (a - axis * a.dot(&axis)).normalize();
        (wr, axis, across, axis.cross(&across).normalize(), m2, m5)
    };
    let rest = vec![Motion::identity(); mo.len()];
    let at_rest = |bone: &str, tail: bool| posed_end(b, raw, &rest, bone, tail);
    let (_, _, _, n_rest, _, _) = frame(&at_rest);
    let out = (at_rest("finger4-3.R", true) - at_rest("finger4-1.R", false)).dot(&n_rest).signum();
    let (wr, axis, across, n, m2, m5) = frame(&|bone, tail| posed_end(b, raw, mo, bone, tail));
    HandFrame { wr, axis, palm: n * out, across, knuckles: Point3::from((m2.coords + m5.coords) / 2.0) }
}

/// The right arm as ANSUR measures the elbow's lengths: the upper arm hung, the elbow bent
/// `elbow` deg forward, the forearm turned so the palm faces in (thumb up), the hand in line with
/// the forearm, the middle finger straight or, if `relaxed`, as MakeHuman holds it. The left arm
/// hung.
fn elbow_pose(b: &Body, raw: &[Vector3<f64>], elbow: f64, relaxed: bool) -> Pose {
    let rig = &b.model.rig;
    let mut m = measuring_pose(b, raw);
    if relaxed {
        for k in 1..=3 {
            m.remove(&format!("finger3-{k}.R"));
        }
    }
    let (ru, ll) = (m["upperarm01.R"], m["lowerarm01.R"]);
    // The forearm from down (-y) to forward (+z), about MakeHuman's x, then carried by the upper
    // arm's turn (as the knee in `pose::leg_rotations`).
    let f = Rotation3::from_axis_angle(&Vector3::x_axis(), -elbow.to_radians());
    m.insert("lowerarm01.R".into(), ru.inverse() * f * ru * ll);
    let mo = rig.motions(raw, &m);
    let el = posed_end(b, raw, &mo, "lowerarm01.R", false);
    let h = hand_frame(b, raw, &mo);
    let af = (h.wr - el).normalize();
    // The palm in, toward the body's middle (+y for the right hand): the lower forearm's twist
    // bone turned about the forearm's axis (it heads the forearm's middle).
    let flat = |v: Vector3<f64>| (v - af * v.dot(&af)).normalize();
    let (p, w) = (flat(h.palm), flat(Vector3::y()));
    let ang = p.cross(&w).dot(&af).atan2(p.dot(&w));
    let t = Rotation3::from_axis_angle(&nalgebra::Unit::new_normalize(to_mh(af)), ang);
    let g1 = mo[rig.index["lowerarm01.R"]].r;
    let l2 = m.get("lowerarm02.R").copied().unwrap_or_else(Rotation3::identity);
    m.insert("lowerarm02.R".into(), g1.inverse() * t * g1 * l2);
    m
}

/// Each control vertex's bone: the one with the most weight on it.
fn dominant(b: &Body) -> Vec<usize> {
    let rig = &b.model.rig;
    let n = b.model.n_body;
    let mut best = vec![(usize::MAX, 0.0f64); n];
    for (k, ws) in rig.weights.iter().enumerate() {
        for &(i, w) in ws {
            let i = i as usize;
            if i < n && w > best[i].1 {
                best[i] = (k, w);
            }
        }
    }
    best.into_iter().map(|x| x.0).collect()
}

/// How far each digit's joints may bend, deg: guards from the hand's normal ranges as commonly
/// quoted (sources differ, e.g. the fingers' end joints 65-90); the run reports any that binds.
/// The thumb (its base swinging out of the palm, then its middle and end joints), the fingers
/// (knuckle, middle, end).
const THUMB_LIMIT: [f64; 3] = [60.0, 50.0, 80.0];
const FINGER_LIMIT: [f64; 3] = [90.0, 100.0, 80.0];

/// The hand closed on a bar, and how.
pub struct Grip {
    pub pose: Pose,
    pub bar: Bar,
    /// How far each joint bent, deg: thumb, index, middle, ring, little; base to tip.
    pub angles: [[f64; 3]; 5],
    /// What stopped each joint: "bar", "palm" (behind the bar), "limit"; the thumb's base
    /// "clear" (out of the bar's way) or "-" (it never was in it).
    pub stops: [[&'static str; 3]; 5],
    /// The middle finger wraps the bar to its tip: its end joint stopped on the bar or the palm,
    /// not at its limit.
    pub wrapped: bool,
}

/// Closes the right hand of `base` (the elbow pose, fingers straight) on a bar of `radius` lying
/// across the palm, `u` mm from the wrist joint along the hand.
/// - The bar rests on the palm's middle (the skin over the middle and ring fingers' metacarpals).
///   The pads either side, which give round a held bar, are let overlap it: rigid skin can't give,
///   and resting on them held the bar ~45 mm off the hand's bones, past where the fingers reach
///   round.
/// - Each finger wraps it from the base out, as a hand closes on a bar and as Irwin and Radwin
///   (2008) posed theirs: its knuckle bends until a link touches the bar, or a middle or end link
///   the palm behind it, or the joint's limit; then the next joint; each about the bar's
///   direction.
/// - The thumb first swings out of the palm (its base turning square to the palm) until it
///   clears the bar, then its middle and end joints bend toward the bar until they touch it.
fn close_on(b: &Body, raw: &[Vector3<f64>], base: &Pose, dom: &[usize], radius: f64, u: f64) -> Result<Grip, String> {
    let rig = &b.model.rig;
    let mm = |v: Vector3<f64>| body::to_mm(v, b.floor, b.shift);
    let from_mm = |p: Point3<f64>| Vector3::new((p.y - b.shift.1) / 100.0, p.z / 100.0 + b.floor, (p.x - b.shift.0) / 100.0);
    let mo0 = rig.motions(raw, base);
    let p0 = rig.skin(raw, &mo0);
    let h = hand_frame(b, raw, &mo0);
    let palm_bones: Vec<usize> = ["metacarpal2.R", "metacarpal3.R"].iter().map(|n| rig.index[*n]).collect();
    let mut lift = f64::MIN;
    for (i, &d) in dom.iter().enumerate() {
        if !palm_bones.contains(&d) {
            continue;
        }
        let q = mm(p0[i]) - h.wr;
        let s = q.dot(&h.axis) - u;
        if s.abs() < radius {
            lift = lift.max(q.dot(&h.palm) + (radius * radius - s * s).sqrt());
        }
    }
    if lift == f64::MIN {
        return Err(format!("no palm under a bar {u:.0} mm from the wrist"));
    }
    let across_mid = (h.knuckles - h.wr).dot(&h.across);
    let bar = Bar { centre: h.wr + h.axis * u + h.palm * lift + h.across * across_mid, axis: h.across, radius };
    let palm_at = bar.centre - h.palm * radius;
    let e = to_mh(bar.axis);
    let c_mh = from_mm(bar.centre);
    let ends = |mo: &[Motion], bi: usize| {
        let head = mo[bi].apply(rig.joint(raw, &rig.bones[bi].head).unwrap_or_default());
        let tail = mo[bi].apply(rig.joint(raw, &rig.bones[bi].tail).unwrap_or_default());
        (head, tail)
    };
    // The digits: bones, owned control vertices, and each joint's axis in its parent's posed frame.
    let mut bones = [[0usize; 3]; 5];
    let mut owners: Vec<Vec<Vec<usize>>> = vec![vec![Vec::new(); 3]; 5];
    let mut axes = [[Vector3::x(); 3]; 5];
    for k in 0..5 {
        for j in 0..3 {
            let bi = rig.index[&format!("finger{}-{}.R", k + 1, j + 1)];
            bones[k][j] = bi;
            owners[k][j] = (0..dom.len()).filter(|&i| dom[i] == bi).collect();
        }
        let parent = |j: usize| rig.bones[bones[k][j]].parent.unwrap_or(0);
        let (_, tip) = ends(&mo0, bones[k][2]);
        if k == 0 {
            // The thumb's base: square to its bone and the palm, the way that lifts its tip out of
            // the palm.
            let (head, tail) = ends(&mo0, bones[0][0]);
            let mut w = (tail - head).cross(&to_mh(h.palm)).normalize();
            if w.cross(&(tip - head)).dot(&to_mh(h.palm)) < 0.0 {
                w = -w;
            }
            axes[0][0] = mo0[parent(0)].r.inverse() * w;
        } else {
            for j in 0..3 {
                let (head, _) = ends(&mo0, bones[k][j]);
                let w = if e.cross(&(tip - head)).dot(&to_mh(h.palm)) >= 0.0 { e } else { -e };
                axes[k][j] = mo0[parent(j)].r.inverse() * w;
            }
        }
    }
    let limit = |k: usize, j: usize| if k == 0 { THUMB_LIMIT[j] } else { FINGER_LIMIT[j] };
    let pose_of = |th: &[[f64; 3]; 5], axes: &[[Vector3<f64>; 3]; 5]| {
        let mut m = base.clone();
        for k in 0..5 {
            for j in 0..3 {
                let name = &rig.bones[bones[k][j]].name;
                let l0 = base.get(name).copied().unwrap_or_else(Rotation3::identity);
                let r = Rotation3::from_axis_angle(&nalgebra::Unit::new_normalize(axes[k][j]), th[k][j].to_radians());
                m.insert(name.clone(), r * l0);
            }
        }
        m
    };
    let mut th = [[0.0f64; 3]; 5];
    let mut stops = [[""; 3]; 5];
    let skin_of = |th: &[[f64; 3]; 5], axes: &[[Vector3<f64>; 3]; 5]| {
        let mo = rig.motions(raw, &pose_of(th, axes));
        let s = rig.skin(raw, &mo);
        (mo, s)
    };
    // The thumb out of the bar's way: its two phalanges clear of it (its base, the pad at the
    // root of the thumb, lies under the bar with the palm's other pads).
    let in_bar = |skin: &[Vector3<f64>], k: usize| (1..3).any(|j| owners[k][j].iter().any(|&i| bar.dist(&mm(skin[i])) < radius));
    stops[0][0] = "-";
    loop {
        let (_, skin) = skin_of(&th, &axes);
        if !in_bar(&skin, 0) {
            if th[0][0] > 0.0 {
                stops[0][0] = "clear";
            }
            break;
        }
        if th[0][0] >= THUMB_LIMIT[0] {
            stops[0][0] = "limit";
            break;
        }
        th[0][0] += 1.0;
    }
    // Its middle and end joints toward the bar, square to the bone, from where it now is.
    {
        let (mo, _) = skin_of(&th, &axes);
        for j in 1..3 {
            let bi = bones[0][j];
            let (head, tail) = ends(&mo, bi);
            let near = c_mh + e * (head - c_mh).dot(&e);
            let w = (tail - head).cross(&(near - head)).normalize();
            axes[0][j] = mo[rig.bones[bi].parent.unwrap_or(0)].r.inverse() * w;
        }
    }
    // A link touching: the bar, or (a finger's middle or end link) the palm behind the bar.
    let touches = |k: usize, p: &Point3<f64>, jj: usize| {
        if bar.dist(p) < radius {
            Some("bar")
        } else if k > 0 && jj > 0 && (p - bar.centre).dot(&h.axis) < -0.5 * radius && (p - palm_at).dot(&h.palm) < 0.0 {
            Some("palm")
        } else {
            None
        }
    };
    // The thumb's middle and end joints one after the other. The fingers' three joints together,
    // at one rate (GraspIt's auto-grasp): when a link touches, the joints from the base to it
    // stop and those beyond keep closing.
    let mut at_thumb = 1usize;
    let mut active = [[true; 3]; 5];
    active[0] = [false; 3];
    for _ in 0..400 {
        if at_thumb >= 3 && active.iter().flatten().all(|a| !a) {
            break;
        }
        let prev = th;
        if at_thumb < 3 {
            th[0][at_thumb] = (th[0][at_thumb] + 1.0).min(limit(0, at_thumb));
        }
        for k in 1..5 {
            for j in 0..3 {
                if active[k][j] {
                    th[k][j] = (th[k][j] + 1.0).min(limit(k, j));
                }
            }
        }
        let (_, skin) = skin_of(&th, &axes);
        let hit = |k: usize, from: usize| (from..3).rev().find_map(|jj| owners[k][jj].iter().find_map(|&i| touches(k, &mm(skin[i]), jj)).map(|w| (jj, w)));
        if at_thumb < 3 {
            let j = at_thumb;
            if let Some((_, w)) = hit(0, j) {
                th[0][j] = prev[0][j];
                stops[0][j] = w;
                at_thumb += 1;
            } else if th[0][j] >= limit(0, j) {
                stops[0][j] = "limit";
                at_thumb += 1;
            }
        }
        for k in 1..5 {
            if let Some((jj, w)) = hit(k, 0) {
                for q in 0..=jj {
                    if active[k][q] {
                        th[k][q] = prev[k][q];
                        active[k][q] = false;
                        stops[k][q] = w;
                    }
                }
            }
            for q in 0..3 {
                if active[k][q] && th[k][q] >= limit(k, q) {
                    active[k][q] = false;
                    stops[k][q] = "limit";
                }
            }
        }
    }
    let wrapped = stops[2][2] == "bar" || stops[2][2] == "palm";
    Ok(Grip { pose: pose_of(&th, &axes), bar, angles: th, stops, wrapped })
}

/// What U2's checks measure on the bent arm (mm): ANSUR's three elbow lengths, and the grip.
pub struct ElbowChecks {
    pub forearm_hand: f64,
    pub shoulder_elbow: f64,
    /// The back of the elbow behind the elbow joint, mm.
    pub elbow_back: f64,
    /// Per dowel position (mm from the wrist joint: ANSUR's p5, median, p95): the grip, the
    /// elbow's back to the dowel's centre, and how far the hand's skin overlaps the dowel (mm).
    pub grips: Vec<(f64, Grip, f64, [f64; 3])>,
    pub hand: IndexedMesh,
    pub arm: IndexedMesh,
}

/// The arm bent 90 deg at the elbow, as ANSUR measured it: the forearm-hand and shoulder-elbow
/// lengths with the hand straight; then the hand closed on ANSUR's dowel where ANSUR's men held
/// it (`ansur::STYLION_GRIP`), from straight fingers, and the elbow's back to the dowel's centre
/// (ANSUR's forearm-center of grip length). The elbow's back ("olecranon, rear": "the rearmost
/// point of the elbow") is taken within 10 mm of the elbow joint's height: higher up, the back of
/// the upper arm stands further back.
pub fn elbow_checks(b: &Body, raw: &[Vector3<f64>], acromion_z: f64) -> Result<ElbowChecks, String> {
    let m = &b.model;
    let rig = &m.rig;
    let mm = |v: Vector3<f64>| body::to_mm(v, b.floor, b.shift);
    let reg = Regions::new(b, raw);
    let rest = m.s2.apply(&m.s1.apply(&raw[..m.n_body]));
    let el0 = mm(rig.joint(raw, "lowerarm01.R____head").unwrap_or_default());
    let hand_set: std::collections::HashSet<u32> = reg.hand.iter().flatten().copied().collect();
    let elbow: Vec<usize> = (0..rest.len()).filter(|&i| (mm(rest[i]) - el0).norm() < 80.0 && !hand_set.contains(&(i as u32))).collect();
    let smooth = |pose: &Pose| {
        let mo = rig.motions(raw, pose);
        let praw = rig.skin(raw, &mo);
        (m.s2.apply(&m.s1.apply(&praw[..m.n_body])), mo)
    };
    // The hand straight.
    let (skin, mo) = smooth(&elbow_pose(b, raw, 90.0, false));
    let el = posed_end(b, raw, &mo, "lowerarm01.R", false);
    let af = (posed_end(b, raw, &mo, "wrist.R", false) - el).normalize();
    let rear = |skin: &[Vector3<f64>]| {
        elbow.iter().map(|&i| mm(skin[i])).filter(|p| (p.z - el.z).abs() < 10.0).map(|p| p.coords.dot(&af)).fold(f64::MAX, f64::min)
    };
    let tip = hand_set.iter().map(|&i| mm(skin[i as usize]).coords.dot(&af)).fold(f64::MIN, f64::max);
    let forearm_hand = tip - rear(&skin);
    let elbow_back = el.coords.dot(&af) - rear(&skin);
    let bottom = elbow.iter().map(|&i| mm(skin[i]).z).fold(f64::MAX, f64::min);
    let shoulder_elbow = acromion_z - bottom;
    // The grip, from straight fingers.
    let mut base = elbow_pose(b, raw, 90.0, true);
    straighten_fingers(b, raw, &mut base);
    let dom = dominant(b);
    let mut grips = Vec::new();
    for u in ansur::STYLION_GRIP_RANGE {
        let g = close_on(b, raw, &base, &dom, 0.5 * DOWEL_DIAMETER, u)?;
        let (skin, _) = smooth(&g.pose);
        let len = g.bar.centre.coords.dot(&af) - rear(&skin);
        // How far into the dowel: the palm (its pads), the thumb's phalanges, the fingers' (the
        // skin's control points, each by its bone).
        let thumb: Vec<usize> = ["finger1-2.R", "finger1-3.R"].iter().map(|n| rig.index[*n]).collect();
        let fingers: Vec<usize> = (2..=5).flat_map(|k| (1..=3).map(move |s| format!("finger{k}-{s}.R"))).map(|n| rig.index[&n]).collect();
        let mut overlap = [0.0f64; 3];
        for &i in &hand_set {
            let i = i as usize;
            if i >= m.n_body {
                continue;
            }
            let c = if thumb.contains(&dom[i]) { 1 } else if fingers.contains(&dom[i]) { 2 } else { 0 };
            overlap[c] = overlap[c].max(g.bar.radius - g.bar.dist(&mm(skin[i])));
        }
        grips.push((u, g, len, overlap));
    }
    let (_, g, _, _) = &grips[1];
    let (skin, _) = smooth(&g.pose);
    let hand = body::mesh_of_quads(&skin, b.floor, b.shift, &reg.hand);
    let arm = body::mesh_of_quads(&skin, b.floor, b.shift, &m.quads);
    Ok(ElbowChecks { forearm_hand, shoulder_elbow, elbow_back, grips, hand, arm })
}

/// The right hand's four fingers straight: each joint on its parent's line, the first on its
/// metacarpal's (the thumb as MakeHuman holds it).
fn straighten_fingers(b: &Body, raw: &[Vector3<f64>], pose: &mut Pose) {
    let rig = &b.model.rig;
    let j = |name: &str| rig.joint(raw, name).unwrap_or_default();
    let between = |a: Vector3<f64>, c: Vector3<f64>| Rotation3::rotation_between(&a, &c).unwrap_or_else(Rotation3::identity);
    for k in 2..=5 {
        let mc = j(&format!("metacarpal{}.R____tail", k - 1)) - j(&format!("metacarpal{}.R____head", k - 1));
        let d = [1, 2, 3].map(|s| j(&format!("finger{k}-{s}.R____tail")) - j(&format!("finger{k}-{s}.R____head")));
        pose.insert(format!("finger{k}-1.R"), between(d[0], mc));
        pose.insert(format!("finger{k}-2.R"), between(d[1], d[0]));
        pose.insert(format!("finger{k}-3.R"), between(d[2], d[1]));
    }
}

/// A bar as a mesh, `length` long.
fn bar_mesh(bar: &Bar, length: f64) -> IndexedMesh {
    let n = 48;
    let e1 = bar.axis.cross(&Vector3::x()).try_normalize(1e-9).unwrap_or_else(|| bar.axis.cross(&Vector3::y()).normalize());
    let e2 = bar.axis.cross(&e1);
    let mut m = IndexedMesh::default();
    for end in [-0.5, 0.5] {
        for k in 0..n {
            let a = std::f64::consts::TAU * k as f64 / n as f64;
            m.vertices.push(bar.centre + bar.axis * (end * length) + (e1 * a.cos() + e2 * a.sin()) * bar.radius);
        }
    }
    for k in 0..n as u32 {
        let (a, c) = (k, (k + 1) % n as u32);
        m.faces.push([a, c, n as u32 + a]);
        m.faces.push([c, n as u32 + c, n as u32 + a]);
    }
    m
}

/// `cargo run --release -- upper <scan> [out]`: the stand-in body above the waist, fitted to
/// ANSUR II and drawn (docs/upper/).
pub fn run(scan: &Path, out: &Path) -> Result<(), String> {
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let docs = Path::new("docs/upper");
    std::fs::create_dir_all(docs).map_err(|e| e.to_string())?;
    let b = body::build(scan, 1250.0)?;
    let (u, meas) = fit(&b)?;
    let mut lines = Vec::new();
    let mut say = |s: String| {
        println!("{s}");
        lines.push(s);
    };
    say(format!("stand-in body above the waist (track U1), on G1's body ({})", scan.display()));
    say("  fitted to ANSUR II medians, men 1700-1800 mm (mm):".into());
    for i in 0..M {
        say(format!("    {:<22} {:>6.1}  (ANSUR {:.0})", NAMES[i], u.measured[i], GOAL[i]));
    }
    say(format!(
        "  sliders: {}",
        SLIDERS.iter().zip(u.weights).map(|((n, _), x)| format!("{n} {x:+.2}")).collect::<Vec<_>>().join(", ")
    ));
    let c = u.checks;
    say("  not fitted, compared (mm):".into());
    say(format!("    hand circumference     {:>6.1}  (ANSUR {:.0})", c.hand_circ, ansur::HAND_CIRC));
    say(format!("    head circumference     {:>6.1}  (ANSUR {:.1})", c.head_circ, ansur::HEAD_CIRC));
    say(format!("    acromial height        {:>6.1}  (ANSUR {:.0})", c.acromial_height, ansur::ACROMIAL_HEIGHT));
    say(format!("    biacromial breadth     {:>6.1}  (ANSUR {:.0})", c.biacromial, ansur::BIACROMIAL));
    say(format!("    stature                {:>6.1}  (ANSUR {:.0})", c.stature, ansur::STATURE));
    say(format!("  the hand's lowest point is {:.1} mm from the middle finger's end joint; the ear's top {:.0} mm above tragion, the head's widest section above it {:.0} mm above tragion", c.tip_off, c.ear_top_up, c.euryon_up));
    // Below the waist nothing moved: the standing body's lower part, before and after.
    let m = &b.model;
    let before = m.s2.apply(&m.s1.apply(&b.raw[..m.n_body]));
    let after = m.s2.apply(&m.s1.apply(&u.raw[..m.n_body]));
    let moved = b.quads.iter().flatten().map(|&i| 100.0 * (after[i as usize] - before[i as usize]).norm()).fold(0.0, f64::max);
    say(format!("  the body below the waist (G1's mesh, to 1250 mm) moved at most {moved:.4} mm"));
    // U2: the elbow bent 90 deg, the hand straight and then gripping ANSUR's dowel.
    let t0 = Instant::now();
    let ec = elbow_checks(&b, &u.raw, c.acromial_height)?;
    say(format!("the arm bent 90 deg at the elbow, palm in (track U2; {:.0} s), against ANSUR (mm):", t0.elapsed().as_secs_f64()));
    say(format!("    forearm-hand length    {:>6.1}  (ANSUR {:.0})", ec.forearm_hand, ansur::FOREARM_HAND));
    say(format!("    shoulder-elbow length  {:>6.1}  (ANSUR {:.0})", ec.shoulder_elbow, ansur::SHOULDER_ELBOW));
    say(format!("    the elbow's back {:.1} mm behind the elbow joint (ANSUR's men: {:.0} behind radiale, median)", ec.elbow_back, ansur::OLECRANON_RADIALE));
    let digit = ["thumb", "index", "middle", "ring", "little"];
    for (u, g, len, overlap) in &ec.grips {
        say(format!(
            "  the hand closed on a {DOWEL_DIAMETER} mm dowel {u:.0} mm from the wrist joint: forearm-center of grip {len:.1} (ANSUR {:.0}); the middle finger {} it to its tip; into the dowel by up to {:.1} mm (the palm's pads), {:.1} (the thumb), {:.1} (the fingers)",
            ansur::GRIP_LENGTH,
            if g.wrapped { "wraps" } else { "doesn't wrap" },
            overlap[0],
            overlap[1],
            overlap[2]
        ));
        for k in 0..5 {
            say(format!(
                "    {:<6} bent {} deg (base to tip), stopped by {}",
                digit[k],
                g.angles[k].iter().map(|a| format!("{a:.0}")).collect::<Vec<_>>().join(" / "),
                g.stops[k].join(" / ")
            ));
        }
    }
    let g = &ec.grips[1].1;
    std::fs::write(docs.join("summary.txt"), lines.join("\n") + "\n").map_err(|e| e.to_string())?;
    {
        let skin = [0.85, 0.70, 0.60];
        let dowel = bar_mesh(&g.bar, 140.0);
        let grey = [0.55, 0.55, 0.6];
        // From the thumb's side (above), from the front (the fingers), from below, from behind.
        let views = [
            View::looking(-Vector3::z(), Vector3::x()),
            View::looking(-Vector3::x(), Vector3::z()),
            View::looking(Vector3::z(), Vector3::x()),
            View::looking(Vector3::new(0.3, -1.0, 0.0), Vector3::z()),
        ];
        render_styled(&[Layer::solid(&ec.hand, skin), Layer::solid(&dowel, grey)], &views, 2.0, &docs.join("grip.png"), Style::pretty()).map_err(|e| e.to_string())?;
        render_styled(&[Layer::solid(&ec.arm, skin), Layer::solid(&dowel, grey)], &[View::looking(Vector3::y(), Vector3::z())], 0.35, &docs.join("grip_arm.png"), Style::pretty())
            .map_err(|e| e.to_string())?;
    }

    // Pictures: the whole body as MakeHuman stands it; the measuring pose; the hand and the head
    // with what was measured.
    let rest = body::mesh_of_quads(&after, b.floor, b.shift, &m.quads);
    save_mesh(&rest, out.join("body_whole.stl")).map_err(|e| e.to_string())?;
    let skin = [0.85, 0.70, 0.60];
    let views = [
        View::looking(-Vector3::x(), Vector3::z()),
        View::looking(Vector3::y(), Vector3::z()),
        View::looking(Vector3::x(), Vector3::z()),
        View::looking(Vector3::new(-0.6, 0.65, -0.25), Vector3::z()),
    ];
    render_styled(&[Layer::solid(&rest, skin)], &views, 0.35, &docs.join("body.png"), Style::pretty()).map_err(|e| e.to_string())?;
    let draw = |d: &Drawn, mark: f64, line: f64, views: &[View], px: f64, name: &str| {
        let marks = dots(&d.marks, mark);
        let pts: Vec<Point3<f64>> = d.loops.iter().flatten().copied().collect();
        let loops = dots(&pts, line);
        let layers = [Layer::solid(&d.mesh, skin), Layer::solid(&marks, [0.85, 0.15, 0.1]), Layer::solid(&loops, [0.1, 0.3, 0.85])];
        render_styled(&layers, views, px, &docs.join(name), Style::pretty()).map_err(|e| e.to_string())
    };
    draw(&meas.body, 6.0, 1.5, &views[..2], 0.35, "measuring_pose.png")?;
    // The hand hung, from the back of the hand, the thumb side and the palm; the head from the
    // front, the right and behind.
    let around = [View::looking(Vector3::x(), Vector3::z()), View::looking(Vector3::y(), Vector3::z()), View::looking(-Vector3::x(), Vector3::z())];
    draw(&meas.hand, 1.5, 0.4, &around, 2.0, "hand.png")?;
    let around = [View::looking(-Vector3::x(), Vector3::z()), View::looking(Vector3::y(), Vector3::z()), View::looking(Vector3::x(), Vector3::z())];
    draw(&meas.head, 2.0, 0.6, &around, 1.4, "head.png")?;
    println!("drew docs/upper/body.png, measuring_pose.png, hand.png, head.png; out/upper/body_whole.stl");
    Ok(())
}

/// Small octahedra at points, for marking them in a picture.
pub fn dots(points: &[Point3<f64>], r: f64) -> IndexedMesh {
    let mut m = IndexedMesh::default();
    let d = [Vector3::x(), -Vector3::x(), Vector3::y(), -Vector3::y(), Vector3::z(), -Vector3::z()];
    let faces = [[0, 2, 4], [2, 1, 4], [1, 3, 4], [3, 0, 4], [2, 0, 5], [1, 2, 5], [3, 1, 5], [0, 3, 5]];
    for p in points {
        let k = m.vertices.len() as u32;
        m.vertices.extend(d.iter().map(|v| p + v * r));
        m.faces.extend(faces.iter().map(|f| [k + f[0], k + f[1], k + f[2]]));
    }
    m
}
