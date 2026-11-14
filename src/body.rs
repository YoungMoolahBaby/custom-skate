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
use nalgebra::{Point3, Rotation3, Vector3};

use crate::leg::{Subdivision, ansur as ansur_leg, load_base, load_target, makehuman_dir, tape_perimeter};
use crate::pose::{LegPose, Motion, Rig, leg_rotations};
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
pub struct Model {
    base: Vec<Vector3<f64>>,
    targets: Vec<(Vec<Vector3<f64>>, Vec<Vector3<f64>>)>,
    raw: Vec<Vector3<f64>>,
    raw_targets: Vec<(Vec<Vector3<f64>>, Vec<Vector3<f64>>)>,
    joints: HashMap<String, Vec<u32>>,
    quads: Vec<[u32; 4]>,
    /// The skeleton and skin weights, and the two smoothing passes (to smooth a posed control
    /// mesh).
    rig: Rig,
    s1: Subdivision,
    s2: Subdivision,
    n_body: usize,
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
        let rig = Rig::load(dir)?;
        let quads = s2.quads.clone();
        Ok(Model { base: sub(&raw), targets, raw, raw_targets, joints: base.joints, quads, rig, s1, s2, n_body: base.n_body })
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

    /// Where a kneecap sits on the fitted body (`side` "r" or "l"), in the rest frame (dm), with
    /// `soft` mm of skin and fat over it (`kneecap::SOFT`).
    pub fn kneecap_at(&self, raw: &[Vector3<f64>], side: &str, soft: f64) -> Result<Kneecap, String> {
        let skin = self.s2.apply(&self.s1.apply(&raw[..self.n_body]));
        let joint = self.joint(raw, &format!("joint-{side}-knee"));
        let up = (joint - self.joint(raw, &format!("joint-{side}-ankle"))).normalize();
        let front = (Vector3::z() - up * up.z).normalize();
        let across = up.cross(&front);
        // The skin's front on the knee's midline `dz` mm above the joint.
        let front_at = |dz: f64| -> Option<Vector3<f64>> {
            let q = joint + up * (dz / 100.0);
            skin.iter()
                .filter(|v| {
                    let d = *v - q;
                    d.dot(&up).abs() < 0.03 && d.dot(&across).abs() < 0.1
                })
                .map(|v| (v - q).dot(&front))
                .fold(None, |a: Option<f64>, x| Some(a.map_or(x, |a| a.max(x))))
                .map(|d| q + front * d)
        };
        let skin_mid = front_at(ansur_leg::KNEE_HEIGHT_MID_PATELLA - ansur_leg::TIBIAL_HEIGHT).ok_or("no skin over the kneecap")?;
        let centre = skin_mid - front * ((soft + 0.5 * kneecap::THICK) / 100.0);
        let low = centre - up * (0.5 * kneecap::LENGTH / 100.0);
        // The tuberosity: on the shin's front, `soft` under the skin, the tendon's length from the
        // lower pole.
        let tub_at = |dz: f64| front_at(dz).map(|s| s - front * (soft / 100.0));
        let mut prev: Option<(f64, f64)> = None;
        let mut tub = None;
        for k in 0..=120 {
            let dz = -(k as f64);
            let Some(t) = tub_at(dz) else { continue };
            let l = 100.0 * (t - low).norm();
            if let Some((dz0, l0)) = prev
                && l0 < kneecap::TENDON
                && l >= kneecap::TENDON
            {
                let dz = dz0 + (dz - dz0) * (kneecap::TENDON - l0) / (l - l0);
                tub = tub_at(dz);
                break;
            }
            prev = Some((dz, l));
        }
        let tub = tub.ok_or("no tibial tuberosity within 120 mm below the knee")?;
        Ok(Kneecap { skin: skin_mid, centre, low, tub, joint, up, front })
    }

    /// Kneecaps on both legs (`Rig::add_patella`): the skin over each takes its weight within
    /// `kneecap::HOLD_IN` mm of the skin over its centre, none past `kneecap::HOLD_OUT`, smoothly
    /// between (on the knee's front only).
    fn add_kneecaps(&mut self, raw: &[Vector3<f64>]) -> Result<(), String> {
        for (side, s) in [("r", "R"), ("l", "L")] {
            let k = self.kneecap_at(raw, side, kneecap::SOFT)?;
            let share: Vec<(u32, f64)> = raw[..self.n_body]
                .iter()
                .enumerate()
                .filter_map(|(i, v)| {
                    let d = 100.0 * (v - k.skin).norm();
                    let a = 1.0 - crate::pad::smoothstep(kneecap::HOLD_IN, kneecap::HOLD_OUT, d);
                    ((v - k.joint).dot(&k.front) > 0.0 && a > 0.0).then_some((i as u32, a))
                })
                .collect();
            self.rig.add_patella(raw, s, k.centre, k.low, k.tub, &share)?;
        }
        Ok(())
    }
}

/// A kneecap's points on the body standing, rest frame (dm): the skin over its centre, its
/// centre, its lower pole, the tibial tuberosity; the knee joint, and the leg's up and front.
pub struct Kneecap {
    pub skin: Vector3<f64>,
    pub centre: Vector3<f64>,
    pub low: Vector3<f64>,
    pub tub: Vector3<f64>,
    pub joint: Vector3<f64>,
    pub up: Vector3<f64>,
    pub front: Vector3<f64>,
}

/// The kneecap's size and place (RESEARCH.md 24), mm.
pub mod kneecap {
    /// Patella length and thickness, and the patellar tendon's length: MRI of 1,002 knees, 18-50
    /// years, 712 of them men (Janani et al. 2023), means.
    pub const LENGTH: f64 = 40.3;
    pub const THICK: f64 = 18.6;
    pub const TENDON: f64 = 45.2;
    /// Skin and fat over the kneecap and the tuberosity: assumed, not found in a source; the
    /// `body` run moves it to 3 and 8 mm to see what it changes.
    pub const SOFT: f64 = 5.0;
    /// The skin moving with it: all of it within the patella's half-length of the skin over its
    /// centre, none past its length.
    pub const HOLD_IN: f64 = 0.5 * LENGTH;
    pub const HOLD_OUT: f64 = LENGTH;
}

/// The body posed: each leg first stood (turned at the hip until its knee joint is over its
/// ankle joint: MakeHuman stands with its feet ~44 cm apart), then flexed by `legs` (hip, knee,
/// ankle), by MakeHuman's skeleton and skin weights (`pose.rs`) on the control mesh, then
/// smoothed. Returns the smoothed skin, the joints (dm, MakeHuman's frame: "r hip", "r knee",
/// "r ankle", and "l ..."), and each leg's tilt taken out to stand it (degrees in, forward).
/// (Before 2026-11-12 the legs were turned about the hips by hand, blended over the hip.)
pub fn posed(m: &Model, raw: &[Vector3<f64>], legs: [LegPose; 2]) -> (Vec<Vector3<f64>>, HashMap<String, Vector3<f64>>, [(f64, f64); 2]) {
    let mut stand = [Rotation3::identity(); 2];
    let mut tilts = [(0.0, 0.0); 2];
    for (k, side) in ["r", "l"].iter().enumerate() {
        let d = m.joint(raw, &format!("joint-{side}-knee")) - m.joint(raw, &format!("joint-{side}-ankle"));
        stand[k] = Rotation3::rotation_between(&d, &Vector3::y()).unwrap_or_else(Rotation3::identity);
        tilts[k] = (d.x.atan2(d.y).to_degrees(), d.z.atan2(d.y).to_degrees());
    }
    let motions = m.rig.motions(raw, &leg_rotations(stand, legs));
    let praw = m.rig.skin(raw, &motions);
    let skin = m.s2.apply(&m.s1.apply(&praw[..m.n_body]));
    // The joints ride on their bones: the hip on the thigh (it turns about it), the knee at the
    // thigh's end, the ankle at the shin's end.
    let mut joints = HashMap::new();
    for (side, s) in [("r", "R"), ("l", "L")] {
        let mv = |bone: &str| m.rig.index.get(&format!("{bone}.{s}")).map(|&i| motions[i]).unwrap_or_else(Motion::identity);
        joints.insert(format!("{side} hip"), mv("upperleg01").apply(m.joint(raw, &format!("joint-{side}-upper-leg"))));
        joints.insert(format!("{side} knee"), mv("upperleg02").apply(m.joint(raw, &format!("joint-{side}-knee"))));
        joints.insert(format!("{side} ankle"), mv("lowerleg02").apply(m.joint(raw, &format!("joint-{side}-ankle"))));
    }
    (skin, joints, tilts)
}

/// MakeHuman (dm) to the foot frame (mm), floor at 0, then `shift` in x and y.
pub fn to_mm(v: Vector3<f64>, floor: f64, shift: (f64, f64)) -> Point3<f64> {
    Point3::new(100.0 * v.z + shift.0, 100.0 * v.x + shift.1, 100.0 * (v.y - floor))
}

/// A motion in MakeHuman's frame read across to the foot frame. `to_mm` is A(v) = 100 P v + c,
/// P the axis turn (x, y, z) -> (z, x, y); a motion M there is A M A^-1 here: P R P^T, and
/// P R P^T (-c) + 100 P t + c.
pub fn motion_to_mm(m: &Motion, floor: f64, shift: (f64, f64)) -> Motion {
    let p = nalgebra::Matrix3::new(0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    let c = Vector3::new(shift.0, shift.1, -100.0 * floor);
    let r = Rotation3::from_matrix_unchecked(p * m.r.matrix() * p.transpose());
    Motion { r, t: -(r * c) + 100.0 * (p * m.t) + c }
}

/// The body below `z_top` mm (its connected piece holding the pelvis: hands hang free of it),
/// as triangles in the foot frame.
fn lower_body(m: &Model, skin: &[Vector3<f64>], floor: f64, shift: (f64, f64), z_top: f64) -> IndexedMesh {
    mesh_of_quads(skin, floor, shift, &lower_quads(m, skin, floor, z_top))
}

/// `lower_body`'s quads, as indices into the smoothed skin.
fn lower_quads(m: &Model, skin: &[Vector3<f64>], floor: f64, z_top: f64) -> Vec<[u32; 4]> {
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
    cand.into_iter().filter(|q| find(&mut parent, q[0]) == root).collect()
}

/// Triangles from some of the smoothed skin's quads, vertices numbered as they first appear (so
/// the same quads give the same mesh in any pose).
fn mesh_of_quads(skin: &[Vector3<f64>], floor: f64, shift: (f64, f64), keep: &[[u32; 4]]) -> IndexedMesh {
    let mut map = vec![u32::MAX; skin.len()];
    let mut mesh = IndexedMesh::default();
    for q in keep {
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
    let (skin, raw) = m.at(w);
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
    let (skin, joints, _) = posed(m, &raw, [LegPose::default(); 2]);
    let floor = skin.iter().map(|v| v.y).fold(f64::MAX, f64::min);
    let mesh = lower_body(m, &skin, floor, (0.0, 0.0), 1250.0);
    let knee = 100.0 * (joints["r knee"].y - floor);
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
    /// To pose it again: the model, the fitted control mesh, and its floor and shift into the
    /// foot frame (standing).
    pub model: std::sync::Arc<Model>,
    pub raw: Vec<Vector3<f64>>,
    pub floor: f64,
    pub shift: (f64, f64),
    /// The standing mesh's quads (indices into the smoothed skin), to pose it face for face.
    pub quads: std::sync::Arc<Vec<[u32; 4]>>,
}

impl Body {
    /// A bone's index (as `moves`), by MakeHuman's name ("lowerleg01.R").
    pub fn bone(&self, name: &str) -> Option<usize> {
        self.model.rig.index.get(name).copied()
    }

    /// A bone and every bone below it (bones come parents first).
    pub fn below(&self, name: &str) -> Vec<usize> {
        let Some(top) = self.bone(name) else { return Vec::new() };
        let bones = &self.model.rig.bones;
        let mut inside = vec![false; bones.len()];
        inside[top] = true;
        for (i, b) in bones.iter().enumerate() {
            if b.parent.is_some_and(|p| inside[p]) {
                inside[i] = true;
            }
        }
        (0..bones.len()).filter(|&i| inside[i]).collect()
    }

    fn stand(&self) -> [Rotation3<f64>; 2] {
        let m = &self.model;
        let mut stand = [Rotation3::identity(); 2];
        for (k, side) in ["r", "l"].iter().enumerate() {
            let d = m.joint(&self.raw, &format!("joint-{side}-knee")) - m.joint(&self.raw, &format!("joint-{side}-ankle"));
            stand[k] = Rotation3::rotation_between(&d, &Vector3::y()).unwrap_or_else(Rotation3::identity);
        }
        stand
    }

    /// The right kneecap (`pose::Patella`): where it sits, and how it moves through the knee's
    /// range, and how much the assumed skin over it (`kneecap::SOFT`) changes that.
    pub fn kneecap_report(&self) -> Result<Vec<String>, String> {
        let m = &self.model;
        let p = m.rig.patellae.first().ok_or("no kneecap")?;
        let k = m.kneecap_at(&self.raw, "r", kneecap::SOFT)?;
        let st = self.stand();
        let turn = |knee: f64| leg_rotations(st, [LegPose { knee, ..Default::default() }, LegPose::default()])["lowerleg01.R"];
        let axis = (st[0].inverse() * Vector3::x()).normalize();
        let off_axis = |v: Vector3<f64>| 100.0 * { let d = v - p.knee; d - axis * d.dot(&axis) }.norm();
        let tendon = p.tub - p.low;
        let n_all = m.rig.weights[p.bone].iter().filter(|w| w.1 > 0.0).count();
        let mut out = vec![
            format!(
                "right kneecap (a bone MakeHuman lacks; RESEARCH.md 24): its centre {:.1} mm under the skin over it ({:.0} mm above the knee joint), {:.1} mm from the knee's axis; its tendon {:.1} mm to the tibial tuberosity, {:.0} mm below the joint, at {:+.0} deg to the leg's axis (+: leaning forward at the top); {} skin control vertices move with it, by share",
                kneecap::SOFT + 0.5 * kneecap::THICK,
                100.0 * (k.skin - k.joint).dot(&k.up),
                off_axis(p.centre),
                100.0 * tendon.norm(),
                -100.0 * (p.tub - k.joint).dot(&k.up),
                (-tendon.dot(&k.front)).atan2(-tendon.dot(&k.up)).to_degrees(),
                n_all
            ),
            "  through the knee's range: its turn on the thigh and on the shin, deg; how far round the knee its centre goes per degree of the knee's; its tendon's stretch and the centre's change of distance from the knee's turning centre, mm:".into(),
        ];
        for knee in [0.0, 15.0, 30.0, 45.0, 70.0, 90.0, 110.0, 135.0] {
            let t = turn(knee);
            let (a, share) = p.on_thigh(&t).ok_or(format!("kneecap: no turn round the knee keeps its tendon at knee {knee}"))?;
            let tub = t * (p.tub - p.knee) + p.knee;
            let stretch = 100.0 * ((a.apply(p.low) - tub).norm() - (p.low - p.tub).norm());
            let radius = 100.0 * ((a.apply(p.centre) - p.knee).norm() - (p.centre - p.knee).norm());
            let on_shin = crate::cup::turn_deg(&(t.inverse() * a.r));
            out.push(format!(
                "    knee {knee:>4.0}: on the thigh {:>5.1}, on the shin {:>5.1}; round the knee {share:.3} per deg; tendon {stretch:+.1e}, radius {radius:+.1e}",
                crate::cup::turn_deg(&a.r),
                on_shin
            ));
        }
        // The assumed soft tissue: the skin over its centre carried with 3 or 8 mm instead.
        let mut worst = (0.0f64, 0.0, 0.0);
        for soft in [3.0, 8.0] {
            let q = m.kneecap_at(&self.raw, "r", soft)?;
            let alt = crate::pose::Patella { centre: q.centre, low: q.low, tub: q.tub, ..p.clone() };
            for knee in (0..=135).step_by(5).map(f64::from) {
                let t = turn(knee);
                let (a, _) = p.on_thigh(&t).ok_or("kneecap: no turn")?;
                let (b, _) = alt.on_thigh(&t).ok_or(format!("kneecap with {soft} mm over it: no turn at knee {knee}"))?;
                let d = 100.0 * (a.apply(k.skin) - b.apply(k.skin)).norm();
                if d > worst.0 {
                    worst = (d, soft, knee);
                }
            }
        }
        out.push(format!(
            "  the skin over it assumed {} mm: at 3 or 8 mm instead, the skin over its centre lands at most {:.1} mm elsewhere (at {} mm, knee {:.0})",
            kneecap::SOFT,
            worst.0,
            worst.1,
            worst.2
        ));
        Ok(out)
    }

    /// Each bone's motion from standing to `legs`, mm in the foot frame (index as `model.rig`;
    /// one past the last bone is "no bone": the share of a vertex's weight that stays put).
    pub fn moves(&self, legs: [LegPose; 2]) -> Vec<Motion> {
        let m = &self.model;
        let st = self.stand();
        let rest = m.rig.motions(&self.raw, &leg_rotations(st, [LegPose::default(); 2]));
        let pose = m.rig.motions(&self.raw, &leg_rotations(st, legs));
        let mut out: Vec<Motion> = rest.iter().zip(&pose).map(|(r0, q)| motion_to_mm(&q.after(&r0.inverse()), self.floor, self.shift)).collect();
        out.push(Motion::identity());
        out
    }

    /// Skin weights for points near the skin, standing (mm): each point takes a mix of the
    /// weights of its `k` nearest control-mesh vertices (by 1 / (d^2 + 25 mm^2)), from its own
    /// side of the body only (so a point on the inside of the right thigh doesn't take the left
    /// thigh's), as MakeHuman fits clothes to the body. Bone index as `moves`.
    pub fn weights_near(&self, pts: &[Point3<f64>], k: usize) -> Vec<Vec<(usize, f64)>> {
        let m = &self.model;
        let n = m.n_body;
        let none = m.rig.bones.len();
        let rest = m.rig.skin(&self.raw, &m.rig.motions(&self.raw, &leg_rotations(self.stand(), [LegPose::default(); 2])));
        let ctrl: Vec<Point3<f64>> = rest[..n].iter().map(|&v| to_mm(v, self.floor, self.shift)).collect();
        // Each control vertex's weights, short of 1 made up by "no bone", over 1 normalised.
        let mut vw: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n];
        for (b, ws) in m.rig.weights.iter().enumerate() {
            for &(i, w) in ws {
                if (i as usize) < n {
                    vw[i as usize].push((b, w));
                }
            }
        }
        for l in &mut vw {
            let s: f64 = l.iter().map(|x| x.1).sum();
            if s > 1.0 {
                l.iter_mut().for_each(|x| x.1 /= s);
            } else if s < 1.0 {
                l.push((none, 1.0 - s));
            }
        }
        const CELL: f64 = 25.0;
        let key = |p: &Point3<f64>| ((p.x / CELL).floor() as i32, (p.y / CELL).floor() as i32, (p.z / CELL).floor() as i32);
        let mut grid: HashMap<(i32, i32, i32), Vec<u32>> = HashMap::new();
        for (i, p) in ctrl.iter().enumerate() {
            grid.entry(key(p)).or_default().push(i as u32);
        }
        let mid = self.shift.1;
        pts.iter()
            .map(|q| {
                let (cx, cy, cz) = key(q);
                let mut near: Vec<(f64, u32)> = Vec::new();
                for reach in [2, 4] {
                    near.clear();
                    for dx in -reach..=reach {
                        for dy in -reach..=reach {
                            for dz in -reach..=reach {
                                for &i in grid.get(&(cx + dx, cy + dy, cz + dz)).map(|v| v.as_slice()).unwrap_or(&[]) {
                                    let c = ctrl[i as usize];
                                    if (q.y - mid).abs() > 20.0 && (c.y - mid) * (q.y - mid) < 0.0 {
                                        continue;
                                    }
                                    near.push(((c - q).norm_squared(), i));
                                }
                            }
                        }
                    }
                    if near.len() >= k {
                        break;
                    }
                }
                near.sort_by(|a, b| a.0.total_cmp(&b.0));
                near.truncate(k);
                let mut w: Vec<(usize, f64)> = Vec::new();
                let total: f64 = near.iter().map(|(d2, _)| 1.0 / (d2 + 25.0)).sum();
                for (d2, i) in &near {
                    let a = 1.0 / (d2 + 25.0) / total;
                    for &(b, x) in &vw[*i as usize] {
                        match w.iter_mut().find(|e| e.0 == b) {
                            Some(e) => e.1 += a * x,
                            None => w.push((b, a * x)),
                        }
                    }
                }
                w
            })
            .collect()
    }

    /// The body in another pose, in the same frame as standing (the pelvis stays put), up to
    /// `top` mm; and its joints, mm.
    pub fn posed(&self, legs: [LegPose; 2], top: f64) -> (IndexedMesh, HashMap<String, Point3<f64>>) {
        let (skin, pj, _) = posed(&self.model, &self.raw, legs);
        let mesh = lower_body(&self.model, &skin, self.floor, self.shift, top);
        (mesh, pj.into_iter().map(|(k, v)| (k, to_mm(v, self.floor, self.shift))).collect())
    }

    /// The body in another pose with the standing mesh's faces and vertex order (`posed` picks
    /// its faces again by height, so a foot raised past `top`, the hip flexed 110 deg with the
    /// knee straight, changed the mesh: 2026-11-13).
    pub fn posed_like_standing(&self, legs: [LegPose; 2]) -> (IndexedMesh, HashMap<String, Point3<f64>>) {
        let (skin, pj, _) = posed(&self.model, &self.raw, legs);
        let mesh = mesh_of_quads(&skin, self.floor, self.shift, &self.quads);
        (mesh, pj.into_iter().map(|(k, v)| (k, to_mm(v, self.floor, self.shift))).collect())
    }
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
    let (_, raw) = m.at(&w);
    // Kneecaps on the fitted body (they don't move standing, so the fit doesn't see them).
    let mut m = m;
    m.add_kneecaps(&raw)?;
    let (skin, pj, tilts) = posed(&m, &raw, [LegPose::default(); 2]);
    let quads = lower_quads(&m, &skin, floor, top);
    let mesh = mesh_of_quads(&skin, floor, shift, &quads);
    let joints: HashMap<String, Point3<f64>> = pj.into_iter().map(|(k, v)| (k, to_mm(v, floor, shift))).collect();
    println!(
        "fitted ({:.0} s): {}; sliders {}",
        t0.elapsed().as_secs_f64(),
        (0..N).map(|i| format!("{} {:.1} ({:.0})", names[i], f[i], goal[i])).collect::<Vec<_>>().join(", "),
        TARGETS.iter().zip(w).map(|(n, x)| format!("{} {x:+.2}", n.trim_start_matches("measure-"))).collect::<Vec<_>>().join(", ")
    );
    Ok(Body { mesh, weights: w, measured: f, joints, tilts, raw, floor, shift, quads: std::sync::Arc::new(quads), model: std::sync::Arc::new(m) })
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
    for l in b.kneecap_report()? {
        say(l);
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

/// `cargo run --release -- poses <scan>`: the body in a few poses (both legs alike), side by
/// side, from the right: standing; a skating stance (hip 60 deg, knee 90, ankle 30, so the shin
/// leans forward over a flat foot); kneeling to block a shot (hip 10, knee 100, toes pointed
/// back 20); a deep crouch (hip 90, knee 130). To see how MakeHuman's skin weights bend the knee
/// and the hip.
pub fn run_poses(scan_path: &Path) -> Result<(), String> {
    let docs = Path::new("docs/body");
    std::fs::create_dir_all(docs).map_err(|e| e.to_string())?;
    let b = build(scan_path, 1250.0)?;
    let poses = [
        ("standing", LegPose::default()),
        ("skating stance", LegPose { hip: 60.0, knee: 90.0, ankle: 30.0 }),
        ("kneeling", LegPose { hip: 10.0, knee: 100.0, ankle: -20.0 }),
        ("deep crouch", LegPose { hip: 90.0, knee: 130.0, ankle: 0.0 }),
    ];
    let mut meshes = Vec::new();
    for (k, (name, p)) in poses.iter().enumerate() {
        let (mut m, j) = b.posed([*p, *p], 1250.0);
        let (h, kn, a) = (j["r hip"], j["r knee"], j["r ankle"]);
        let thigh = (kn - h).normalize();
        let shin = (a - kn).normalize();
        println!(
            "{name}: knee joint ({:.0}, {:.0}, {:.0}), ankle ({:.0}, {:.0}, {:.0}); thigh to shin {:.0} deg; {} triangles",
            kn.x,
            kn.y,
            kn.z,
            a.x,
            a.y,
            a.z,
            thigh.dot(&shin).clamp(-1.0, 1.0).acos().to_degrees(),
            m.faces.len()
        );
        for v in &mut m.vertices {
            v.x += 900.0 * k as f64;
        }
        meshes.push(m);
    }
    let skin = [0.85, 0.70, 0.60];
    let layers: Vec<Layer> = meshes.iter().map(|m| Layer::solid(m, skin)).collect();
    render_styled(&layers, &[View::looking(Vector3::y(), Vector3::z())], 0.6, &docs.join("poses.png"), Style::pretty()).map_err(|e| e.to_string())?;
    render_styled(&layers, &[View::looking(Vector3::new(-0.5, 0.8, -0.3), Vector3::z())], 0.6, &docs.join("poses_34.png"), Style::pretty()).map_err(|e| e.to_string())?;
    println!("drew docs/body/poses.png, poses_34.png");
    Ok(())
}
