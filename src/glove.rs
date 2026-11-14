//! U2, glove round 1 (2026-11-14): the glove's hard pieces on the hand closed on the shaft.
//!
//! Jon's glove (PLAN.md, Gloves): tight, "almost like a lacrosse glove but safe for hockey", no
//! empty space inside, a short cuff, the outside looking like a hockey glove (a later round). The
//! hard pieces ride on a foam liner that lies on the skin (a liner glove, the way the knee's foam
//! is in the suit): one piece over each finger bone and one over the back of the hand, 13 in
//! all. The thumb waits for its own round: U2's grip turns its joints about axes set by the
//! stick, not its own, and laps built on those came out absurd (a 40 mm radius at its knuckle).
//!
//! Each piece is built on the hand as it holds the shaft (`upper::shaft_grip`): its inside is the
//! liner's outside (the skin offset by the foam), its wall over that. Where two meet over a joint,
//! the piece nearer the wrist laps over the other, and there both are cylinders about the joint's
//! axis: a surface of revolution turns into itself as the joint turns, so the pieces slide one
//! over the other and can't bind. Each lap reaches far enough either side of the joint to stay
//! closed from the straight hand to the joint's full bend, plus a margin; its radius clears the
//! liner under it at every step.
//!
//! The run builds them, moves the hand from straight to the grip and on to a fist (each joint at
//! its guard, `upper::FINGER_LIMIT`), and at each step checks:
//! - the laps: rays from each joint over its back all meet a piece;
//! - binding: how far any piece runs into another;
//! - the fit on the liner: how far each piece stands off it (empty space) or presses into it.
//!
//! Frame: the foot's (x forward, y left, z up, mm). Each piece is defined where its bone is with
//! the hand straight (the sweep's start, "pose 0") and moves rigidly with its bone.

use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

use cortenforge::cf_design::{Aabb, Solid};
use cortenforge::cf_scan_prep_core::{auto_cap_open_boundaries, clip_mesh_against_plane};
use cortenforge::mesh::io::save_mesh;
use cortenforge::mesh::sdf::{ParitySign, Signed, TriMeshDistance};
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::body::{self, Body};
use crate::hockey_boot::mesh_volume;
use crate::materials::PA12_MJF;
use crate::pad::draw;
use crate::pose::Motion;
use crate::render::{Layer, Style, View};
use crate::round1::mesh_of;
use crate::upper::{self, FINGER_LIMIT, Pose};

/// Round 1's sizes, mm (placeholders where marked).
#[derive(Clone, Copy, Debug)]
pub struct GloveParams {
    /// The liner's foam on the skin (a placeholder: no slash test found, RESEARCH.md 28).
    pub foam: f64,
    /// The hard pieces' wall, PA 12 (as the shin pad's, a placeholder).
    pub wall: f64,
    /// Between two pieces where one slides over the other (as the knee's shells).
    pub clear: f64,
    /// How far each lap still overlaps at the joint's full bend, deg.
    pub margin: f64,
    /// How far down the sides the pieces reach: below the line of the bone's joints.
    pub down: f64,
    /// Between neighbouring fingers' pieces.
    pub side_gap: f64,
    /// The back plate's edge at the wrist: this far toward the knuckles from the wrist joint
    /// (the cuff, soft, covers the wrist).
    pub cuff: f64,
    /// The outer fingers' pieces and the back plate reach this far past the index's and the
    /// little finger's lines.
    pub side_out: f64,
    /// The end pieces reach past the bone's tip by this much.
    pub tip: f64,
    /// Mesh cell.
    pub cell: f64,
    /// How far each lap stays closed: from straight to the joint's bend on the shaft plus this
    /// (deg), or, if None, to its guard (a fist). A lap's height grows steeply with the bend it
    /// has to cover.
    pub grip_plus: Option<f64>,
}

impl Default for GloveParams {
    fn default() -> Self {
        GloveParams { foam: 6.0, wall: 2.0, clear: 1.0, margin: 10.0, down: 6.0, side_gap: 1.0, cuff: 15.0, side_out: 14.0, tip: 2.0, cell: 0.5, grip_plus: Some(15.0) }
    }
}

/// The sweep: 0 straight, 1 the grip on the shaft (where the pieces are built), 2 a fist (each
/// joint at its guard); between, each joint's angle in proportion.
const TS: [f64; 9] = [0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0];
const BUILD: usize = 4;
/// Each lap's surface runs this far (deg) past where the other piece's edge ever goes.
const EXT: f64 = 4.0;

const DIGIT: [&str; 5] = ["thumb", "index", "middle", "ring", "little"];
const BONE: [&str; 3] = ["proximal", "middle", "distal"];
const JOINT: [&str; 3] = ["knuckle", "middle joint", "end joint"];

fn bone_name(k: usize, j: usize) -> String {
    format!("finger{}-{}.R", k + 1, j + 1)
}

/// The pieces: 0 the back plate, then the fingers' (index to little, base to tip).
fn piece_of(k: usize, j: usize) -> usize {
    1 + 3 * (k - 1) + j
}

/// A lap's place in the list of laps (fingers index to little, base to tip).
fn laps_index(k: usize, j: usize) -> usize {
    3 * (k - 1) + j
}

fn joint_name(k: usize, j: usize) -> String {
    format!("{} {}", DIGIT[k], JOINT[j])
}

/// A cylinder about a joint's axis.
#[derive(Clone, Copy, Debug)]
struct Rev {
    c: Point3<f64>,
    w: Vector3<f64>,
    r: f64,
}

impl Rev {
    fn radius(c: &Point3<f64>, w: &Vector3<f64>, q: &Point3<f64>) -> f64 {
        let d = q - c;
        (d - w * d.dot(w)).norm()
    }

    fn eval(&self, q: &Point3<f64>) -> f64 {
        Rev::radius(&self.c, &self.w, q) - self.r
    }
}

/// Where a piece runs over a joint: within `lo..hi` (rad) of the joint's back (`d`, toward the
/// tip `ap`, about the axis `rev.w`), its inside is the lap's cylinder exactly, and it spans the
/// lap's strip across the hand (`x0..x1`, the same for both its pieces).
#[derive(Clone, Copy, Debug)]
struct LapZone {
    rev: Rev,
    d: Vector3<f64>,
    ap: Vector3<f64>,
    lo: f64,
    hi: f64,
    x0: f64,
    x1: f64,
}

impl LapZone {
    fn angle(&self, q: &Point3<f64>) -> f64 {
        let dq = q - self.rev.c;
        dq.dot(&self.ap).atan2(dq.dot(&self.d))
    }
}

/// The back plate's strip over one finger, across the hand (`x0..x1` along the knuckles' axis):
/// its edge there is that finger's knuckle lap (`lap`, and the half-space `p`, `n`), its sides
/// down to `down` below that finger's metacarpal (`side_p`, `side_n`).
#[derive(Clone, Copy, Debug)]
struct Zone {
    x0: f64,
    x1: f64,
    p: Point3<f64>,
    n: Vector3<f64>,
    side_p: Point3<f64>,
    side_n: Vector3<f64>,
    lap: usize,
}

/// Distance to a patch of the skin, signed by the whole hand (closed off at the forearm): a
/// patch is open at its edges, and signing by it alone read points deep in a finger as outside.
type Sdf = Arc<Signed<TriMeshDistance, ParitySign>>;

/// A hard piece, in its pose-0 frame.
#[derive(Clone)]
struct Piece {
    name: String,
    /// The bone it moves with.
    bone: usize,
    /// The skin it is built on (its finger's, or the back of the hand's, in the build pose).
    skin: Sdf,
    /// Pose 0 to the build pose.
    to_build: Motion,
    /// Half-spaces kept: (q - p).n >= 0.
    halves: Vec<(Point3<f64>, Vector3<f64>)>,
    /// A finger piece's strip across the hand (along `e`), away from its laps.
    band: Option<(f64, f64)>,
    /// The back plate's strips (none for a finger's piece).
    zones: Vec<Zone>,
    e: Vector3<f64>,
    /// Where it runs over its joints.
    laps: Vec<LapZone>,
    /// How far down the sides it comes: below the nearest lap's joint centre, along its back.
    down: f64,
    lo: Point3<f64>,
    hi: Point3<f64>,
    foam: f64,
    wall: f64,
}

impl Piece {
    fn zone(&self, q: &Point3<f64>) -> Option<&Zone> {
        if self.zones.is_empty() {
            return None;
        }
        let x = q.coords.dot(&self.e);
        Some(self.zones.iter().find(|z| x >= z.x0 && x < z.x1).unwrap_or(if x < self.zones[0].x0 { &self.zones[0] } else { &self.zones[self.zones.len() - 1] }))
    }

    /// The lap that shapes the piece at `q`: the back plate's strip's, or a finger piece's nearer
    /// joint's.
    fn lap(&self, q: &Point3<f64>) -> Option<&LapZone> {
        match self.zone(q) {
            Some(z) => self.laps.get(z.lap),
            None => self.laps.iter().min_by(|a, b| (q - a.rev.c).norm_squared().partial_cmp(&(q - b.rev.c).norm_squared()).unwrap()),
        }
    }

    /// The laps whose zones `q` is in (the back plate's: only its strip's). On a short bone a
    /// point can be in both its joints' zones; choosing one by the nearer joint flipped partway
    /// along a ray out from a joint and left a band of the piece in neither (round 1, run 8).
    fn in_laps<'a>(&'a self, q: &'a Point3<f64>) -> impl Iterator<Item = &'a LapZone> + 'a {
        let only = self.zone(q).map(|z| z.lap);
        self.laps.iter().enumerate().filter(move |(i, l)| only.is_none_or(|o| o == *i) && (l.lo..=l.hi).contains(&l.angle(q))).map(|(_, l)| l)
    }

    fn cut(&self, q: &Point3<f64>) -> f64 {
        let mut c = f64::MIN;
        for (p, n) in &self.halves {
            c = c.max(-(q - p).dot(n));
        }
        let x = q.coords.dot(&self.e);
        // Across the hand: in a lap's zone its strip (in two, either's: the fingers fan out, and
        // both lie within this piece's own); elsewhere the piece's own.
        let strip = self.in_laps(q).fold(None, |s: Option<(f64, f64)>, l| Some(s.map_or((l.x0, l.x1), |(a, b)| (a.min(l.x0), b.max(l.x1)))));
        if let Some((x0, x1)) = strip.or(self.band) {
            c = c.max(x0 - x).max(x - x1);
        }
        if let Some(z) = self.zone(q) {
            c = c.max(-(q - z.p).dot(&z.n)).max(-(q - z.side_p).dot(&z.side_n));
        } else if let Some(l) = self.lap(q) {
            c = c.max(-((q - l.rev.c).dot(&l.d) + self.down));
        }
        c
    }

    /// The liner side of the piece's inside: negative between it and the skin. In its laps'
    /// zones, their cylinders (united where two meet); elsewhere the liner's outside.
    fn envelope(&self, q: &Point3<f64>) -> f64 {
        let u = self.in_laps(q).map(|l| l.rev.eval(q)).fold(f64::MAX, f64::min);
        if u < f64::MAX { u } else { self.skin.evaluate(Point3::from(self.to_build.apply(q.coords))) - self.foam }
    }

    /// Negative inside, at `q` in the piece's pose-0 frame.
    fn eval(&self, q: Point3<f64>) -> f64 {
        let cut = self.cut(&q);
        if cut > 3.0 {
            return cut;
        }
        let u = self.envelope(&q);
        (-u).max(u - self.wall).max(cut)
    }

    fn inside_box(&self, q: &Point3<f64>) -> bool {
        (0..3).all(|i| q[i] >= self.lo[i] && q[i] <= self.hi[i])
    }
}

/// A lap over a joint: piece `a` (nearer the wrist) over piece `b`. In the pose-0 frame: the
/// joint's centre and axis, its back's direction `d` and `ap` (along the bone, out to the tip),
/// square to the axis. The lap's surfaces reach `beta` back and `alpha` forward of `d` about the
/// joint (rad), plus `EXT`.
struct Lap {
    k: usize,
    j: usize,
    a: usize,
    b: usize,
    c: Point3<f64>,
    w: Vector3<f64>,
    d: Vector3<f64>,
    ap: Vector3<f64>,
    alpha: f64,
    beta: f64,
    /// The under piece's inside radius.
    rho: f64,
    a_bone: usize,
    b_bone: usize,
    /// The back's skin above the joint's centre, straight.
    height: f64,
}

/// The hand at one step of the sweep.
struct HandAt {
    th: [[f64; 3]; 5],
    mo: Vec<Motion>,
    /// Each bone's motion, mm.
    mm: Vec<Motion>,
    /// The smoothed skin, mm.
    skin: Vec<Point3<f64>>,
}

fn hand_at(b: &Body, raw: &[Vector3<f64>], base: &Pose, axes: &[[Vector3<f64>; 3]; 5], th: [[f64; 3]; 5]) -> HandAt {
    let m = &b.model;
    let pose = upper::hand_pose(base, axes, &th);
    let mo = m.rig.motions(raw, &pose);
    let praw = m.rig.skin(raw, &mo);
    let skin = m.s2.apply(&m.s1.apply(&praw[..m.n_body])).iter().map(|&v| body::to_mm(v, b.floor, b.shift)).collect();
    let mm = mo.iter().map(|x| body::motion_to_mm(x, b.floor, b.shift)).collect();
    HandAt { th, mo, mm, skin }
}

/// Triangles from some of the smoothed skin's quads.
fn quad_mesh(pts: &[Point3<f64>], quads: &[[u32; 4]]) -> IndexedMesh {
    let mut map = vec![u32::MAX; pts.len()];
    let mut mesh = IndexedMesh::default();
    for q in quads {
        let ids = q.map(|i| {
            if map[i as usize] == u32::MAX {
                map[i as usize] = mesh.vertices.len() as u32;
                mesh.vertices.push(pts[i as usize]);
            }
            map[i as usize]
        });
        mesh.faces.push([ids[0], ids[1], ids[2]]);
        mesh.faces.push([ids[0], ids[2], ids[3]]);
    }
    mesh
}

/// Per-vertex normals of the smoothed skin over some quads (area-weighted), zero elsewhere.
fn normals(pts: &[Point3<f64>], quads: &[[u32; 4]]) -> Vec<Vector3<f64>> {
    let mut n = vec![Vector3::zeros(); pts.len()];
    for q in quads {
        let [a, b, c, d] = q.map(|i| pts[i as usize]);
        let f = (c - a).cross(&(d - b));
        for &i in q {
            n[i as usize] += f;
        }
    }
    n.iter().map(|v| if v.norm() > 0.0 { v.normalize() } else { *v }).collect()
}

fn moved(m: &IndexedMesh, by: &Motion) -> IndexedMesh {
    IndexedMesh { vertices: m.vertices.iter().map(|v| Point3::from(by.apply(v.coords))).collect(), faces: m.faces.clone() }
}

fn bbox(pts: &[Point3<f64>]) -> (Point3<f64>, Point3<f64>) {
    let lo = pts.iter().fold(Vector3::repeat(f64::MAX), |a, v| a.inf(&v.coords));
    let hi = pts.iter().fold(Vector3::repeat(f64::MIN), |a, v| a.sup(&v.coords));
    (Point3::from(lo), Point3::from(hi))
}

fn par_map<T: Sync, R: Send>(items: &[T], threads: usize, f: impl Fn(&T) -> R + Sync) -> Vec<R> {
    let chunk = items.len().div_ceil(threads.max(1)).max(1);
    std::thread::scope(|sc| {
        let hs: Vec<_> = items.chunks(chunk).map(|c| sc.spawn(|| c.iter().map(&f).collect::<Vec<R>>())).collect();
        hs.into_iter().flat_map(|h| h.join().unwrap()).collect()
    })
}

/// A mesh's section by a plane, as strips `width` mm wide lying in the plane (to draw seen along
/// its normal: a cut mesh's walls lie along the view, edge-on, and draw as nothing).
fn section_lines(mesh: &IndexedMesh, p: Point3<f64>, n: Vector3<f64>, width: f64) -> IndexedMesh {
    let n = n.normalize();
    let mut out = IndexedMesh::default();
    for c in crate::scan::section(mesh, p, n).contours {
        let pts = &c.points;
        if pts.len() < 2 {
            continue;
        }
        let closed = (pts[0] - pts[pts.len() - 1]).norm() < 1.0;
        let segs = pts.len() - 1 + usize::from(closed && pts.len() > 2);
        for s in 0..segs {
            let (a, b) = (pts[s], pts[(s + 1) % pts.len()]);
            let t = b - a;
            if t.norm() < 1e-9 {
                continue;
            }
            let o = t.normalize().cross(&n) * (0.5 * width);
            let k = out.vertices.len() as u32;
            out.vertices.extend([a - o, a + o, b + o, b - o]);
            out.faces.extend([[k, k + 1, k + 2], [k, k + 2, k + 3]]);
        }
    }
    out
}

/// How a piece sits on the liner at one step: how far it stands off each point under it (within
/// 12 mm along the skin's normal), and the most it presses in, and where.
#[derive(Clone, Default)]
struct Sit {
    stands: Vec<f64>,
    press_max: f64,
    press_at: Option<Point3<f64>>,
}

/// `cargo run --release -- glove <scan> [out]`: glove round 1 (docs/glove/).
/// With `fist`, the laps stay closed to each joint's guard (a fist) and the results go to
/// docs/glove/fist_laps/.
pub fn run(scan: &Path, out: &Path, fist: bool) -> Result<(), String> {
    let t0 = Instant::now();
    let p = GloveParams { grip_plus: if fist { None } else { GloveParams::default().grip_plus }, ..GloveParams::default() };
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let docs_dir = if fist { Path::new("docs/glove/fist_laps") } else { Path::new("docs/glove") };
    let docs = docs_dir;
    std::fs::create_dir_all(docs).map_err(|e| e.to_string())?;
    let threads = std::env::var("PAD_THREADS").ok().and_then(|v| v.parse::<usize>().ok()).unwrap_or(2).max(1);
    let b = body::build(scan, 1250.0)?;
    let (u, _) = upper::fit(&b)?;
    let raw = &u.raw;
    let (base, grip) = upper::shaft_grip(&b, raw)?;
    let m = &b.model;
    let rig = &m.rig;
    let quads = upper::hand_quads(&b, raw);
    println!("  body fitted, hand on the shaft ({:.0} s)", t0.elapsed().as_secs_f64());

    // The sweep: every joint from straight to the grip, then on to its guard (the thumb too, as
    // U2 moves it, though it carries no pieces).
    let lim = |k: usize, j: usize| if k == 0 { upper::THUMB_LIMIT[j] } else { FINGER_LIMIT[j] };
    let th_at = |t: f64| {
        let mut th = [[0.0f64; 3]; 5];
        for k in 0..5 {
            for j in 0..3 {
                let g = grip.angles[k][j];
                th[k][j] = if t <= 1.0 { t * g } else { g + (t - 1.0) * (lim(k, j) - g) };
            }
        }
        th
    };
    // How far each lap stays closed (deg).
    let closed_to = |k: usize, j: usize| match p.grip_plus {
        Some(g) => (grip.angles[k][j] + g).min(lim(k, j)),
        None => lim(k, j),
    };
    let hands: Vec<HandAt> = TS.iter().map(|&t| hand_at(&b, raw, &base, &grip.axes, th_at(t))).collect();
    let h0 = &hands[0];
    let hb = &hands[BUILD];
    let rel = |h: &HandAt, bone: usize| h.mm[bone].after(&h0.mm[bone].inverse());
    let nrms: Vec<Vec<Vector3<f64>>> = hands.iter().map(|h| normals(&h.skin, &quads)).collect();

    // Pose 0's bones.
    let bi = |n: &str| rig.index[n];
    let head0 = |n: &str| upper::posed_end(&b, raw, &h0.mo, n, false);
    let tail0 = |n: &str| upper::posed_end(&b, raw, &h0.mo, n, true);
    let dir0 = |n: &str| (tail0(n) - head0(n)).normalize();
    let from_mh = |v: Vector3<f64>| Vector3::new(v.z, v.x, v.y);
    let axis0 = |k: usize, j: usize| {
        let i = bi(&bone_name(k, j));
        let par = rig.bones[i].parent.unwrap_or(0);
        from_mh(h0.mo[par].r * grip.axes[k][j]).normalize()
    };
    let e = axis0(2, 0);
    let skew = (1..5).flat_map(|k| (0..3).map(move |j| (k, j))).map(|(k, j)| axis0(k, j).dot(&e).abs().min(1.0).acos().to_degrees()).fold(0.0, f64::max);
    // Each piece's strip across the hand (along e, which every finger joint turns about, so a
    // point's place along it never changes): its bone's line (it runs across a little, the
    // fingers fan out) widened toward each neighbour finger to halfway to the nearest of that
    // finger's bones at any level, less half the gap; an outer finger's outer side by `side_out`.
    // So no piece ever overlaps another finger's, at any level, and a lap's two pieces both cover
    // the joint's line either side. y = s x grows from the index to the little finger.
    let s = (head0(&bone_name(4, 0)).coords.dot(&e) - head0(&bone_name(1, 0)).coords.dot(&e)).signum();
    let yr = |k: usize, j: usize| {
        let n = bone_name(k, j);
        let (a, b) = (s * head0(&n).coords.dot(&e), s * tail0(&n).coords.dot(&e));
        (a.min(b), a.max(b))
    };
    let ymin = |k: usize| (0..3).map(|j| yr(k, j).0).fold(f64::MAX, f64::min);
    let ymax = |k: usize| (0..3).map(|j| yr(k, j).1).fold(f64::MIN, f64::max);
    let band_at = |k: usize, j: usize| {
        let (y0, y1) = yr(k, j);
        let lo = if k > 1 { 0.5 * (y0 + ymax(k - 1)) + 0.5 * p.side_gap } else { y0 - p.side_out };
        let hi = if k < 4 { 0.5 * (y1 + ymin(k + 1)) - 0.5 * p.side_gap } else { y1 + p.side_out };
        ((s * lo).min(s * hi), (s * lo).max(s * hi))
    };
    // The back plate's strips: split halfway between neighbouring fingers.
    let bnd = |k: usize| s * 0.5 * (ymax(k) + ymin(k + 1));
    // A lap's: where both its pieces' strips are (at the knuckle, the proximal piece's, which lies
    // within the back plate's).
    let lap_band = |k: usize, j: usize| {
        let (a, b) = (band_at(k, j.saturating_sub(1)), band_at(k, j));
        (a.0.max(b.0), a.1.min(b.1))
    };
    let fan: Vec<f64> = (1..5).map(|k| s * (tail0(&bone_name(k, 2)) - head0(&bone_name(k, 0))).dot(&e)).collect();

    // The skin: the back of the hand's (with the fingers' first bones) and each finger's (with its
    // metacarpal), each a patch of the smoothed skin's quads owned by those bones.
    let dom = upper::dominant(&b);
    let owned = |bones: &[String]| -> Vec<f64> {
        let ids: Vec<usize> = bones.iter().map(|n| bi(n)).collect();
        let vals: Vec<Vector3<f64>> = (0..m.n_body).map(|i| if ids.contains(&dom[i]) { Vector3::x() } else { Vector3::zeros() }).collect();
        m.s2.apply(&m.s1.apply(&vals)).iter().map(|v| v.x).collect()
    };
    let patch = |bones: &[String]| -> Vec<[u32; 4]> {
        let sm = owned(bones);
        quads.iter().copied().filter(|q| q.iter().all(|&i| sm[i as usize] > 0.5)).collect()
    };
    let hand_verts: Vec<usize> = {
        let mut v: Vec<usize> = quads.iter().flatten().map(|&i| i as usize).collect();
        v.sort_unstable();
        v.dedup();
        v
    };
    // The skin's vertices owned by some bones (smoothed: a vertex between two counts for the one
    // with more of it).
    let bone_verts = |bones: &[String]| -> Vec<usize> {
        let sm = owned(bones);
        hand_verts.iter().copied().filter(|&i| sm[i] > 0.5).collect()
    };
    let plate_bones: Vec<String> = (1..=4).map(|i| format!("metacarpal{i}.R")).chain(["wrist.R".to_string()]).chain((1..5).map(|k| bone_name(k, 0))).collect();
    let finger_bones = |k: usize| -> Vec<String> { (0..3).map(|j| bone_name(k, j)).chain([format!("metacarpal{k}.R")]).collect() };
    let mut patches = vec![patch(&plate_bones)];
    patches.extend((1..5).map(|k| patch(&finger_bones(k))));
    let mut closed_hand = quad_mesh(&hb.skin, &quads);
    auto_cap_open_boundaries(&mut closed_hand);
    let sdfs: Vec<Sdf> = patches
        .iter()
        .map(|q| {
            let dist = TriMeshDistance::new(quad_mesh(&hb.skin, q)).map_err(|e| format!("skin distance: {e}"))?;
            let sign = ParitySign::new(&closed_hand).map_err(|e| format!("hand inside test: {e}"))?;
            Ok(Arc::new(Signed { distance: dist, sign }))
        })
        .collect::<Result<_, String>>()?;
    // The skin's sign: outside along the back of the middle finger's middle bone, inside at the
    // bone.
    {
        let n = bone_name(2, 1);
        let to_b = rel(hb, bi(&n));
        let mid_b = Point3::from(to_b.apply((head0(&n).coords + tail0(&n).coords) * 0.5));
        let d_b = to_b.r * dir0(&n).cross(&axis0(2, 1)).normalize();
        let (inside, outside) = (sdfs[2].evaluate(mid_b), sdfs[2].evaluate(mid_b + d_b * 20.0));
        if !(inside < 0.0 && outside > 0.0) {
            return Err(format!("the skin's signed distance is the wrong way round: {inside:.1} at the bone, {outside:.1} 20 mm over it"));
        }
    }

    // The laps: each finger joint's, the span split evenly either side.
    let mut laps: Vec<Lap> = Vec::new();
    for k in 1..5 {
        for j in 0..3 {
            let bn = bone_name(k, j);
            let c = head0(&bn);
            let w = axis0(k, j);
            let a_name = if j == 0 { format!("metacarpal{k}.R") } else { bone_name(k, j - 1) };
            let d = dir0(&a_name).cross(&w).normalize();
            let ap = w.cross(&d);
            let half = 0.5 * (closed_to(k, j) + p.margin).to_radians();
            let a = if j == 0 { 0 } else { piece_of(k, j - 1) };
            // The back's skin above the joint, straight: the highest skin within 2 mm of the joint
            // along the bone and 4 mm across.
            let height = bone_verts(&finger_bones(k))
                .iter()
                .map(|&i| h0.skin[i] - c)
                .filter(|q| q.dot(&ap).abs() < 2.0 && q.dot(&w).abs() < 4.0)
                .map(|q| q.dot(&d))
                .fold(f64::MIN, f64::max);
            laps.push(Lap { k, j, a, b: piece_of(k, j), c, w, d, ap, alpha: half, beta: half, rho: 0.0, a_bone: bi(&a_name), b_bone: bi(&bn), height });
        }
    }
    // Each lap's radius: clear of the liner over its two bones' skin under it, at every step: the
    // liner's outside (the skin out along its normal by the foam), seen from each bone. Only the
    // two bones' own skin: seen from a bone, skin two joints away swings round over its back as
    // the finger bends.
    for lap in laps.iter_mut() {
        let own = bone_verts(&[rig.bones[lap.a_bone].name.clone(), rig.bones[lap.b_bone].name.clone()]);
        let (x0, x1) = lap_band(lap.k, lap.j);
        let mut rmax = 0.0f64;
        for (ti, h) in hands.iter().enumerate() {
            for bone in [lap.a_bone, lap.b_bone] {
                let back = rel(h, bone).inverse();
                for &i in &own {
                    let q = Point3::from(back.apply((h.skin[i] + nrms[ti][i] * p.foam).coords));
                    let dq = q - lap.c;
                    let (dv, du) = (dq.dot(&lap.d), dq.dot(&lap.ap));
                    let psi = du.atan2(dv);
                    let x = q.coords.dot(&e);
                    if psi < -lap.beta - EXT.to_radians() || psi > lap.alpha + EXT.to_radians() || dv < -p.down || x < x0 || x > x1 {
                        continue;
                    }
                    rmax = rmax.max(Rev::radius(&lap.c, &lap.w, &q));
                }
            }
        }
        lap.rho = rmax;
    }
    // Nest them like a telescope: each joint's lap at least a wall and the clearance (and 1 mm,
    // at the knuckle 3) wider than the next joint's toward the tip. On a short bone the lap behind and the lap in
    // front of it overlap over it (round 1: the index's and the little finger's middle bones,
    // with the hand straight), and three pieces stacked there only fit nested.
    let mut raised: Vec<(usize, f64)> = Vec::new();
    for k in 1..5 {
        for j in (0..2).rev() {
            let next = laps.iter().find(|l| l.k == k && l.j == j + 1).map(|l| l.rho).unwrap_or(0.0);
            let l = laps.iter_mut().find(|l| l.k == k && l.j == j).unwrap();
            // The knuckle by 3 mm: with the hand straight, the back plate's front edge met the next lap's
            // back end on the index's first piece at 1 mm (round 1, run 11).
            let need = next + p.wall + p.clear + if j == 0 { 3.0 } else { 1.0 };
            if l.rho < need {
                raised.push((laps_index(k, j), need - l.rho));
                l.rho = need;
            }
        }
    }

    // The pieces.
    let over = p.wall + p.clear;
    let front = |l: &Lap| (l.c, l.d * l.alpha.sin() - l.ap * l.alpha.cos());
    let back = |l: &Lap| (l.c, l.d * l.beta.sin() + l.ap * l.beta.cos());
    let lap_at = |k: usize, j: usize| laps.iter().find(|l| l.k == k && l.j == j).unwrap();
    let zone_of = |l: &Lap, r: f64| {
        let (x0, x1) = lap_band(l.k, l.j);
        LapZone { rev: Rev { c: l.c, w: l.w, r }, d: l.d, ap: l.ap, lo: -l.beta - EXT.to_radians(), hi: l.alpha + EXT.to_radians(), x0, x1 }
    };
    let mut pieces: Vec<Piece> = Vec::new();
    {
        // The back plate: per finger, its edge that finger's knuckle lap and its sides down to
        // `down` below that finger's metacarpal; at the wrist, short of the cuff.
        let h = upper::hand_frame(&b, raw, &h0.mo);
        let halves = vec![
            (h.wr + h.axis * p.cuff, h.axis),
            (Point3::from(e * (s * (yr(1, 0).0 - p.side_out))), e * s),
            (Point3::from(e * (s * (yr(4, 0).1 + p.side_out))), -e * s),
        ];
        let mut zones = Vec::new();
        let mut lz = Vec::new();
        for k in 1..5 {
            let l = lap_at(k, 0);
            let lo = if k > 1 { bnd(k - 1) } else { s * (yr(1, 0).0 - 1e4) };
            let hi = if k < 4 { bnd(k) } else { s * (yr(4, 0).1 + 1e4) };
            let (pp, n) = front(l);
            zones.push(Zone { x0: lo.min(hi), x1: lo.max(hi), p: pp, n, side_p: l.c - l.d * p.down, side_n: l.d, lap: k - 1 });
            lz.push(zone_of(l, l.rho + over));
        }
        zones.sort_by(|a, b| a.x0.partial_cmp(&b.x0).unwrap());
        let bone = bi("metacarpal2.R");
        let (lo, hi) = bbox(&bone_verts(&plate_bones).iter().map(|&i| h0.skin[i]).collect::<Vec<_>>());
        let mg = Vector3::repeat(30.0);
        pieces.push(Piece { name: "back of the hand".into(), bone, skin: sdfs[0].clone(), to_build: rel(hb, bone), halves, band: None, zones, e, laps: lz, down: p.down, lo: lo - mg, hi: hi + mg, foam: p.foam, wall: p.wall });
    }
    for k in 1..5 {
        for j in 0..3 {
            let bn = bone_name(k, j);
            let bone = bi(&bn);
            let (h, t) = (head0(&bn), tail0(&bn));
            let a = (t - h).normalize();
            let (lo_x, hi_x) = band_at(k, j);
            let mut halves = Vec::new();
            // Under at the joint nearer the wrist, over at the one nearer the tip.
            let l = lap_at(k, j);
            halves.push(back(l));
            let mut lz = vec![zone_of(l, l.rho)];
            if j < 2 {
                let l = lap_at(k, j + 1);
                halves.push(front(l));
                lz.push(zone_of(l, l.rho + over));
            } else {
                halves.push((t + a * p.tip, -a));
            }
            let mg = Vector3::repeat(30.0);
            let (lo, hi) = bbox(&[h, t]);
            pieces.push(Piece {
                name: format!("{} {}", DIGIT[k], BONE[j]),
                bone,
                skin: sdfs[k].clone(),
                to_build: rel(hb, bone),
                halves,
                band: Some((lo_x, hi_x)),
                zones: Vec::new(),
                e,
                laps: lz,
                down: p.down,
                lo: lo - mg,
                hi: hi + mg,
                foam: p.foam,
                wall: p.wall,
            });
        }
    }
    let n_p = pieces.len();
    // The skin each piece sits over: its own bone's and, where it laps, its neighbours'.
    let piece_verts: Vec<Vec<usize>> = (0..n_p)
        .map(|i| {
            if i == 0 {
                return bone_verts(&plate_bones);
            }
            let (k, j) = (1 + (i - 1) / 3, (i - 1) % 3);
            let mut v = vec![bone_name(k, j), if j > 0 { bone_name(k, j - 1) } else { format!("metacarpal{k}.R") }];
            if j < 2 {
                v.push(bone_name(k, j + 1));
            }
            bone_verts(&v)
        })
        .collect();

    // Mesh them (pose-0 frame).
    let t1 = Instant::now();
    let idx: Vec<usize> = (0..n_p).collect();
    let meshes: Vec<IndexedMesh> = par_map(&idx, threads, |&i| {
        let pc = pieces[i].clone();
        let (lo, hi) = (pc.lo, pc.hi);
        mesh_of(&Solid::user_fn(move |q| pc.eval(q), Aabb::new(lo, hi)), p.cell)
    });
    let grams: Vec<f64> = meshes.iter().map(|mm| mesh_volume(mm).abs() / 1000.0 * PA12_MJF.density / 1000.0).collect();
    println!("  meshed {n_p} pieces ({:.0} s)", t1.elapsed().as_secs_f64());
    for (i, mm) in meshes.iter().enumerate() {
        if mm.faces.is_empty() {
            return Err(format!("{}: empty", pieces[i].name));
        }
    }

    // Each piece's motion at each step, and its mesh there.
    let rels: Vec<Vec<Motion>> = hands.iter().map(|h| pieces.iter().map(|pc| rel(h, pc.bone)).collect()).collect();
    let posed: Vec<Vec<IndexedMesh>> = rels.iter().map(|rs| meshes.iter().zip(rs).map(|(mm, r)| moved(mm, r)).collect()).collect();
    let boxes: Vec<Vec<(Point3<f64>, Point3<f64>)>> = posed.iter().map(|ms| ms.iter().map(|mm| bbox(&mm.vertices)).collect()).collect();
    // Where a point is at a step: the nearest joint centre.
    let where_at = |ti: usize, q: &Point3<f64>| {
        let (name, d) = laps.iter().map(|l| (joint_name(l.k, l.j), (Point3::from(rels[ti][l.a].apply(l.c.coords)) - q).norm())).min_by(|a, b| a.1.partial_cmp(&b.1).unwrap()).unwrap();
        format!("{d:.0} mm from the {name}'s centre")
    };

    // Check 1: the laps stay closed. From each joint's centre (and 5 mm either side along its
    // axis), rays over its back, from where the under piece's edge is with the hand straight to
    // where the over piece's edge is at this step; each must meet a piece within 40 mm.
    let t2 = Instant::now();
    let debug = std::env::var("GLOVE_DEBUG").is_ok();
    let gaps: Vec<Vec<f64>> = par_map(&laps, threads, |l| {
        hands
            .iter()
            .enumerate()
            .map(|(ti, h)| {
                let theta = h.th[l.k][l.j].to_radians();
                let a2b = rels[ti][l.b].inverse().after(&rels[ti][l.a]);
                let (pa, pb) = (&pieces[l.a], &pieces[l.b]);
                let step = 2.0f64.to_radians();
                let n = ((l.alpha + l.beta + theta) / step).ceil() as usize;
                let mut worst = 0.0f64;
                for dx in [-5.0, 0.0, 5.0] {
                    let o = l.c + l.w * dx;
                    let (mut run, mut best) = (0usize, 0usize);
                    for i in 0..=n {
                        let psi = -l.beta + i as f64 * step;
                        let dir = l.d * psi.cos() + l.ap * psi.sin();
                        let hit = (0..=80).any(|r| {
                            let q = o + dir * (0.5 * r as f64);
                            pa.eval(q) < 0.0 || pb.eval(Point3::from(a2b.apply(q.coords))) < 0.0
                        });
                        if !hit && debug && ti == 2 {
                            let probe = |r: f64| {
                                let q = o + dir * r;
                                let qb = Point3::from(a2b.apply(q.coords));
                                format!("r {r:.1}: A cut {:.2} env {:.2} | B cut {:.2} env {:.2}", pa.cut(&q), pa.envelope(&q), pb.cut(&qb), pb.envelope(&qb))
                            };
                            println!("  gap {} step {:.2} dx {dx:+.0} psi {:.0}: {} ; {}", joint_name(l.k, l.j), TS[ti], psi.to_degrees(), probe(l.rho + 0.5 * p.wall), probe(l.rho + over + 0.5 * p.wall));
                        }
                        if hit {
                            best = best.max(run);
                            run = 0;
                        } else {
                            run += 1;
                        }
                    }
                    worst = worst.max(best.max(run) as f64 * 2.0);
                }
                worst
            })
            .collect()
    });
    println!("  laps checked ({:.0} s)", t2.elapsed().as_secs_f64());

    // Check 2: binding. Every piece's surface against every other piece it comes near, at every
    // step: how far inside the other it goes (or how close it stays), and where.
    let t3 = Instant::now();
    let near = |ti: usize, i: usize, j: usize| {
        let (a, b) = (boxes[ti][i], boxes[ti][j]);
        (0..3).all(|c| a.0[c] <= b.1[c] + 1.0 && b.0[c] <= a.1[c] + 1.0)
    };
    let jobs: Vec<(usize, usize, usize)> =
        (0..hands.len()).flat_map(|ti| (0..n_p).flat_map(move |i| (0..n_p).filter(move |&j| j != i).map(move |j| (ti, i, j)))).filter(|&(ti, i, j)| near(ti, i, j)).collect();
    let closest: Vec<(f64, Point3<f64>, usize)> = par_map(&jobs, threads, |&(ti, i, j)| {
        let to_j = rels[ti][j].inverse().after(&rels[ti][i]);
        meshes[i]
            .vertices
            .iter()
            .step_by(2)
            .map(|v| Point3::from(to_j.apply(v.coords)))
            .filter(|q| pieces[j].inside_box(q))
            .map(|q| (pieces[j].eval(q), q))
            .fold((f64::MAX, Point3::origin(), 0usize), |a, b| (if b.0 < a.0 { b.0 } else { a.0 }, if b.0 < a.0 { b.1 } else { a.1 }, a.2 + usize::from(b.0 < -0.3)))
    });
    println!("  binding checked: {} pairs near each other over the sweep ({:.0} s)", jobs.len(), t3.elapsed().as_secs_f64());

    // Check 3: the fit on the liner. From each skin point under a piece, out along the skin's
    // normal from the liner's outside (the skin + the foam): how far to the piece (empty space),
    // or how far the piece's inside already is past it (pressing into the liner).
    let t4 = Instant::now();
    let mut owner: Vec<Vec<usize>> = vec![Vec::new(); hb.skin.len()];
    for (i, vs) in piece_verts.iter().enumerate() {
        for &v in vs {
            owner[v].push(i);
        }
    }
    let skin_verts: Vec<usize> = (0..owner.len()).filter(|&v| !owner[v].is_empty()).collect();
    let sits: Vec<Vec<Sit>> = hands
        .iter()
        .enumerate()
        .map(|(ti, h)| {
            let per: Vec<Option<(usize, f64)>> = par_map(&skin_verts, threads, |&v| {
                let n = nrms[ti][v];
                if n.norm() == 0.0 {
                    return None;
                }
                let liner = h.skin[v] + n * p.foam;
                let mut best: Option<(usize, f64)> = None;
                for &i in &owner[v] {
                    let (lo, hi) = boxes[ti][i];
                    if (0..3).any(|c| liner[c] < lo[c] - 20.0 || liner[c] > hi[c] + 20.0) {
                        continue;
                    }
                    let back = rels[ti][i].inverse();
                    let (q, nq) = (Point3::from(back.apply(liner.coords)), back.r * n);
                    let pc = &pieces[i];
                    let val = if pc.eval(q) < 0.0 {
                        // Pressing: back toward the skin until out of the wall.
                        let r = (1..=((p.foam / 0.25) as usize)).map(|s| s as f64 * 0.25).find(|&r| pc.eval(q - nq * r) >= 0.0).unwrap_or(p.foam);
                        Some(-r)
                    } else {
                        (1..=24).map(|s| s as f64 * 0.5).find(|&r| pc.eval(q + nq * r) < 0.0).map(|r| {
                            let (mut a, mut c) = (r - 0.5, r);
                            for _ in 0..5 {
                                let mid = 0.5 * (a + c);
                                if pc.eval(q + nq * mid) < 0.0 { c = mid } else { a = mid }
                            }
                            0.5 * (a + c)
                        })
                    };
                    if let Some(x) = val {
                        if best.is_none_or(|(_, y)| x < y) {
                            best = Some((i, x));
                        }
                    }
                }
                best
            });
            let mut out = vec![Sit::default(); n_p];
            for (&v, r) in skin_verts.iter().zip(per) {
                let Some((i, x)) = r else { continue };
                let s = &mut out[i];
                s.stands.push(x.max(0.0));
                if -x > s.press_max {
                    s.press_max = -x;
                    s.press_at = Some(h.skin[v]);
                }
            }
            out
        })
        .collect();
    println!("  fit on the liner checked ({:.0} s)", t4.elapsed().as_secs_f64());

    // Report.
    let mut lines = Vec::new();
    let mut say = |s: String| {
        println!("{s}");
        lines.push(s);
    };
    say(format!("glove round 1 (track U2): the hard pieces on the right hand closed on the shaft ({})", scan.display()));
    say(format!(
        "  liner foam {:.0} mm, wall {:.0} mm (PA 12), {:.0} mm between sliding pieces, laps {:.0} deg past the bend each covers ({}), down the sides {:.0} mm below the joints' line (placeholders)",
        p.foam,
        p.wall,
        p.clear,
        p.margin,
        match p.grip_plus {
            Some(g) => format!("each joint's bend on the shaft + {g:.0} deg: beyond, in a fist, the laps open over the liner"),
            None => "each joint's full bend, to a fist".into(),
        },
        p.down
    ));
    say(format!(
        "  the fingers' joints all turn about one direction across the hand (within {skew:.1} deg), so pieces on neighbouring fingers, kept {:.0} mm apart across it, never meet; the fingers fan out along it (knuckle to tip, toward the little finger) {}",
        p.side_gap,
        (1..5).map(|k| format!("{} {:+.1} mm", DIGIT[k], fan[k - 1])).collect::<Vec<_>>().join(", ")
    ));
    say("  the sweep: straight (0), the grip (1, where the pieces are built), a fist (2: every joint at its guard)".into());
    say(format!("  {n_p} hard pieces, {:.1} g together (PA 12, {:.2} g/cm3):", grams.iter().sum::<f64>(), PA12_MJF.density / 1000.0));
    for (i, pc) in pieces.iter().enumerate() {
        say(format!("    {:<22} {:>5.1} g", pc.name, grams[i]));
    }
    say("  the laps (over piece / under piece, cylinders about each joint's axis): the under piece's inside radius (clear of the liner), the back's skin over the joint's centre, how far the lap stands proud of the pieces either side, its reach either side of the joint and the bend it stays closed to:".into());
    for l in &laps {
        let body_top = l.height + p.foam + p.wall;
        let lap_top = l.rho + 2.0 * p.wall + p.clear;
        say(format!(
            "    {:<20} {:<17} over {:<17}: radius {:>4.1} mm, skin {:.1} mm over the centre; proud by {:>4.1} mm; {:.0} deg either side of the joint, closed to {:.0} deg (on the shaft {:.0}, guard {:.0}){}",
            joint_name(l.k, l.j),
            pieces[l.a].name,
            pieces[l.b].name,
            l.rho,
            l.height,
            lap_top - body_top,
            l.alpha.to_degrees(),
            (l.alpha + l.beta).to_degrees() - p.margin,
            grip.angles[l.k][l.j],
            lim(l.k, l.j),
            raised.iter().find(|r| r.0 == laps_index(l.k, l.j)).map_or(String::new(), |r| format!("; radius raised {:.1} mm to nest over the next lap", r.1))
        ));
    }
    say("  check 1, laps closed: the widest gap seen over each joint's back (deg), at each step of the sweep:".into());
    say(format!("    {:<20} {}", "", TS.iter().map(|t| format!("{t:>5.2}")).collect::<Vec<_>>().join("")));
    for (l, g) in laps.iter().zip(&gaps) {
        say(format!("    {:<20} {}", joint_name(l.k, l.j), g.iter().map(|x| format!("{x:>5.0}")).collect::<Vec<_>>().join("")));
    }
    say("  check 2, binding: at each step, the deepest any piece runs into another (mm), or how close the closest two come".into());
    let mut worst_all = (f64::MAX, 0usize, 0usize, 0usize);
    for ti in 0..hands.len() {
        let (mut wv, mut wi, mut wj, mut wq, mut wn) = (f64::MAX, 0, 0, Point3::origin(), 0usize);
        for (jb, &(t, i, j)) in jobs.iter().enumerate() {
            if t == ti && closest[jb].0 < wv {
                (wv, wi, wj, wq, wn) = (closest[jb].0, i, j, Point3::from(rels[ti][j].apply(closest[jb].1.coords)), closest[jb].2);
            }
        }
        if wv < worst_all.0 {
            worst_all = (wv, ti, wi, wj);
        }
        say(format!(
            "    step {:.2}: {}",
            TS[ti],
            if wv < 0.0 {
                format!("{:.1} mm ({wn} of its surface points over 0.3 mm in), {} into {}, {}", -wv, pieces[wi].name, pieces[wj].name, where_at(ti, &wq))
            } else {
                format!("none; closest {:.1} mm ({} by {})", wv, pieces[wi].name, pieces[wj].name)
            }
        ));
    }
    say("  check 3, the fit on the liner: how far each piece stands off the liner (empty space) at each step, mean / 90th percentile over the skin points under it, and the most it presses into it (mm):".into());
    let stats = |v: &[f64]| {
        if v.is_empty() {
            return (0.0, 0.0);
        }
        let mut s = v.to_vec();
        s.sort_by(|a, b| a.partial_cmp(b).unwrap());
        (s.iter().sum::<f64>() / s.len() as f64, s[(0.9 * (s.len() - 1) as f64) as usize])
    };
    say(format!("    {:<22} {:>5} {}", "", "points", TS.iter().map(|t| format!("{:>14}", format!("{t:.2}"))).collect::<Vec<_>>().join("")));
    for (i, pc) in pieces.iter().enumerate() {
        let cells: Vec<String> = sits
            .iter()
            .map(|s| {
                let (mean, p90) = stats(&s[i].stands);
                format!("{:>14}", format!("{mean:.1}/{p90:.1} +{:.1}", s[i].press_max))
            })
            .collect();
        say(format!("    {:<22} {:>5} {}", pc.name, sits[BUILD][i].stands.len(), cells.join("")));
    }
    say("    (each step: mean / 90th percentile standing off, + the most pressing in)".into());
    for (i, pc) in pieces.iter().enumerate() {
        let (ti, sp) = sits.iter().enumerate().map(|(ti, s)| (ti, &s[i])).max_by(|a, b| a.1.press_max.partial_cmp(&b.1.press_max).unwrap()).unwrap();
        if let (true, Some(q)) = (sp.press_max >= 1.0, sp.press_at) {
            say(format!("    {} presses in most ({:.1} mm) at step {:.2}, {}", pc.name, sp.press_max, TS[ti], where_at(ti, &q)));
        }
    }
    std::fs::write(docs.join("summary.txt"), lines.join("\n") + "\n").map_err(|e| e.to_string())?;

    // The pieces as built.
    for (i, pc) in pieces.iter().enumerate() {
        save_mesh(&posed[BUILD][i], out.join(format!("{i:02}_{}.stl", pc.name.replace(' ', "_")))).map_err(|e| e.to_string())?;
    }

    // Pictures.
    let skin_c = [0.85, 0.70, 0.60];
    let navy = [0.16, 0.20, 0.40];
    let blue = [0.32, 0.42, 0.68];
    let grey = [0.55, 0.55, 0.6];
    let colour = |i: usize| if i == 0 || (i - 1) % 2 == 1 { navy } else { blue };
    let h = upper::hand_frame(&b, raw, &h0.mo);
    let wrist_cut = |mm: &IndexedMesh| clip_mesh_against_plane(mm, h.wr - h.axis * 40.0, h.axis);
    let views = [View::looking(h.palm, h.axis), View::looking(-h.across, h.axis), View::looking(h.across, h.axis)];
    let shaft = upper::bar_mesh(&grip.bar, 140.0);
    for (ti, name) in [(0usize, "glove_straight.png"), (BUILD, "glove.png"), (TS.len() - 1, "glove_fist.png")] {
        let hand = wrist_cut(&quad_mesh(&hands[ti].skin, &quads));
        let mut layers = vec![Layer::solid(&hand, skin_c)];
        for (i, mm) in posed[ti].iter().enumerate() {
            layers.push(Layer::solid(mm, colour(i)));
        }
        if ti == BUILD {
            layers.push(Layer::solid(&shaft, grey));
        }
        draw(&layers, &views, 2.0, &docs.join(name), Style::pretty())?;
    }
    // The middle finger cut down its middle bone's line: straight, gripping, a fist.
    {
        let cut_p = Point3::from(e * (0.5 * (head0(&bone_name(2, 1)) + tail0(&bone_name(2, 1)).coords).coords.dot(&e)));
        let mut keep: Vec<IndexedMesh> = Vec::new();
        let mut cols: Vec<[f64; 3]> = Vec::new();
        for (slot, ti) in [0usize, BUILD, TS.len() - 1].into_iter().enumerate() {
            let shift = Motion { r: nalgebra::Rotation3::identity(), t: h.axis * (230.0 * slot as f64) };
            keep.push(moved(&section_lines(&wrist_cut(&quad_mesh(&hands[ti].skin, &quads)), cut_p, e, 0.8), &shift));
            cols.push([0.6, 0.4, 0.3]);
            for i in [0usize, piece_of(2, 0), piece_of(2, 1), piece_of(2, 2)] {
                keep.push(moved(&section_lines(&posed[ti][i], cut_p, e, 0.8), &shift));
                cols.push(colour(i));
            }
        }
        let layers: Vec<Layer> = keep.iter().zip(&cols).map(|(mm, c)| Layer::solid(mm, *c)).collect();
        draw(&layers, &[View::looking(e, -h.palm)], 2.5, &docs.join("glove_section.png"), Style::pretty())?;
    }
    if worst_all.0 < -0.3 {
        println!("  NOTE: pieces bind: {:.1} mm, {} into {} at step {:.2}", -worst_all.0, pieces[worst_all.2].name, pieces[worst_all.3].name, TS[worst_all.1]);
    }
    println!("drew {}/glove.png, glove_straight.png, glove_fist.png, glove_section.png; pieces in {} ({:.0} s)", docs.display(), out.display(), t0.elapsed().as_secs_f64());
    Ok(())
}
