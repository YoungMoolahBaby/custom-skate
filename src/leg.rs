//! Track D1: the stand-in leg, for the shin pads (docs/PLAN.md, track D).
//!
//! The stand-in foot scans stop 125 mm up. Above that the leg comes from MakeHuman's base mesh
//! (CC0; RESEARCH.md 18):
//! - made male: the three `*-male-young` macro targets at a third each (an even ethnic mix, the
//!   way MakeHuman weights them for gender 1, age 25);
//! - smoothed by two levels of Catmull-Clark, as MakeHuman smooths its own mesh (its quads are
//!   ~10-15 mm on the shin, which would show as facets on a pad fitted to it);
//! - set by three of its own measure targets so that the knee joint sits at ANSUR II's median
//!   tibial height and the calf's largest girth is ANSUR's median, for men 170-180 cm tall
//!   (stand-in 022 is a 175 cm man), and the leg's girth 115 mm up equals the foot scan's own
//!   there, so the two join without a step.
//!
//! Then it is moved over the scan's leg (the sections' centroids at 115 mm), and the two
//! distance fields are blended over 100-120 mm and meshed as one surface.
//!
//! Frames: MakeHuman is in decimetres, y up, z forward, +x the model's left. The foot's standard
//! frame (foot.rs) is mm, x forward, y left, z up, so x = 100 z_mh, y = 100 x_mh,
//! z = 100 (y_mh - floor). The right leg is x_mh < 0, so y < 0 here, and its inner (medial)
//! side faces +y, as on the right-foot stand-ins.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

use cortenforge::cf_design::{Aabb, Solid};
use cortenforge::cf_scan_prep_core::{auto_cap_open_boundaries, clip_mesh_against_plane};
use cortenforge::mesh::io::{load_mesh, save_mesh};
use cortenforge::mesh::repair::weld_vertices;
use cortenforge::mesh::sdf::{CachedGridSdf, PseudoNormalSign, Signed, TriMeshDistance, WALL_THRESHOLD_FACTOR_DEFAULT};
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::foot::{self, Foot};
use crate::render::{Layer, Style, View, render_styled};
use crate::scan;

/// ANSUR II, men 1700-1800 mm tall (2,222 of 4,082), medians in mm (RESEARCH.md 18).
pub mod ansur {
    pub const TIBIAL_HEIGHT: f64 = 466.0;
    pub const CALF_CIRC: f64 = 391.0;
    pub const ANKLE_CIRC: f64 = 228.0;
    pub const KNEE_HEIGHT_MID_PATELLA: f64 = 486.0;
    pub const LATERAL_MALLEOLUS_HEIGHT: f64 = 73.0;
}

/// Where the MakeHuman files are (RESEARCH.md 18): `MAKEHUMAN_DIR`, or the default on D:.
pub fn makehuman_dir() -> PathBuf {
    std::env::var_os("MAKEHUMAN_DIR").map(PathBuf::from).unwrap_or_else(|| PathBuf::from("D:/custom-skate-data/makehuman"))
}

/// MakeHuman's base mesh: every vertex (helpers and joint markers too, since targets index
/// them), the skin's quads (group `body`), and the joint markers' vertices by name.
pub struct MhBase {
    pub verts: Vec<Vector3<f64>>,
    pub quads: Vec<[u32; 4]>,
    /// Skin vertices are `0..n_body`.
    pub n_body: usize,
    pub joints: HashMap<String, Vec<u32>>,
}

pub fn load_base(path: &Path) -> Result<MhBase, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let (mut verts, mut quads, mut joints) = (Vec::new(), Vec::new(), HashMap::<String, Vec<u32>>::new());
    let mut group = String::new();
    for line in text.lines() {
        let mut it = line.split_whitespace();
        match it.next() {
            Some("v") => {
                let c: Vec<f64> = it.take(3).map(|s| s.parse::<f64>().unwrap_or(f64::NAN)).collect();
                if c.len() != 3 || c.iter().any(|x| !x.is_finite()) {
                    return Err(format!("{}: bad vertex line {line:?}", path.display()));
                }
                verts.push(Vector3::new(c[0], c[1], c[2]));
            }
            Some("g") => group = it.next().unwrap_or("").to_string(),
            Some("f") => {
                let idx: Result<Vec<u32>, _> =
                    it.map(|s| s.split('/').next().unwrap_or("").parse::<u32>().map(|i| i - 1)).collect();
                let idx = idx.map_err(|e| format!("{}: bad face {line:?}: {e}", path.display()))?;
                if group == "body" {
                    let q: [u32; 4] = idx.as_slice().try_into().map_err(|_| format!("body face not a quad: {line:?}"))?;
                    quads.push(q);
                } else if group.starts_with("joint-") {
                    joints.entry(group.clone()).or_default().extend(idx);
                }
            }
            _ => {}
        }
    }
    for v in joints.values_mut() {
        v.sort_unstable();
        v.dedup();
    }
    let n_body = quads.iter().flatten().map(|&i| i as usize + 1).max().unwrap_or(0);
    Ok(MhBase { verts, quads, n_body, joints })
}

/// A MakeHuman target (`index dx dy dz` lines, 0-based) as a dense offset per vertex.
pub fn load_target(path: &Path, n: usize) -> Result<Vec<Vector3<f64>>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut d = vec![Vector3::zeros(); n];
    for line in text.lines().filter(|l| !l.starts_with('#') && !l.trim().is_empty()) {
        let f: Vec<&str> = line.split_whitespace().collect();
        let bad = || format!("{}: bad target line {line:?}", path.display());
        if f.len() != 4 {
            return Err(bad());
        }
        let i: usize = f[0].parse().map_err(|_| bad())?;
        let v: Vec<f64> = f[1..].iter().map(|s| s.parse::<f64>()).collect::<Result<_, _>>().map_err(|_| bad())?;
        if i < n {
            d[i] = Vector3::new(v[0], v[1], v[2]);
        }
    }
    Ok(d)
}

/// One level of Catmull-Clark on a quad mesh, as a linear map from the old vertices' values to
/// the new ones. Subdividing is linear in the positions, so a target's offsets can be
/// subdivided on their own and added afterwards. New vertices: the old ones (moved), then a
/// face point per quad, then an edge point per edge. Boundary edges and vertices take the usual
/// crease rules (midpoint; 3/4 + 1/8 + 1/8).
pub struct Subdivision {
    pub stencils: Vec<Vec<(u32, f64)>>,
    pub quads: Vec<[u32; 4]>,
}

impl Subdivision {
    pub fn new(quads: &[[u32; 4]], n: usize) -> Self {
        let mut edge_id: HashMap<(u32, u32), u32> = HashMap::new();
        let (mut edge_verts, mut edge_faces) = (Vec::<[u32; 2]>::new(), Vec::<Vec<u32>>::new());
        let mut face_edges = Vec::with_capacity(quads.len());
        for (fi, q) in quads.iter().enumerate() {
            let mut fe = [0u32; 4];
            for k in 0..4 {
                let (a, b) = (q[k], q[(k + 1) % 4]);
                let key = (a.min(b), a.max(b));
                let id = *edge_id.entry(key).or_insert_with(|| {
                    edge_verts.push([key.0, key.1]);
                    edge_faces.push(Vec::new());
                    (edge_verts.len() - 1) as u32
                });
                edge_faces[id as usize].push(fi as u32);
                fe[k] = id;
            }
            face_edges.push(fe);
        }
        let (nf, ne) = (quads.len(), edge_verts.len());
        let mut st: Vec<Vec<(u32, f64)>> = vec![Vec::new(); n + nf + ne];
        for (fi, q) in quads.iter().enumerate() {
            st[n + fi] = q.iter().map(|&v| (v, 0.25)).collect();
        }
        for e in 0..ne {
            let [a, b] = edge_verts[e];
            st[n + nf + e] = if edge_faces[e].len() == 2 {
                let mut s = vec![(a, 0.25), (b, 0.25)];
                for &f in &edge_faces[e] {
                    s.extend(quads[f as usize].iter().map(|&v| (v, 0.0625)));
                }
                s
            } else {
                vec![(a, 0.5), (b, 0.5)]
            };
        }
        let (mut vf, mut ve) = (vec![Vec::<u32>::new(); n], vec![Vec::<u32>::new(); n]);
        for (fi, q) in quads.iter().enumerate() {
            for &v in q {
                vf[v as usize].push(fi as u32);
            }
        }
        for (e, [a, b]) in edge_verts.iter().enumerate() {
            ve[*a as usize].push(e as u32);
            ve[*b as usize].push(e as u32);
        }
        for v in 0..n {
            if ve[v].is_empty() {
                st[v] = vec![(v as u32, 1.0)];
                continue;
            }
            let boundary: Vec<u32> = ve[v].iter().copied().filter(|&e| edge_faces[e as usize].len() == 1).collect();
            if !boundary.is_empty() {
                st[v] = if boundary.len() == 2 {
                    let mut s = vec![(v as u32, 0.75)];
                    for &e in &boundary {
                        let [a, b] = edge_verts[e as usize];
                        s.push((if a as usize == v { b } else { a }, 0.125));
                    }
                    s
                } else {
                    vec![(v as u32, 1.0)]
                };
                continue;
            }
            // (F + 2R + (k - 3) P) / k: F the mean of the face points, R of the edge midpoints.
            let k = ve[v].len() as f64;
            let nfv = vf[v].len() as f64;
            let mut s = vec![(v as u32, (k - 3.0) / k)];
            for &f in &vf[v] {
                s.extend(quads[f as usize].iter().map(|&u| (u, 0.25 / nfv / k)));
            }
            for &e in &ve[v] {
                let [a, b] = edge_verts[e as usize];
                s.push((a, 1.0 / (k * k)));
                s.push((b, 1.0 / (k * k)));
            }
            st[v] = s;
        }
        let mut nq = Vec::with_capacity(4 * nf);
        for (fi, q) in quads.iter().enumerate() {
            let fp = (n + fi) as u32;
            let ep = |k: usize| (n + nf) as u32 + face_edges[fi][k];
            for k in 0..4 {
                nq.push([q[k], ep(k), fp, ep((k + 3) % 4)]);
            }
        }
        Subdivision { stencils: st, quads: nq }
    }

    pub fn apply(&self, x: &[Vector3<f64>]) -> Vec<Vector3<f64>> {
        self.stencils.iter().map(|s| s.iter().fold(Vector3::zeros(), |a, &(i, w)| a + x[i as usize] * w)).collect()
    }
}

/// The perimeter of the convex hull of points in the xy plane: what a tape measure reads round
/// a limb (it bridges hollows). Andrew's monotone chain.
pub fn tape_perimeter(points: &[(f64, f64)]) -> f64 {
    let mut p: Vec<(f64, f64)> = points.to_vec();
    p.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    p.dedup();
    if p.len() < 3 {
        return 0.0;
    }
    let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0);
    let mut hull: Vec<(f64, f64)> = Vec::with_capacity(2 * p.len());
    for pass in 0..2 {
        let start = hull.len();
        let it: Box<dyn Iterator<Item = &(f64, f64)>> = if pass == 0 { Box::new(p.iter()) } else { Box::new(p.iter().rev()) };
        for &q in it {
            while hull.len() >= start + 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], q) <= 0.0 {
                hull.pop();
            }
            hull.push(q);
        }
        hull.pop();
    }
    (0..hull.len()).map(|i| {
        let (a, b) = (hull[i], hull[(i + 1) % hull.len()]);
        ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt()
    }).sum()
}

/// A horizontal section at height `z`: its tape girth and its centroid (x, y).
pub fn girth_at(mesh: &IndexedMesh, z: f64) -> Option<(f64, (f64, f64))> {
    let cs = scan::section(mesh, Point3::new(0.0, 0.0, z), Vector3::z());
    if cs.points.len() < 3 {
        return None;
    }
    let pts: Vec<(f64, f64)> = cs.points.iter().map(|p| (p.x, p.y)).collect();
    Some((tape_perimeter(&pts), (cs.centroid.x, cs.centroid.y)))
}

/// The three measure targets the fit moves, each a slider in [-1, 1] (`-decr` below 0,
/// `-incr` above), subdivided twice.
const FIT_TARGETS: [&str; 3] = ["measure-lowerleg-height", "measure-calf-circ", "measure-ankle-circ"];

/// MakeHuman's male body, subdivided, with the fit targets ready to add.
struct MhModel {
    /// Subdivided base + male, dm, MakeHuman frame.
    base: Vec<Vector3<f64>>,
    /// Per fit target, (decr, incr), subdivided.
    targets: Vec<(Vec<Vector3<f64>>, Vec<Vector3<f64>>)>,
    /// The same for the joint markers: unsubdivided positions of every vertex (base + male), and
    /// the raw targets.
    raw: Vec<Vector3<f64>>,
    raw_targets: Vec<(Vec<Vector3<f64>>, Vec<Vector3<f64>>)>,
    knee: Vec<u32>,
    ankle: Vec<u32>,
    quads: Vec<[u32; 4]>,
}

impl MhModel {
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
        for name in FIT_TARGETS {
            let load = |s: &str| load_target(&dir.join("targets/measure").join(format!("{name}-{s}.target")), n);
            raw_targets.push((load("decr")?, load("incr")?));
        }
        let targets = raw_targets.iter().map(|(d, i)| (sub(d), sub(i))).collect();
        let joint = |name: &str| base.joints.get(name).cloned().ok_or(format!("base.obj has no {name}"));
        Ok(MhModel {
            base: sub(&raw),
            targets,
            raw,
            raw_targets,
            knee: joint("joint-r-knee")?,
            ankle: joint("joint-r-ankle")?,
            quads: s2.quads,
        })
    }

    fn add(x: &mut [Vector3<f64>], t: &(Vec<Vector3<f64>>, Vec<Vector3<f64>>), w: f64) {
        let (d, s) = if w < 0.0 { (&t.0, -w) } else { (&t.1, w) };
        for (a, b) in x.iter_mut().zip(d) {
            *a += b * s;
        }
    }

    /// Skin vertices (subdivided) and joint vertices (raw) at slider weights `w`, dm.
    fn at(&self, w: &[f64; 3]) -> (Vec<Vector3<f64>>, Vec<Vector3<f64>>) {
        let (mut skin, mut raw) = (self.base.clone(), self.raw.clone());
        for k in 0..3 {
            Self::add(&mut skin, &self.targets[k], w[k]);
            Self::add(&mut raw, &self.raw_targets[k], w[k]);
        }
        (skin, raw)
    }
}

fn mean_of(v: &[Vector3<f64>], idx: &[u32]) -> Vector3<f64> {
    idx.iter().fold(Vector3::zeros(), |a, &i| a + v[i as usize]) / idx.len() as f64
}

/// The right leg's skin quads below `z_top` mm (fixed for every weight: the topology doesn't
/// change), as the connected piece holding the right ankle.
fn right_leg_quads(m: &MhModel, skin: &[Vector3<f64>], floor: f64, z_top: f64) -> Vec<[u32; 4]> {
    let h = |i: u32| 100.0 * (skin[i as usize].y - floor);
    let cand: Vec<[u32; 4]> = m.quads.iter().copied().filter(|q| q.iter().all(|&i| skin[i as usize].x < 0.0 && h(i) < z_top)).collect();
    // Union-find over vertices.
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
    let ankle = mean_of(&m.raw, &m.ankle);
    let seed = cand
        .iter()
        .flatten()
        .copied()
        .min_by(|&a, &b| (skin[a as usize] - ankle).norm().total_cmp(&(skin[b as usize] - ankle).norm()))
        .unwrap_or(0);
    let root = find(&mut parent, seed);
    cand.into_iter().filter(|q| find(&mut parent, q[0]) == root).collect()
}

/// MakeHuman to the foot frame (mm), with the shin stood upright: turned about the ankle joint
/// until the knee joint is straight above it. MakeHuman's rest pose has the feet ~44 cm apart,
/// so its shin leans inward ~7 deg; it also leans forward ~2 deg. The boot is built upright and
/// then leaned forward by its own `lean_deg` (the cuff's forward lean, hockey_boot.rs), which is
/// what sets the shin's angle in the skate, so the leg goes in upright both ways. Then `shift`
/// moves it in x and y.
#[derive(Clone, Copy)]
struct Placement {
    floor: f64,
    pivot: Vector3<f64>,
    rot: nalgebra::Rotation3<f64>,
    shift: (f64, f64),
    /// MakeHuman's shin lean taken out, degrees: inward (seen from the front), forward (side).
    lean: (f64, f64),
}

impl Placement {
    fn new(m: &MhModel, skin: &[Vector3<f64>], raw: &[Vector3<f64>]) -> Self {
        let floor = skin.iter().map(|v| v.y).fold(f64::MAX, f64::min);
        let ff = |v: Vector3<f64>| Vector3::new(100.0 * v.z, 100.0 * v.x, 100.0 * (v.y - floor));
        let (ankle, knee) = (ff(mean_of(raw, &m.ankle)), ff(mean_of(raw, &m.knee)));
        let d = knee - ankle;
        let rot = nalgebra::Rotation3::rotation_between(&d, &Vector3::z()).unwrap_or_else(nalgebra::Rotation3::identity);
        Placement { floor, pivot: ankle, rot, shift: (0.0, 0.0), lean: (d.y.atan2(d.z).to_degrees(), d.x.atan2(d.z).to_degrees()) }
    }

    fn place(&self, v: Vector3<f64>) -> Point3<f64> {
        let p = Vector3::new(100.0 * v.z, 100.0 * v.x, 100.0 * (v.y - self.floor));
        let q = self.rot * (p - self.pivot) + self.pivot;
        Point3::new(q.x + self.shift.0, q.y + self.shift.1, q.z)
    }

}

/// Skin quads to a triangle mesh in the foot frame (mm), only the vertices used.
fn to_foot_frame(skin: &[Vector3<f64>], quads: &[[u32; 4]], pl: &Placement) -> IndexedMesh {
    let mut map = vec![u32::MAX; skin.len()];
    let mut mesh = IndexedMesh::default();
    let mut id = |i: u32, mesh: &mut IndexedMesh| {
        if map[i as usize] == u32::MAX {
            map[i as usize] = mesh.vertices.len() as u32;
            mesh.vertices.push(pl.place(skin[i as usize]));
        }
        map[i as usize]
    };
    for q in quads {
        let [a, b, c, d] = q.map(|i| id(i, &mut mesh));
        mesh.faces.push([a, b, c]);
        mesh.faces.push([a, c, d]);
    }
    mesh
}

/// What the fit measures, mm, on the upright leg: the knee joint's height, the calf's largest
/// girth, the girth at the join height.
fn measures(m: &MhModel, w: &[f64; 3], quads: &[[u32; 4]], join_z: f64) -> ([f64; 3], IndexedMesh, Placement) {
    let (skin, raw) = m.at(w);
    let pl = Placement::new(m, &skin, &raw);
    let mesh = to_foot_frame(&skin, quads, &pl);
    let knee = pl.place(mean_of(&raw, &m.knee)).z;
    let calf = (0..)
        .map(|k| 150.0 + 2.0 * k as f64)
        .take_while(|&z| z < knee - 40.0)
        .filter_map(|z| girth_at(&mesh, z).map(|g| g.0))
        .fold(0.0, f64::max);
    let join = girth_at(&mesh, join_z).map(|g| g.0).unwrap_or(f64::NAN);
    ([knee, calf, join], mesh, pl)
}

/// The stand-in leg: one closed surface from the floor to `top` mm, the scan's foot below
/// ~100 mm and MakeHuman's leg above ~120 mm.
pub struct Leg {
    pub mesh: IndexedMesh,
    /// MakeHuman's leg alone, in the same frame (closed, cut at its ends): for drawing.
    pub mh: IndexedMesh,
    pub foot: Foot,
    /// Knee and ankle joint centres (MakeHuman's markers), mm.
    pub knee: Point3<f64>,
    pub ankle: Point3<f64>,
    pub weights: [f64; 3],
    pub top: f64,
}

pub const JOIN_Z: f64 = 115.0;
const BLEND: (f64, f64) = (100.0, 120.0);

pub fn build(scan_path: &Path, top: f64) -> Result<Leg, String> {
    let t0 = Instant::now();
    let (raw, _, _) = scan::load(scan_path)?;
    let fm = scan::measure(&raw, 20);
    let foot = foot::build(&raw, &fm, 1.5, 24.0)?;
    let (foot_girth, foot_c) =
        girth_at(&foot.mesh, JOIN_Z).ok_or(format!("the scan has no section at {JOIN_Z} mm"))?;
    println!("foot: {} ({:.1} x {:.1} mm); girth {foot_girth:.1} mm at z {JOIN_Z}", scan_path.display(), foot.length, foot.width);

    let dir = makehuman_dir();
    let m = MhModel::load(&dir)?;
    println!("MakeHuman: {} skin quads after two Catmull-Clark levels ({:.1} s)", m.quads.len(), t0.elapsed().as_secs_f64());
    let (skin0, _) = m.at(&[0.0; 3]);
    let floor0 = skin0.iter().map(|v| v.y).fold(f64::MAX, f64::min);
    // The fit always sees the leg to `FIT_TOP`, whatever `top` is kept: the calf's largest girth
    // is ~360 mm up, and a leg cut lower made the fit swell the calf to compensate (2026-11-12).
    const FIT_TOP: f64 = 700.0;
    if top > FIT_TOP - 40.0 {
        return Err(format!("leg top {top} mm: at most {} (the fit's leg reaches {FIT_TOP})", FIT_TOP - 40.0));
    }
    let quads = right_leg_quads(&m, &skin0, floor0, FIT_TOP);

    // Newton on the three sliders, Jacobian by central differences.
    let goal = [ansur::TIBIAL_HEIGHT, ansur::CALF_CIRC, foot_girth];
    let mut w = [0.0; 3];
    let (m0, _, _) = measures(&m, &w, &quads, JOIN_Z);
    println!(
        "  male, unfitted: knee joint {:.1} mm up, calf {:.1}, girth at {JOIN_Z} {:.1} (goals {:.0}, {:.0}, {:.1})",
        m0[0], m0[1], m0[2], goal[0], goal[1], goal[2]
    );
    for it in 0..8 {
        let (f, _, _) = measures(&m, &w, &quads, JOIN_Z);
        let r: Vec<f64> = (0..3).map(|i| f[i] - goal[i]).collect();
        if r.iter().all(|x| x.abs() < 0.3) {
            break;
        }
        let h = 0.05;
        let mut jac = nalgebra::Matrix3::zeros();
        for j in 0..3 {
            let (mut wp, mut wm) = (w, w);
            wp[j] += h;
            wm[j] -= h;
            let (fp, _, _) = measures(&m, &wp, &quads, JOIN_Z);
            let (fm_, _, _) = measures(&m, &wm, &quads, JOIN_Z);
            for i in 0..3 {
                jac[(i, j)] = (fp[i] - fm_[i]) / (2.0 * h);
            }
        }
        let dw = jac.lu().solve(&nalgebra::Vector3::new(r[0], r[1], r[2])).ok_or("fit: singular Jacobian")?;
        for j in 0..3 {
            w[j] = (w[j] - dw[j]).clamp(-1.0, 1.0);
        }
        println!("  fit {it}: residuals {:+.2} {:+.2} {:+.2} mm -> sliders {:+.3} {:+.3} {:+.3}", r[0], r[1], r[2], w[0], w[1], w[2]);
    }
    let (f, unshifted, mut pl) = measures(&m, &w, &quads, JOIN_Z);
    let (skin, raw_v) = m.at(&w);
    let (_, mh_c) = girth_at(&unshifted, JOIN_Z).ok_or("no MakeHuman section at the join")?;
    pl.shift = (foot_c.0 - mh_c.0, foot_c.1 - mh_c.1);
    let shift = pl.shift;
    let knee = pl.place(mean_of(&raw_v, &m.knee));
    let ankle = pl.place(mean_of(&raw_v, &m.ankle));
    println!(
        "fitted, shin stood upright (MakeHuman's lean taken out: {:.1} deg inward, {:.1} deg forward): knee joint {:.1} mm up (ANSUR tibial height {:.0}), calf {:.1} ({:.0}), girth at {JOIN_Z} {:.1} (scan {:.1}); sliders {}; moved ({:+.1}, {:+.1}) mm onto the scan",
        pl.lean.0.abs(),
        pl.lean.1,
        f[0],
        ansur::TIBIAL_HEIGHT,
        f[1],
        ansur::CALF_CIRC,
        f[2],
        foot_girth,
        FIT_TARGETS.iter().zip(w).map(|(n, x)| format!("{n} {x:+.3}")).collect::<Vec<_>>().join(", "),
        shift.0,
        shift.1
    );

    // MakeHuman's leg alone, cut below the blend and at the top, capped, with its own field.
    let mut mh = to_foot_frame(&skin, &quads, &pl);
    mh = clip_mesh_against_plane(&mh, Point3::new(0.0, 0.0, top), -Vector3::z());
    mh = clip_mesh_against_plane(&mh, Point3::new(0.0, 0.0, BLEND.0 - 20.0), Vector3::z());
    weld_vertices(&mut mh, 1e-6);
    let caps = auto_cap_open_boundaries(&mut mh);
    let open = open_edges(&mh);
    println!("MakeHuman leg: {} triangles, cut at z {:.0} and {top:.0}, {caps} cap(s), {open} open edge(s)", mh.faces.len(), BLEND.0 - 20.0);
    if open > 0 {
        return Err(format!("MakeHuman leg not closed: {open} open edges"));
    }
    let lo = mh.vertices.iter().fold(Vector3::repeat(f64::MAX), |a, v| a.inf(&v.coords)) - Vector3::repeat(30.0);
    let hi = mh.vertices.iter().fold(Vector3::repeat(f64::MIN), |a, v| a.sup(&v.coords)) + Vector3::repeat(30.0);
    // Its distances: exact (to its closed, clean mesh) within `NEAR` mm of the skin, where the
    // surface is made; a 2 mm grid further out, where only the sign and a rough value matter. A
    // grid alone left terraces on the leg; exact everywhere took 306 s to mesh (CORTENFORGE.md).
    let t = Instant::now();
    let dist = TriMeshDistance::new(mh.clone()).map_err(|e| format!("distance: {e}"))?;
    let (grid, _) = CachedGridSdf::build(&dist, Aabb::new(Point3::from(lo), Point3::from(hi)), 2.0, WALL_THRESHOLD_FACTOR_DEFAULT)
        .map_err(|e| format!("sdf grid: {e:?}"))?;
    let sign = PseudoNormalSign::from_distance(&dist);
    let exact = Arc::new(Signed { distance: dist, sign });
    let coarse = Arc::new(grid);
    println!("  its coarse grid, 2 mm: {:.1} s", t.elapsed().as_secs_f64());
    const NEAR: f64 = 4.0;
    let leg_d = move |p: Point3<f64>| {
        let g = coarse.signed_distance(p);
        if g.abs() > NEAR { g } else { exact.evaluate(p) }
    };

    // One surface: the foot's field below the blend, the leg's above, mixed between.
    let fs = foot.sdf.clone();
    let fb = foot.bias;
    let field = move |p: Point3<f64>| {
        let s = smoothstep(BLEND.0, BLEND.1, p.z);
        if s <= 0.0 {
            fs.signed_distance(p) + fb
        } else if s >= 1.0 {
            leg_d(p)
        } else {
            (1.0 - s) * (fs.signed_distance(p) + fb) + s * leg_d(p)
        }
    };
    let flo = foot.bounds.min.coords.inf(&lo);
    let fhi = foot.bounds.max.coords.sup(&hi);
    let bounds = Aabb::new(Point3::new(flo.x, flo.y, -2.0), Point3::new(fhi.x, fhi.y, top + 2.0));
    let solid = Solid::user_fn(field, bounds)
        .intersect(Solid::plane(Vector3::z(), top))
        .intersect(Solid::plane(-Vector3::z(), 0.5));
    let t = Instant::now();
    let mut mesh = solid.mesh(1.0).geometry;
    weld_vertices(&mut mesh, 1e-6);
    println!("stand-in leg: {} triangles, {} open edge(s) ({:.1} s; {:.1} s in all)", mesh.faces.len(), open_edges(&mesh), t.elapsed().as_secs_f64(), t0.elapsed().as_secs_f64());
    Ok(Leg { mesh, mh, foot, knee, ankle, weights: w, top })
}

impl Leg {
    /// The foot scan and the leg as one `Foot` (standard frame, the foot's own length and
    /// width), with a distance grid at `cell` mm reaching `margin` mm past it: what the boot is
    /// built on.
    pub fn as_foot(&self, cell: f64, margin: f64) -> Result<Foot, String> {
        foot::from_standard(self.mesh.clone(), self.foot.length, self.foot.width, cell, margin)
    }

    /// The scan's own top (above it the surface is MakeHuman's), mm.
    pub fn scan_top(&self) -> f64 {
        self.foot.height
    }
}

fn smoothstep(e0: f64, e1: f64, x: f64) -> f64 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

pub fn open_edges(m: &IndexedMesh) -> usize {
    let mut count: HashMap<(u32, u32), u32> = HashMap::new();
    for f in &m.faces {
        for k in 0..3 {
            let (a, b) = (f[k], f[(k + 1) % 3]);
            *count.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    count.values().filter(|&&c| c != 2).count()
}

/// `cargo run --release -- leg <scan> [out]`: the stand-in leg on one foot scan, measured
/// against ANSUR II, drawn alone and in the boot (if `out/boot` has the boot's meshes).
pub fn run(scan_path: &Path, out: &Path) -> Result<(), String> {
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let docs = Path::new("docs/leg");
    std::fs::create_dir_all(docs).map_err(|e| e.to_string())?;
    let leg = build(scan_path, 640.0)?;
    let mut lines = Vec::new();
    let mut say = |s: String| {
        println!("{s}");
        lines.push(s);
    };

    // The joined leg measured the way ANSUR measures (a tape round it, horizontal).
    let girths: Vec<(f64, f64, (f64, f64))> = (0..)
        .map(|k| 60.0 + 5.0 * k as f64)
        .take_while(|&z| z < leg.knee.z + 120.0)
        .filter_map(|z| girth_at(&leg.mesh, z).map(|(g, c)| (z, g, c)))
        .collect();
    let calf = girths.iter().filter(|g| g.0 > 150.0 && g.0 < leg.knee.z - 40.0).fold((0.0, 0.0), |a, g| if g.1 > a.1 { (g.0, g.1) } else { a });
    let ankle_min = girths.iter().filter(|g| g.0 > 90.0 && g.0 < calf.0).fold((0.0, f64::MAX), |a, g| if g.1 < a.1 { (g.0, g.1) } else { a });
    say(format!(
        "joined leg: knee joint ({:.0}, {:.0}, {:.0}), ankle joint ({:.0}, {:.0}, {:.0}) mm",
        leg.knee.x, leg.knee.y, leg.knee.z, leg.ankle.x, leg.ankle.y, leg.ankle.z
    ));
    say(format!(
        "  calf girth {:.1} mm at z {:.0} (ANSUR median {:.0}); smallest girth above the ankle {:.1} mm at z {:.0} (ANSUR ankle circumference median {:.0}, p5-p95 207-252)",
        calf.1,
        calf.0,
        ansur::CALF_CIRC,
        ankle_min.1,
        ankle_min.0,
        ansur::ANKLE_CIRC
    ));
    // The join: girths through the blend, and the step in the section's centre.
    for g in girths.iter().filter(|g| g.0 >= 90.0 && g.0 <= 140.0) {
        say(format!("  z {:>5.0}: girth {:.1} mm, centre ({:.1}, {:.1})", g.0, g.1, g.2.0, g.2.1));
    }
    // The front of the ankle through the join: the most forward point of each section on the
    // scan, on MakeHuman's leg (cut at 80 mm) and on the joined surface. MakeHuman's front is
    // fuller there; below the blend only the scan counts.
    let front = |m: &IndexedMesh, z: f64| {
        let cs = scan::section(m, Point3::new(0.0, 0.0, z), Vector3::z());
        cs.points.iter().map(|p| p.x).fold(f64::NAN, f64::max)
    };
    for z in [85.0, 95.0, 100.0, 105.0, 110.0, 115.0, 120.0, 125.0, 135.0] {
        let (s, m, j) = (front(&leg.foot.mesh, z), front(&leg.mh, z), front(&leg.mesh, z));
        say(format!("  front of the ankle, z {z:>3.0}: scan x {s:>6.1}, MakeHuman {m:>6.1} ({:+.1}), joined {j:>6.1}", m - s));
    }
    // The shin's front edge (the tibial crest's skin line): the most forward point of each
    // section between the ankle and the knee. The pad's shell runs along it.
    let crest: Vec<Point3<f64>> = (0..)
        .map(|k| leg.ankle.z + 40.0 + 20.0 * k as f64)
        .take_while(|&z| z < leg.knee.z - 30.0)
        .filter_map(|z| {
            let cs = scan::section(&leg.mesh, Point3::new(0.0, 0.0, z), Vector3::z());
            cs.points.iter().copied().max_by(|a, b| a.x.total_cmp(&b.x))
        })
        .collect();
    say(format!(
        "  shin's front line (most forward point per section): {}",
        crest.iter().map(|p| format!("({:.0}, {:.0}, {:.0})", p.x, p.y, p.z)).collect::<Vec<_>>().join(" ")
    ));

    save_mesh(&leg.mesh, &out.join("leg.stl")).map_err(|e| e.to_string())?;
    save_mesh(&leg.mh, &out.join("makehuman_leg.stl")).map_err(|e| e.to_string())?;

    // Drawings: the leg from four sides, then in the boot.
    let skin = [0.85, 0.70, 0.60];
    let e = |e: std::io::Error| e.to_string();
    let pretty = Style::pretty();
    let views = [
        View::looking(-Vector3::x(), Vector3::z()),
        View::looking(Vector3::y(), Vector3::z()),
        View::looking(-Vector3::y(), Vector3::z()),
        View::looking(Vector3::new(-0.6, 0.65, -0.3), Vector3::z()),
    ];
    render_styled(&[Layer::solid(&leg.mesh, skin)], &views, 1.0, &docs.join("leg.png"), pretty).map_err(e)?;
    // The join, close: the scan (grey) and MakeHuman's leg (skin) where they overlap.
    let near: Vec<View> = vec![View::looking(Vector3::new(-0.3, 1.0, -0.15), Vector3::z()), View::looking(Vector3::new(-1.0, -0.4, -0.15), Vector3::z())];
    let foot_low = clip_mesh_against_plane(&leg.foot.mesh, Point3::new(0.0, 0.0, 200.0), -Vector3::z());
    // MakeHuman's leg from where it starts to count (the blend's bottom): its own cut at 80 mm
    // stood ~5 mm proud of the scan's instep and read as an overhang (Jon, 2026-11-12).
    let mh_low = clip_mesh_against_plane(&leg.mh, Point3::new(0.0, 0.0, 220.0), -Vector3::z());
    let mh_low = clip_mesh_against_plane(&mh_low, Point3::new(0.0, 0.0, BLEND.0), Vector3::z());
    render_styled(&[Layer::solid(&foot_low, [0.6, 0.6, 0.62]), Layer::solid(&mh_low, skin)], &near, 2.5, &docs.join("leg_join.png"), pretty).map_err(e)?;
    render_styled(&[Layer::solid(&clip_mesh_against_plane(&leg.mesh, Point3::new(0.0, 0.0, 220.0), -Vector3::z()), skin)], &near, 2.5, &docs.join("leg_joined.png"), pretty).map_err(e)?;

    // The boot's meshes (`LEG_BOOT_DIR`, default out/boot) are leaned forward by the boot's
    // `lean_deg` above the ankle bones; the leg is leaned the same way to sit in them.
    let boot_dir = std::env::var_os("LEG_BOOT_DIR").map(PathBuf::from).unwrap_or_else(|| PathBuf::from("out/boot"));
    let shell_p = boot_dir.join("shell.stl");
    if shell_p.exists() {
        let load = |n: &str| load_mesh(boot_dir.join(n)).map_err(|e| format!("{n}: {e}"));
        let shell = load("shell.stl")?;
        let tongue = load("tongue.stl")?;
        let liner = load("liner.stl")?;
        let lm = crate::hockey_boot::landmarks(&leg.foot, leg.scan_top());
        let zp = 0.5 * (lm.medial.z + lm.lateral.z);
        let lean_deg = crate::hockey_boot::HockeyParams::default().lean_deg;
        let mut leaned = leg.mesh.clone();
        for v in &mut leaned.vertices {
            v.x += crate::hockey_boot::lean_dx_at(zp, lean_deg, v.z);
        }
        // How the liner meets the leg above the scan: its vertices inside the leg, and the
        // closest it comes (the liner's outer face is ~4 mm further out and never the closest).
        let dist = TriMeshDistance::new(leaned.clone()).map_err(|e| format!("distance: {e}"))?;
        let sign = PseudoNormalSign::from_distance(&dist);
        let leg_dist = Signed { distance: dist, sign };
        let mut worst = (f64::MAX, Point3::origin());
        let (mut inside, mut deep) = (0, 0);
        for v in liner.vertices.iter().filter(|v| v.z > 125.0) {
            let d = leg_dist.evaluate(*v);
            inside += usize::from(d < 0.0);
            deep += usize::from(d < -1.5);
            if d < worst.0 {
                worst = (d, *v);
            }
        }
        say(format!(
            "in the boot ({}; leg leaned {lean_deg} deg like the boot): {inside} liner vertices above z 125 inside the leg, {deep} more than 1.5 mm in; closest {:+.1} mm (negative: into the leg) at ({:.0}, {:.0}, {:.0})",
            boot_dir.display(),
            worst.0,
            worst.1.x,
            worst.1.y,
            worst.1.z
        ));
        let leg_up = clip_mesh_against_plane(&leaned, Point3::new(0.0, 0.0, 100.0), Vector3::z());
        let layers = [Layer::solid(&leg_up, skin), Layer::solid(&shell, [0.12, 0.12, 0.13]).with_spec(0.35), Layer::solid(&tongue, [0.9, 0.9, 0.87])];
        let hero = [View::looking(Vector3::new(-0.62, 0.62, -0.30), Vector3::z()), View::looking(Vector3::new(0.0, 1.0, 0.0), Vector3::z())];
        render_styled(&layers, &hero, 1.4, &docs.join("leg_in_boot.png"), pretty).map_err(e)?;
    } else {
        say("no out/boot/shell.stl: run `boot` first to draw the leg in the boot".into());
    }
    std::fs::write(docs.join("summary.txt"), lines.join("\n") + "\n").map_err(|e| e.to_string())?;
    Ok(())
}
