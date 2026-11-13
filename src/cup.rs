//! G3, round 2: the knee shield as a cup on the bent knee.
//!
//! Jon, on round 1's `gap.png` (2026-11-12): "the knee part is usually more of a cup, as hockey
//! players usually are always skating with a knee bend, and wearing socks, or in roller hockey,
//! pants". So the shin pad's knee pieces (the knee shield with its liner, the thigh guard with its
//! foam) are built on the body bent to the skating knee (`KNEE_BUILD`, RESEARCH.md 23):
//! - the stand-in body (`body.rs`) posed by MakeHuman's skeleton and skin weights, read into the
//!   shin's frame and leaned like the boot, so the pad's lower parts (`pad` run, on the leg) meet
//!   the cup as before: below the knee the bent body is the standing one;
//! - each part an offset from that bent skin, with the pad's own layers (`pad::Pad::field`);
//! - the outlines stay those drawn on the standing leg (`pad::Outlines`): the skin carries them to
//!   the bent knee (`pad::Warp`), each point near the bent skin reading the angle round the leg
//!   and the height that the closest skin point had standing.
//!
//! Worn, the sock (ice) or pants (roller) hold the cup on the knee. At any other pose each piece
//! rests on the skin under it (`fit`): a rigid motion where the skin pressing into its liner
//! costs far more than the liner standing off it (the sock's pull). What is left says how the cup
//! sits: skin pressing into the liner, or the liner standing off the skin.

use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

use cortenforge::cf_design::Aabb;
use cortenforge::cf_scan_prep_core::{auto_cap_open_boundaries, clip_mesh_against_plane};
use cortenforge::mesh::io::{load_mesh, save_mesh};
use cortenforge::mesh::repair::weld_vertices;
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Matrix3, Point3, Rotation3, Vector3};

use crate::body::{self, Body};
use crate::hockey_boot::mesh_volume;
use crate::pad::{self, Field, Leaned, NearMesh, Outlines, Pad, PadParams, Part, R_REF, Ring, Warp, boot_top, draw};
use crate::pose::{LegPose, Motion};
use crate::render::{Layer, Style, View};

/// The knee's bend the cup is built at, degrees: between the treadmill stride's 65.5 (Lockwood et
/// al. 2023) and the on-ice side-cut's peak 75.2 (Yu et al. 2025); RESEARCH.md 23.
pub const KNEE_BUILD: f64 = 70.0;
/// The hip's, for the same stance (59.6 and 65.8 in the same studies). It doesn't shape the knee:
/// the skin there moves with the thigh and shin bones only.
pub const HIP_BUILD: f64 = 60.0;

pub fn build_pose() -> LegPose {
    LegPose { hip: HIP_BUILD, knee: KNEE_BUILD, ankle: 0.0 }
}

/// The cup's two hard pieces, each moving as one with its foam.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Piece {
    Shield,
    Thigh,
}

impl Piece {
    pub const ALL: [Piece; 2] = [Piece::Shield, Piece::Thigh];

    pub fn parts(&self) -> [Part; 2] {
        match self {
            Piece::Shield => [Part::KneeCap, Part::KneeLiner],
            Piece::Thigh => [Part::ThighGuard, Part::ThighFoam],
        }
    }

    pub fn of(part: Part) -> Option<Piece> {
        Piece::ALL.into_iter().find(|p| p.parts().contains(&part))
    }

    pub fn name(&self) -> &'static str {
        match self {
            Piece::Shield => "knee shield",
            Piece::Thigh => "thigh guard",
        }
    }
}

/// The carried outlines' blend round the skin: sigma this times the distance from it (`pad::Warp`).
pub const WARP_SMOOTH: f64 = 0.6;

pub const CUP_PARTS: [Part; 4] = [Part::KneeCap, Part::KneeLiner, Part::ThighGuard, Part::ThighFoam];

/// The file a cup part is saved to, in `out/pad` (the leaned shin frame at the build pose).
pub fn stl_name(part: Part) -> String {
    format!("cup-{}.stl", part.name().replace([' ', ','], "_").replace("__", "_"))
}

/// The body posed (both legs alike), with the standing body's vertices in the same order, its
/// joints, and every bone's motion from standing.
pub fn posed(b: &Body, p: LegPose) -> Result<(IndexedMesh, HashMap<String, Point3<f64>>, Vec<Motion>), String> {
    let (m, j) = b.posed_like_standing([p, p]);
    if m.vertices.len() != b.mesh.vertices.len() || m.faces != b.mesh.faces {
        return Err(format!("posed body (hip {}, knee {}) doesn't match the standing one vertex for vertex", p.hip, p.knee));
    }
    Ok((m, j, b.moves([p, p])))
}

/// The pad frame's (theta, s, z) for a standing body point (leaned like the pad).
fn standing_tsz(ll: &Leaned, v: Point3<f64>) -> (f64, f64, f64) {
    let p = ll.lean(v);
    let th = ll.f.theta(p);
    (th, th.to_radians() * R_REF, p.z)
}

/// The skin under each piece: the right leg's vertices whose standing place is inside its
/// outline.
pub fn anchors(b: &Body, ll: &Leaned, o: &Outlines) -> [Vec<usize>; 2] {
    let mid = b.shift.1;
    let mut out = [Vec::new(), Vec::new()];
    for (i, v) in b.mesh.vertices.iter().enumerate() {
        if v.y >= mid {
            continue;
        }
        let (_, s, z) = standing_tsz(ll, *v);
        if o.knee.eval(s, z) < 0.0 {
            out[0].push(i);
        }
        if o.thigh.eval(s, z) < 0.0 {
            out[1].push(i);
        }
    }
    out
}

/// A rotation's angle, degrees, from its trace (clamped: nalgebra's `angle()` gives NaN a hair
/// from the identity).
pub fn turn_deg(r: &Rotation3<f64>) -> f64 {
    ((r.matrix().trace() - 1.0) / 2.0).clamp(-1.0, 1.0).acos().to_degrees()
}

/// The rigid motion taking points `a` to `b` best, by least squares (Kabsch: the SVD of the
/// cross-covariance, a reflection ruled out).
pub fn kabsch(a: &[Point3<f64>], b: &[Point3<f64>]) -> Motion {
    let n = a.len().max(1) as f64;
    let ca = a.iter().fold(Vector3::zeros(), |s, p| s + p.coords) / n;
    let cb = b.iter().fold(Vector3::zeros(), |s, p| s + p.coords) / n;
    let mut h = Matrix3::zeros();
    for (p, q) in a.iter().zip(b) {
        h += (p.coords - ca) * (q.coords - cb).transpose();
    }
    let svd = h.svd(true, true);
    let (u, vt) = (svd.u.unwrap(), svd.v_t.unwrap());
    let d = (vt.transpose() * u.transpose()).determinant().signum();
    let r = vt.transpose() * Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, d)) * u.transpose();
    let r = Rotation3::from_matrix_unchecked(r);
    Motion { r, t: cb - r * ca }
}

/// Per vertex, the outward unit normal (area-weighted), turned so that the front of the right
/// shin faces forward.
pub fn normals(m: &IndexedMesh, front_of_shin: usize) -> Vec<Vector3<f64>> {
    let mut n = vec![Vector3::zeros(); m.vertices.len()];
    for f in &m.faces {
        let [a, b, c] = f.map(|i| m.vertices[i as usize]);
        let fn_ = (b - a).cross(&(c - a));
        for &i in f {
            n[i as usize] += fn_;
        }
    }
    let flip = if n[front_of_shin].x < 0.0 { -1.0 } else { 1.0 };
    n.into_iter().map(|v| v.try_normalize(1e-12).unwrap_or_else(Vector3::zeros) * flip).collect()
}

/// The right shin's most forward vertex 100 mm below the knee, standing (to orient normals).
pub fn front_of_shin(b: &Body) -> usize {
    let k = b.joints["r knee"];
    b.mesh
        .vertices
        .iter()
        .enumerate()
        .filter(|(_, v)| v.y < b.shift.1 && (v.z - (k.z - 100.0)).abs() < 8.0)
        .max_by(|a, b| a.1.x.total_cmp(&b.1.x))
        .map(|(i, _)| i)
        .unwrap_or(0)
}

/// How a piece sits after its fit: per anchor, the skin's offset from where the piece carries it,
/// along the skin's outward normal (+: the skin pushes into the liner, -: it stands off) and
/// across it (sliding under the piece). (most into, most off, most slide), mm.
pub fn sit(idx: &[usize], from: &IndexedMesh, to: &IndexedMesh, m: &Motion, n_to: &[Vector3<f64>]) -> (f64, f64, f64) {
    let (mut into, mut off, mut slide) = (0.0f64, 0.0f64, 0.0f64);
    for &i in idx {
        let e = to.vertices[i] - Point3::from(m.apply(from.vertices[i].coords));
        let dn = e.dot(&n_to[i]);
        into = into.max(dn);
        off = off.max(-dn);
        slide = slide.max((e - n_to[i] * dn).norm());
    }
    (into, off, slide)
}

/// How hard the skin pushes back where it would go into a piece's liner, against how hard the
/// sock or pants pull the piece on where it stands off: the foam is far stiffer than the pull.
pub const PUSH_OVER_PULL: f64 = 100.0;
/// How firmly the sock holds the piece where the skin carries it, along the skin (against the
/// pull across it): friction and the sock's own hold; small.
pub const HOLD: f64 = 0.05;

/// A piece's place on the skin from the build pose to another (both in the standing frame): it
/// rests on the skin. A rigid motion found by reweighted least squares (Gauss-Newton on a small
/// turn and shift, 40 rounds) from the plain fit (Kabsch): per skin point under the piece, its
/// offset across the skin weighted `PUSH_OVER_PULL` where the skin is past the piece's inside
/// (pressing into the liner) and 1 where it stands off (the pull), plus `HOLD` times its offset
/// along the skin.
pub fn fit(idx: &[usize], from: &IndexedMesh, to: &IndexedMesh, n_to: &[Vector3<f64>]) -> Motion {
    let a: Vec<Point3<f64>> = idx.iter().map(|&i| from.vertices[i]).collect();
    let b: Vec<Point3<f64>> = idx.iter().map(|&i| to.vertices[i]).collect();
    let mut m = kabsch(&a, &b);
    let c = b.iter().fold(Vector3::zeros(), |s, p| s + p.coords) / b.len().max(1) as f64;
    for _ in 0..40 {
        let mut h = nalgebra::Matrix6::<f64>::zeros();
        let mut g = nalgebra::Vector6::<f64>::zeros();
        for (k, &i) in idx.iter().enumerate() {
            let x = m.apply(a[k].coords);
            let e = x - b[k].coords;
            let n = n_to[i];
            // d(x)/d(turn w, shift t) = [-[x - c]x, I].
            let r = x - c;
            let mut j = nalgebra::Matrix3x6::<f64>::zeros();
            j.fixed_view_mut::<3, 3>(0, 0).copy_from(&(-r.cross_matrix()));
            j.fixed_view_mut::<3, 3>(0, 3).copy_from(&Matrix3::identity());
            // Past the piece's inside: the skin is further out than the piece carries it.
            let s = n.dot(&e);
            let wn = if s < 0.0 { PUSH_OVER_PULL } else { 1.0 };
            let jn = n.transpose() * j;
            let p = Matrix3::identity() - n * n.transpose();
            let jt = p * j;
            h += jn.transpose() * jn * wn + jt.transpose() * jt * HOLD;
            g += jn.transpose() * (s * wn) + jt.transpose() * (p * e) * HOLD;
        }
        let Some(d) = h.lu().solve(&(-g)) else { break };
        let w = Vector3::new(d[0], d[1], d[2]);
        let step = Rotation3::new(w);
        let t = Vector3::new(d[3], d[4], d[5]);
        // x -> step (x - c) + c + t, after m.
        let dm = Motion { r: step, t: c - step * c + t };
        m = dm.after(&m);
        if d.norm() < 1e-7 {
            break;
        }
    }
    m
}

/// Clip a mesh to the side of each plane its normal points to, then cap the cuts.
fn clip_closed(m: &IndexedMesh, planes: &[(Point3<f64>, Vector3<f64>)]) -> IndexedMesh {
    let mut c = m.clone();
    for (p, n) in planes {
        c = clip_mesh_against_plane(&c, *p, *n);
    }
    weld_vertices(&mut c, 1e-6);
    auto_cap_open_boundaries(&mut c);
    c
}

fn grams(m: &IndexedMesh, part: Part) -> f64 {
    mesh_volume(m).abs() / 1000.0 * part.material().density / 1000.0
}

fn bbox_of<'a>(pts: impl Iterator<Item = &'a Point3<f64>>) -> Option<(Point3<f64>, Point3<f64>)> {
    let mut it = pts.peekable();
    it.peek()?;
    let (lo, hi) = it.fold((Vector3::repeat(f64::MAX), Vector3::repeat(f64::MIN)), |(lo, hi), p| (lo.inf(&p.coords), hi.sup(&p.coords)));
    Some((Point3::from(lo), Point3::from(hi)))
}

/// `cargo run --release -- cup <scan>`: build the cup, check it, weigh it against the straight
/// pieces, see how it sits through the knee's range, draw it.
pub fn run(scan_path: &Path) -> Result<(), String> {
    let t0 = Instant::now();
    let out = Path::new("out/pad");
    let docs = Path::new("docs/pad");
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let threads = std::env::var("PAD_THREADS").ok().and_then(|v| v.parse::<usize>().ok()).unwrap_or(4).max(1);
    let mut lines: Vec<String> = Vec::new();
    let mut say = |l: String| {
        println!("{l}");
        lines.push(l);
    };

    let ll = pad::leaned_leg(scan_path)?;
    let load = |n: &str| load_mesh(Path::new("out/boot").join(n)).map_err(|e| format!("out/boot/{n}: {e} (run `boot` first)"));
    let shell = load("shell.stl")?;
    let p = PadParams::default();
    let top = boot_top(&shell, &ll.f);
    let z_bot = Ring(top.0.iter().map(|z| z + p.clear_boot).collect());
    let o = Arc::new(Outlines::new(ll.knee.z));
    let b = body::build(scan_path, 1250.0)?;
    let shin = b.bone("lowerleg01.R").ok_or("no lowerleg01.R")?;
    let mid = b.shift.1;

    // The body bent to the build pose, in the shin's frame, leaned like the boot.
    let bp = build_pose();
    let (pm, pj, mv) = posed(&b, bp)?;
    let to_shin = mv[shin].inverse();
    let in_frame = |v: Point3<f64>| ll.lean(Point3::from(to_shin.apply(v.coords)));
    let mut bent = pm.clone();
    bent.vertices.iter_mut().for_each(|v| *v = in_frame(*v));
    let (kb, hb) = (in_frame(pj["r knee"]), in_frame(pj["r hip"]));
    let thigh_dir = (hb - kb).normalize();
    say(format!(
        "the cup (G3, round 2): the knee shield and thigh guard built on the stand-in body ({}) bent to hip {HIP_BUILD}, knee {KNEE_BUILD} deg, in the shin's frame leaned {} deg like the boot",
        scan_path.display(),
        ll.lean
    ));
    say(format!(
        "knee joint: the leg's ({:.0}, {:.0}, {:.0}), the bent body's in the shin's frame ({:.0}, {:.0}, {:.0}) mm; the line from knee to hip {:.1} deg from the shin's axis, leaned (standing {:.1}: the hip stands behind the knee, so the bend from standing is the angle less that, and the lean adds a little)",
        ll.knee.x,
        ll.knee.y,
        ll.knee.z,
        kb.x,
        kb.y,
        kb.z,
        thigh_dir.dot(&ll.f.u).clamp(-1.0, 1.0).acos().to_degrees(),
        (ll.lean(b.joints["r hip"]) - ll.lean(b.joints["r knee"])).normalize().dot(&ll.f.u).clamp(-1.0, 1.0).acos().to_degrees()
    ));

    // The bent knee as a closed surface: the right leg from 120 mm below the knee to 250 mm up
    // the thigh.
    let closed = clip_closed(
        &bent,
        &[
            (Point3::new(0.0, mid, 0.0), -Vector3::y()),
            (Point3::new(0.0, 0.0, kb.z - 120.0), Vector3::z()),
            (kb + thigh_dir * 250.0, -thigh_dir),
        ],
    );
    let bent_f = Field::new(&closed, 25.0, 2.0, Some(6.0), "bent knee")?;

    // Where each piece goes: the bent skin under its outline (standing), padded for the shells.
    let vals: Vec<[f64; 3]> = b
        .mesh
        .vertices
        .iter()
        .map(|v| {
            let (th, _, z) = standing_tsz(&ll, *v);
            let t = th.to_radians();
            [t.cos(), t.sin(), z]
        })
        .collect();
    let right: Vec<bool> = b.mesh.vertices.iter().map(|v| v.y < mid).collect();
    let near = |e: &dyn Fn(f64, f64) -> f64| {
        bbox_of(bent.vertices.iter().enumerate().filter(|(i, _)| right[*i] && e(vals[*i][1].atan2(vals[*i][0]) * R_REF, vals[*i][2]) < 8.0).map(|(_, v)| v))
    };
    let pad_box = |(lo, hi): (Point3<f64>, Point3<f64>)| Aabb::new(lo - Vector3::repeat(25.0), hi + Vector3::repeat(25.0));
    let knee_box = pad_box(near(&|s, z| o.knee.eval(s, z)).ok_or("no skin under the knee shield")?);
    let thigh_box = pad_box(near(&|s, z| o.thigh.eval(s, z)).ok_or("no skin under the thigh guard")?);
    let lo = Point3::from(knee_box.min.coords.inf(&thigh_box.min.coords));
    let hi = Point3::from(knee_box.max.coords.sup(&thigh_box.max.coords));
    let reach = 45.0;
    let faces: Vec<[u32; 3]> = bent
        .faces
        .iter()
        .copied()
        .filter(|f| {
            f.iter().all(|&i| right[i as usize])
                && f.iter().any(|&i| {
                    let v = bent.vertices[i as usize];
                    (0..3).all(|k| v[k] > lo[k] - reach && v[k] < hi[k] + reach)
                })
        })
        .collect();
    let t1 = Instant::now();
    let warp = Warp::build(&bent, &vals, &faces, lo, hi, 1.5, reach, WARP_SMOOTH, threads);
    say(format!(
        "outlines carried to the bent knee: {} skin triangles, a {:.0} x {:.0} x {:.0} mm grid at 1.5 mm ({:.1} s)",
        faces.len(),
        hi.x - lo.x,
        hi.y - lo.y,
        hi.z - lo.z,
        t1.elapsed().as_secs_f64()
    ));

    // The pad's lower parts (the `pad` run's flush pad, on the leg): the shield laps over the
    // hard ones and the shin liner's border.
    let flush_name = |pt: Part| format!("fitted_tongue_under_flush-{}.stl", pt.name().replace([' ', ','], "_").replace("__", "_"));
    let lower: Vec<(Part, IndexedMesh)> = Part::all()
        .into_iter()
        .filter(|pt| Piece::of(*pt).is_none())
        .map(|pt| load_mesh(out.join(flush_name(pt))).map(|m| (pt, m)).map_err(|e| format!("out/pad/{}: {e} (run `pad` first)", flush_name(pt))))
        .collect::<Result<_, _>>()?;
    let lapped = [Part::ShinShell, Part::SidePanel(true), Part::SidePanel(false), Part::ShinLiner];
    let lap: Vec<NearMesh> = lower.iter().filter(|(pt, _)| lapped.contains(pt)).map(|(_, m)| NearMesh::new(m, "lapped part")).collect::<Result<_, _>>()?;

    let cup = Pad {
        name: "cup".into(),
        lap: Some(Arc::new(lap)),
        body: Arc::new(pad::Body::Fitted { leg: bent_f.clone(), tongue: None, laces: None }),
        leg: bent_f.clone(),
        f: ll.f,
        p,
        z_bot,
        knee_z: ll.knee.z,
        lo: lo.coords,
        hi: hi.coords,
        flush: true,
        o: o.clone(),
        warp: Some(Arc::new(warp)),
        boxes: Some(Arc::new(vec![
            (Part::KneeCap, knee_box),
            (Part::KneeLiner, knee_box),
            (Part::ThighGuard, thigh_box),
            (Part::ThighFoam, thigh_box),
        ])),
    };
    let t2 = Instant::now();
    let parts = pad::build_these(&cup, &CUP_PARTS);
    say(format!("built {} parts ({:.0} s)", parts.len(), t2.elapsed().as_secs_f64()));

    // Against the straight pieces (the `pad` run's flush pad, built on the standing leg).
    say("weights, the cup against the straight pieces (same outlines, layers and PA 12):".into());
    let (mut tc, mut ts) = (0.0, 0.0);
    for (part, m, g) in &parts {
        let straight = load_mesh(out.join(format!("fitted_tongue_under_flush-{}.stl", part.name().replace([' ', ','], "_").replace("__", "_"))))
            .map(|s| grams(&s, *part))
            .unwrap_or(f64::NAN);
        tc += g;
        ts += straight;
        say(format!("  {:<12} {:>6.1} g (straight {:>5.1} g)  {} triangles", part.name(), g, straight, m.faces.len()));
        save_mesh(m, out.join(stl_name(*part))).map_err(|e| e.to_string())?;
    }
    say(format!("  {:<12} {:>6.1} g (straight {:>5.1} g)", "the cup", tc, ts));

    // Checks: the bent skin stays outside the cup's insides (built `ease` off it), and the shield
    // still laps over the front shell's and side panels' tops (they are on the shin, unchanged).
    let fields: Vec<(Part, NearMesh)> = parts.iter().map(|(pt, m, _)| NearMesh::new(m, "cup part").map(|f| (*pt, f))).collect::<Result<_, _>>()?;
    let mut deepest = (f64::MAX, Part::KneeCap);
    for (i, v) in bent.vertices.iter().enumerate() {
        if !right[i] || !(0..3).all(|k| v[k] > lo[k] && v[k] < hi[k]) {
            continue;
        }
        for (pt, f) in &fields {
            let d = f.eval(*v);
            if d < deepest.0 {
                deepest = (d, *pt);
            }
        }
    }
    say(format!("bent skin to the cup: {:.2} mm at the closest (the {}; the sock is {} mm)", deepest.0, deepest.1.name(), p.ease));
    // The parts it laps, against the cup's shield and (for comparison) the straight one.
    let straight = load_mesh(out.join(flush_name(Part::KneeCap))).map_err(|e| e.to_string())?;
    let shields = [("cup", fields.iter().find(|(pt, _)| *pt == Part::KneeCap).unwrap().1.clone()), ("straight", NearMesh::new(&straight, "straight shield")?)];
    for (pt, m) in lower.iter().filter(|(pt, _)| lapped.contains(pt)) {
        let mut row = format!("  the {} to the knee shield:", pt.name());
        for (label, sh) in &shields {
            let (mut best, mut n_in) = (f64::MAX, 0usize);
            for v in m.vertices.iter().filter(|v| (0..3).all(|k| v[k] > knee_box.min[k] && v[k] < knee_box.max[k])) {
                let d = sh.eval(*v);
                best = best.min(d);
                n_in += usize::from(d < 0.0);
            }
            row += &format!(" {label} {best:.2} mm at the closest, {n_in} vertices inside;");
        }
        say(row);
    }

    // How it sits through the knee's range: each piece resting on the skin under it (`fit`).
    let anc = anchors(&b, &ll, &o);
    let fs = front_of_shin(&b);
    let femur = b.bone("upperleg01.R").ok_or("no upperleg01.R")?;
    say(format!(
        "through the knee's range (hip {HIP_BUILD}): each piece resting on the skin under it ({} points under the shield, {} under the thigh guard; the skin pushing into the liner {PUSH_OVER_PULL} times as hard as the sock pulls where it stands off), from the bent pose to each. The skin past the piece's inside (into the liner) and short of it (standing off), and sliding along it, most of each, mm; the shield's turn on the shin and on the thigh, the thigh guard's on the thigh, deg:",
        anc[0].len(),
        anc[1].len()
    ));
    let b_from = pm;
    for knee in [0.0, 15.0, 30.0, 45.0, 60.0, 70.0, 90.0, 105.0, 120.0, 135.0] {
        let q = LegPose { hip: HIP_BUILD, knee, ankle: 0.0 };
        let (qm, _, qv) = posed(&b, q)?;
        let n = normals(&qm, fs);
        let mut row = format!("  knee {knee:>4.0}:");
        for (k, pc) in Piece::ALL.iter().enumerate() {
            let m = fit(&anc[k], &b_from, &qm, &n);
            let (into, off, slide) = sit(&anc[k], &b_from, &qm, &m, &n);
            // The piece's turn read on a bone: the bone's own motion taken out.
            let on = |bone: usize| turn_deg(&qv[bone].inverse().after(&m).after(&mv[bone]).r);
            let turns = match pc {
                Piece::Shield => format!("turns {:>4.1} on the shin, {:>4.1} on the thigh", on(shin), on(femur)),
                Piece::Thigh => format!("turns {:>4.1} on the thigh", on(femur)),
            };
            row += &format!("  {} {into:>4.1} in / {off:>4.1} off, slides {slide:>4.1}, {turns};", pc.name());
        }
        say(row);
    }
    std::fs::write(docs.join("cup.txt"), lines.join("\n") + "\n").map_err(|e| e.to_string())?;

    // Drawn: the bent knee in the boot, the pad's lower parts and the cup, from the outside, the
    // front three-quarter and the front; then closer, the knee alone.
    let mut leg_m = clip_mesh_against_plane(&bent, Point3::new(0.0, mid, 0.0), -Vector3::y());
    leg_m = clip_mesh_against_plane(&leg_m, Point3::new(0.0, 0.0, 100.0), Vector3::z());
    leg_m = clip_mesh_against_plane(&leg_m, kb + thigh_dir * 330.0, -thigh_dir);
    let boot: Vec<(IndexedMesh, [f64; 3], f64)> = [
        ("shell.stl", [0.12, 0.12, 0.13], 0.35),
        ("tongue_plate.stl", [0.90, 0.90, 0.87], 0.0),
        ("laces.stl", [0.96, 0.96, 0.94], 0.0),
        ("eyelets.stl", [0.80, 0.81, 0.84], 0.5),
        ("tendon_guard.stl", [0.10, 0.10, 0.11], 0.0),
    ]
    .into_iter()
    .filter_map(|(n, c, s)| load(n).ok().map(|m| (m, c, s)))
    .collect();
    let skin = [0.85, 0.70, 0.60];
    let shown = |pt: &Part| !matches!(pt, Part::WingFoam(_) | Part::KneeLiner | Part::ThighFoam);
    let mut ls = vec![Layer::solid(&leg_m, skin)];
    ls.extend(boot.iter().map(|(m, c, s)| Layer::solid(m, *c).with_spec(*s)));
    for (pt, m) in lower.iter().filter(|(pt, _)| shown(pt)) {
        ls.push(Layer::solid(m, pt.color()).with_spec(if pt.hard() { 0.3 } else { 0.0 }));
    }
    for (pt, m, _) in parts.iter().filter(|(pt, ..)| shown(pt)) {
        ls.push(Layer::solid(m, pt.color()).with_spec(0.3));
    }
    let views = [
        View::looking(Vector3::y(), Vector3::z()),
        View::looking(Vector3::new(-0.70, 0.55, -0.25), Vector3::z()),
        View::looking(-Vector3::x(), Vector3::z()),
    ];
    let png = docs.join("cup.png");
    draw(&ls, &views, 1.2, &png, Style::pretty())?;
    // Closer: the knee from 160 mm below the joint, the lower parts cut there too.
    let cut = |m: &IndexedMesh| clip_mesh_against_plane(m, Point3::new(0.0, 0.0, kb.z - 160.0), Vector3::z());
    let leg_c = cut(&leg_m);
    let lower_c: Vec<(Part, IndexedMesh)> = lower.iter().filter(|(pt, _)| shown(pt)).map(|(pt, m)| (*pt, cut(m))).collect();
    let mut lc = vec![Layer::solid(&leg_c, skin)];
    for (pt, m) in &lower_c {
        lc.push(Layer::solid(m, pt.color()).with_spec(if pt.hard() { 0.3 } else { 0.0 }));
    }
    for (pt, m, _) in parts.iter().filter(|(pt, ..)| shown(pt)) {
        lc.push(Layer::solid(m, pt.color()).with_spec(0.3));
    }
    let close = docs.join("cup_close.png");
    draw(&lc, &views, 2.5, &close, Style::pretty())?;
    println!("drew {} and {}; wrote {} ({:.0} s in all)", png.display(), close.display(), docs.join("cup.txt").display(), t0.elapsed().as_secs_f64());
    Ok(())
}
