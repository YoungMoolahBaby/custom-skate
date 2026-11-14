//! G3, round 2: the knee shield as a cup on the bent knee.
//!
//! Jon, on round 1's `gap.png` (2026-11-12): "the knee part is usually more of a cup, as hockey
//! players usually are always skating with a knee bend, and wearing socks, or in roller hockey,
//! pants". So the shin pad's knee pieces (the knee shield with its liner, the thigh guard with its
//! foam) are built on the body bent to the skating knee (`KNEE_BUILD`, RESEARCH.md 23):
//! - the stand-in body (`body.rs`) posed by MakeHuman's skeleton and skin weights, with a kneecap
//!   added (`pose::Patella`: MakeHuman has none, and without it the skin over the knee followed
//!   the thigh), read into the shin's frame and leaned like the boot, so the pad's lower parts
//!   (`pad` run, on the leg) meet the cup as before: below the knee the bent body is the standing
//!   one;
//! - each part an offset from that bent skin, with the pad's own layers (`pad::Pad::field`);
//! - the outlines stay those drawn on the standing leg (`pad::Outlines`): the skin carries them to
//!   the bent knee (`pad::Warp`), each point near the bent skin reading the angle round the leg
//!   and the height that the closest skin point had standing.
//!
//! Two hard shells over foam in the suit (G3, round 4, 2026-11-13). A knee cap over the
//! kneecap, flex-hinged to the front shell at its bottom (Jon's pads: "like a flex hinge"); and
//! a top on the thigh, with the thigh guard, its upper edge just below the girdle ("when its flat
//! it just goes like right below the girdle, but still full covers the knee when bent"). The
//! kneecap travels ~60 mm along the thigh between bent and straight, so the cap slides under the
//! top's lower part rather than dragging the top up the thigh (round 3, hinged, dragged it 64
//! mm). The foam under both is in a full-length compression suit with the girdle, not in either
//! shell, so it stays on the skin wherever the shells slide.
//!
//! Worn, the sock (ice) or pants (roller) hold the shells on. At any other pose each rests on the
//! skin under it (`fit`), the cap held to the front shell by its hinge (`place`): a rigid motion
//! where the skin pressing into the foam costs far more than the foam standing off it (the
//! sock's pull). What is left says how each sits (pressing into the foam, or standing off it),
//! what the hinge must do (turn, give), and how far the cap slides under the top.

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

/// Where the cap ends, mm above the knee joint on the standing leg: above the kneecap (its top is
/// ~40 mm up: mid-patella 20 mm above the joint, ANSUR; 40 mm long, RESEARCH.md 24), as high as
/// leaves the cap room to slide its 64 mm up under the top when straight, with 5 to spare.
pub const CAP_TOP: f64 = 52.0;
/// Where the top's lower edge reaches down to over the cap, mm above the knee joint: 77 mm of
/// overlap at the stance. The cap slides down out from under the top 71 mm by a deep crouch
/// (knee 135, hip 60; with 50 mm of overlap a 17 mm gap opened there; with 72, 0.7 mm was left:
/// cup runs, 2026-11-13).
pub const TOP_BOTTOM: f64 = -25.0;
/// The top's upper edge (the thigh guard's), mm above the knee joint: just below the girdle's
/// thigh pads (600 mm up, 134 above the joint). At 121 the closest it came to them was 8.4 mm
/// (gap run, 2026-11-13).
pub const THIGH_TOP: f64 = 125.0;

/// How far round the knee the cup's hard shell reaches, degrees from the front: to the sides of
/// the knee, as on the Bauer Vapor pad in the photo Jon gave as the reference for hard shell
/// coverage (2026-11-13). The knee outline alone wraps ~115 deg at the joint, and that wing's tip
/// was where the skin pressed into the cup when straight or crouched.
pub const CUP_WRAP: f64 = 90.0;

/// The cup's layers: the pad's, the shield in two sliding shells and cut at the sides.
pub fn cup_params() -> PadParams {
    PadParams { knee_lap: Some((CAP_TOP, TOP_BOTTOM)), knee_wrap: Some(CUP_WRAP), ..PadParams::default() }
}

/// The pad's outlines for the cup: the thigh guard reaching up to `THIGH_TOP`.
pub fn outlines(kz: f64) -> Outlines {
    Outlines::new(kz).with_thigh_top(kz, THIGH_TOP)
}

/// The cup's two hard pieces:
/// - the cap over the kneecap, hinged to the front shell at its bottom;
/// - the top on the thigh, the thigh guard part of it, lapping over the cap's upper part.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Piece {
    Cap,
    Top,
}

impl Piece {
    pub const ALL: [Piece; 2] = [Piece::Cap, Piece::Top];

    pub fn parts(&self) -> &'static [Part] {
        match self {
            Piece::Cap => &[Part::KneeCap],
            Piece::Top => &[Part::KneeTop, Part::ThighGuard],
        }
    }

    pub fn of(part: Part) -> Option<Piece> {
        Piece::ALL.into_iter().find(|p| p.parts().contains(&part))
    }

    pub fn index(&self) -> usize {
        match self {
            Piece::Cap => 0,
            Piece::Top => 1,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Piece::Cap => "knee cap",
            Piece::Top => "top",
        }
    }
}

/// The carried outlines' blend round the skin: sigma this times the distance from it (`pad::Warp`).
pub const WARP_SMOOTH: f64 = 0.6;

/// The foam under the shells, in the compression suit: it rides the skin, not either shell.
pub const SUIT_PARTS: [Part; 2] = [Part::KneeLiner, Part::ThighFoam];

pub const CUP_PARTS: [Part; 5] = [Part::KneeCap, Part::KneeTop, Part::ThighGuard, Part::KneeLiner, Part::ThighFoam];

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

/// How the cup's pieces are held: the skin each rests on, the cap's flex hinge to the front shell
/// and its top edge under the top, as points at the middle of the hard shell, in the build pose
/// (the standing frame).
pub struct Hold {
    /// Per piece (`Piece::index`), the right leg's skin vertices under it: the cap's outside the
    /// part of it that laps the front shell (that part sits on the shell, not the skin).
    pub anchors: [Vec<usize>; 2],
    /// The lower hinge, the cap's bottom on the front shell's top edge: its middle
    /// (`HINGE_HALF` each side of the front), tied.
    pub lower: Vec<Point3<f64>>,
    /// The cap's top edge's middle, where it slides under the top (tied only in `place_with`'s
    /// what-ifs; round 3 had a flex hinge there).
    pub upper: Vec<Point3<f64>>,
    /// Per tied point (`lower`, `upper`), in the build pose: across the hinge toward the piece
    /// above it (the cap for the lower hinge, the top for the upper), along the surface; and the
    /// surface's outward normal. To read a hinge's give (`give`).
    pub lower_dirs: Vec<(Vector3<f64>, Vector3<f64>)>,
    pub upper_dirs: Vec<(Vector3<f64>, Vector3<f64>)>,
}

/// Half the width of a hinge's tied middle, mm of arc round the leg. A hinge turns about a line;
/// tied along their whole curved edges (round the shin, round the knee) the pieces couldn't turn
/// at all, and soft ties there stretched 12-27 mm (the first try, 2026-11-13).
pub const HINGE_HALF: f64 = 20.0;

/// The skin and hinge points for the cup, from the body in the build pose (`build`, the
/// standing frame) and its normals.
pub fn hold(b: &Body, ll: &Leaned, o: &Outlines, p: &PadParams, build: &IndexedMesh, n_build: &[Vector3<f64>]) -> Hold {
    let mid = b.shift.1;
    let (kz, h) = (ll.knee.z, p.knee_lap.map_or(p.knee_hinge.unwrap_or(CAP_TOP), |l| l.0));
    let mut out = Hold {
        anchors: [Vec::new(), Vec::new()],
        lower: Vec::new(),
        upper: Vec::new(),
        lower_dirs: Vec::new(),
        upper_dirs: Vec::new(),
    };
    let mut normal_at = [Vec::new(), Vec::new()];
    for (i, v) in b.mesh.vertices.iter().enumerate() {
        if v.y >= mid {
            continue;
        }
        let (th, s, z) = standing_tsz(ll, *v);
        let knee = pad::knee_eval(o, p, s, z);
        let front = o.front.eval(s, z);
        let under_shell = front < 0.0 || o.side.eval(s.abs(), z) < 0.0;
        if knee < 0.0 && z < kz + h && !under_shell {
            out.anchors[0].push(i);
        }
        if (knee < 0.0 && z >= kz + h) || o.thigh.eval(s, z) < 0.0 {
            out.anchors[1].push(i);
        }
        // The hinges' points at the middle of the hard shell over this skin point.
        if knee < -3.0 && (z - (kz + h)).abs() < 2.0 {
            let pt = build.vertices[i] + n_build[i] * (p.ease + p.knee_liner + pad::dome(p, kz, s, z) + 0.5 * p.wall);
            if s.abs() <= HINGE_HALF {
                out.upper.push(pt);
                normal_at[1].push(n_build[i]);
            }
        }
        if knee < -3.0 && front.abs() < 2.0 {
            let pt = build.vertices[i] + n_build[i] * (p.ease + pad::liner_at(p, th) + p.wall + pad::LAP_CLEAR + 0.5 * p.wall);
            if s.abs() <= HINGE_HALF {
                out.lower.push(pt);
                normal_at[0].push(n_build[i]);
            }
        }
    }
    // Across each hinge: toward the middle of the skin under the piece above it.
    let centre = |idx: &[usize]| idx.iter().fold(Vector3::zeros(), |s, &i| s + build.vertices[i].coords) / idx.len().max(1) as f64;
    let dirs = |pts: &[Point3<f64>], ns: &[Vector3<f64>], toward: Vector3<f64>| -> Vec<(Vector3<f64>, Vector3<f64>)> {
        pts.iter()
            .zip(ns)
            .map(|(pt, n)| {
                let d = toward - pt.coords;
                ((d - n * n.dot(&d)).try_normalize(1e-9).unwrap_or_else(Vector3::zeros), *n)
            })
            .collect()
    };
    out.lower_dirs = dirs(&out.lower, &normal_at[0], centre(&out.anchors[0]));
    out.upper_dirs = dirs(&out.upper, &normal_at[1], centre(&out.anchors[1]));
    out
}

/// How a hinge's tied middle gives at a pose, mm, the most of each over its points: the piece
/// above it (`a`) against the one below (`b`), read in the build pose's frame of the one below.
/// Opening: its edge moving away across the hinge (a strip there stretches); closing: moving in
/// over the one below (it laps further, or the edges meet); lift: off the surface; sideways:
/// along the hinge line.
#[derive(Clone, Copy, Debug, Default)]
pub struct Give {
    pub open: f64,
    pub close: f64,
    pub lift: f64,
    pub side: f64,
}

pub fn give(points: &[Point3<f64>], dirs: &[(Vector3<f64>, Vector3<f64>)], a: &Motion, b: &Motion) -> Give {
    let rel = b.inverse().after(a);
    let mut g = Give::default();
    for (p, (u, n)) in points.iter().zip(dirs) {
        let e = rel.apply(p.coords) - p.coords;
        let (eu, en) = (e.dot(u), e.dot(n));
        g.open = g.open.max(eu);
        g.close = g.close.max(-eu);
        g.lift = g.lift.max(en.abs());
        g.side = g.side.max((e - u * eu - n * en).norm());
    }
    g
}

/// How firmly a flex hinge holds the two edges it joins together, per point of its tied middle,
/// against the sock's pull of 1 per skin point: it turns, it doesn't stretch.
pub const HINGE_HOLD: f64 = 500.0;

/// d(x)/d(turn w, shift t) for a motion's small step about `c`: [-[x - c]x, I].
fn jac(x: Vector3<f64>, c: Vector3<f64>) -> nalgebra::Matrix3x6<f64> {
    let mut j = nalgebra::Matrix3x6::<f64>::zeros();
    j.fixed_view_mut::<3, 3>(0, 0).copy_from(&(-(x - c).cross_matrix()));
    j.fixed_view_mut::<3, 3>(0, 3).copy_from(&Matrix3::identity());
    j
}

/// A small step (turn w, shift t) about `c`, after `m`.
fn step(m: &Motion, d: &[f64], c: Vector3<f64>) -> Motion {
    let r = Rotation3::new(Vector3::new(d[0], d[1], d[2]));
    Motion { r, t: c - r * c + Vector3::new(d[3], d[4], d[5]) }.after(m)
}

/// The Gauss-Newton terms of a piece resting on its skin (see `fit`): skin points `a` (build
/// pose) going to `b`, normals per vertex `n_to` by index `idx`.
fn skin_terms(m: &Motion, idx: &[usize], a: &[Point3<f64>], b: &[Point3<f64>], n_to: &[Vector3<f64>], c: Vector3<f64>) -> (nalgebra::Matrix6<f64>, nalgebra::Vector6<f64>) {
    let mut h = nalgebra::Matrix6::<f64>::zeros();
    let mut g = nalgebra::Vector6::<f64>::zeros();
    for (k, &i) in idx.iter().enumerate() {
        let x = m.apply(a[k].coords);
        let e = x - b[k].coords;
        let n = n_to[i];
        let j = jac(x, c);
        // Past the piece's inside: the skin is further out than the piece carries it.
        let s = n.dot(&e);
        let wn = if s < 0.0 { PUSH_OVER_PULL } else { 1.0 };
        let jn = n.transpose() * j;
        let p = Matrix3::identity() - n * n.transpose();
        let jt = p * j;
        h += jn.transpose() * jn * wn + jt.transpose() * jt * HOLD;
        g += jn.transpose() * (s * wn) + jt.transpose() * (p * e) * HOLD;
    }
    (h, g)
}

/// The cup's pieces at a pose (`Piece::index` order), from the build pose, both in the standing
/// frame, solved together: each rests on its skin (as `fit`), the cap's bottom held to the front
/// shell (`shell`: the shin's motion from the build pose) at the hinge's tied middle; the top
/// slides on the cap, untied. Gauss-Newton on both pieces' small turns and shifts, from each
/// one's plain fit (Kabsch).
pub fn place(h: &Hold, build: &IndexedMesh, q: &IndexedMesh, n_q: &[Vector3<f64>], shell: &Motion) -> [Motion; 2] {
    place_with(h, build, q, n_q, shell, [HINGE_HOLD, 0.0])
}

/// `place` with the lower hinge and the cap-to-top tie holding by `hold` per tied point (round 3
/// had a flex hinge between the cap and the top; untied, 0, they slide).
pub fn place_with(h: &Hold, build: &IndexedMesh, q: &IndexedMesh, n_q: &[Vector3<f64>], shell: &Motion, hold: [f64; 2]) -> [Motion; 2] {
    let pts = |k: usize| -> (Vec<Point3<f64>>, Vec<Point3<f64>>) {
        (h.anchors[k].iter().map(|&i| build.vertices[i]).collect(), h.anchors[k].iter().map(|&i| q.vertices[i]).collect())
    };
    let (ab, bb) = (pts(0), pts(1));
    let mut ms = [kabsch(&ab.0, &ab.1), kabsch(&bb.0, &bb.1)];
    let centre = |v: &[Point3<f64>]| v.iter().fold(Vector3::zeros(), |s, p| s + p.coords) / v.len().max(1) as f64;
    let cs = [centre(&ab.1), centre(&bb.1)];
    let low: Vec<(Vector3<f64>, Vector3<f64>)> = h.lower.iter().map(|p| (p.coords, shell.apply(p.coords))).collect();
    for _ in 0..60 {
        let mut hh = nalgebra::SMatrix::<f64, 12, 12>::zeros();
        let mut gg = nalgebra::SVector::<f64, 12>::zeros();
        for (k, (a, b)) in [&ab, &bb].into_iter().enumerate() {
            let (hk, gk) = skin_terms(&ms[k], &h.anchors[k], a, b, n_q, cs[k]);
            hh.fixed_view_mut::<6, 6>(6 * k, 6 * k).copy_from(&hk);
            gg.fixed_rows_mut::<6>(6 * k).copy_from(&gk);
        }
        for (p, target) in &low {
            let x = ms[0].apply(*p);
            let j = jac(x, cs[0]);
            let mut b = hh.fixed_view_mut::<6, 6>(0, 0);
            b += j.transpose() * j * hold[0];
            let mut g = gg.fixed_rows_mut::<6>(0);
            g += j.transpose() * (x - target) * hold[0];
        }
        for p in &h.upper {
            let (xc, xt) = (ms[0].apply(p.coords), ms[1].apply(p.coords));
            let (jc, jt) = (jac(xc, cs[0]), jac(xt, cs[1]));
            let r = xc - xt;
            let mut b = hh.fixed_view_mut::<6, 6>(0, 0);
            b += jc.transpose() * jc * hold[1];
            let mut b = hh.fixed_view_mut::<6, 6>(6, 6);
            b += jt.transpose() * jt * hold[1];
            let mut b = hh.fixed_view_mut::<6, 6>(0, 6);
            b -= jc.transpose() * jt * hold[1];
            let mut b = hh.fixed_view_mut::<6, 6>(6, 0);
            b -= jt.transpose() * jc * hold[1];
            let mut g = gg.fixed_rows_mut::<6>(0);
            g += jc.transpose() * r * hold[1];
            let mut g = gg.fixed_rows_mut::<6>(6);
            g -= jt.transpose() * r * hold[1];
        }
        let Some(d) = hh.lu().solve(&(-gg)) else { break };
        ms[0] = step(&ms[0], &d.as_slice()[0..6], cs[0]);
        ms[1] = step(&ms[1], &d.as_slice()[6..12], cs[1]);
        if d.norm() < 1e-7 {
            break;
        }
    }
    ms
}

/// A hinge at a pose: how far its two sides turn on each other (deg) and the most it has to
/// stretch (mm), from the build pose.
pub fn hinge(points: &[Point3<f64>], a: &Motion, b: &Motion) -> (f64, f64) {
    let stretch = points.iter().map(|p| (a.apply(p.coords) - b.apply(p.coords)).norm()).fold(0.0, f64::max);
    (turn_deg(&b.inverse().after(a).r), stretch)
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
/// along the skin; and per tie (a point on the piece, where it is held), `HINGE_HOLD` times its
/// offset.
pub fn fit(idx: &[usize], from: &IndexedMesh, to: &IndexedMesh, n_to: &[Vector3<f64>], ties: &[(Point3<f64>, Point3<f64>)]) -> Motion {
    let a: Vec<Point3<f64>> = idx.iter().map(|&i| from.vertices[i]).collect();
    let b: Vec<Point3<f64>> = idx.iter().map(|&i| to.vertices[i]).collect();
    let mut m = kabsch(&a, &b);
    let c = b.iter().fold(Vector3::zeros(), |s, p| s + p.coords) / b.len().max(1) as f64;
    for _ in 0..40 {
        let (mut h, mut g) = skin_terms(&m, idx, &a, &b, n_to, c);
        for (a, target) in ties {
            let x = m.apply(a.coords);
            let j = jac(x, c);
            h += j.transpose() * j * HINGE_HOLD;
            g += j.transpose() * (x - target.coords) * HINGE_HOLD;
        }
        let Some(d) = h.lu().solve(&(-g)) else { break };
        m = step(&m, d.as_slice(), c);
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
    let p = cup_params();
    let top = boot_top(&shell, &ll.f);
    let z_bot = Ring(top.0.iter().map(|z| z + p.clear_boot).collect());
    let o = Arc::new(outlines(ll.knee.z));
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
        "the cup (G3, round 4: two hard shells sliding over foam in the suit): built on the stand-in body ({}) bent to hip {HIP_BUILD}, knee {KNEE_BUILD} deg, in the shin's frame leaned {} deg like the boot",
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
        .filter(|pt| !CUP_PARTS.contains(pt))
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
            (Part::KneeTop, knee_box),
            (Part::ThighGuard, thigh_box),
            (Part::ThighFoam, thigh_box),
        ])),
        over: None,
    };
    let t2 = Instant::now();
    // The cap and the rest first; then the top, lapping over the cap as built (the cap's own lap
    // over the front shell pushes it out a little where the top's offset alone doesn't know: it
    // went 1.35 mm into the top there, 2026-11-13).
    let first: Vec<Part> = CUP_PARTS.into_iter().filter(|pt| *pt != Part::KneeTop).collect();
    let mut parts = pad::build_these(&cup, &first);
    let cap_mesh = parts.iter().find(|x| x.0 == Part::KneeCap).ok_or("no knee cap built")?.1.clone();
    let over = Pad { over: Some(Arc::new(NearMesh::new(&cap_mesh, "knee cap")?)), ..cup.clone() };
    parts.extend(pad::build_these(&over, &[Part::KneeTop]));
    say(format!("built {} parts ({:.0} s)", parts.len(), t2.elapsed().as_secs_f64()));

    // Against the straight pieces (the `pad` run's flush pad, built on the standing leg).
    say(format!(
        "weights: the knee cap (up to {CAP_TOP} mm above the knee joint) and the top (down over the cap to {TOP_BOTTOM} mm, {} mm off it; the thigh guard part of it, up to {THIGH_TOP} mm), cut {CUP_WRAP} deg round; and the foam under them, in the suit; against the straight pad's pieces (same layers, PA 12):",
        p.slide_clear
    ));
    let g_of = |pt: Part| parts.iter().find(|x| x.0 == pt).map_or(0.0, |x| x.2);
    let straight_g = |pt: Part| load_mesh(out.join(flush_name(pt))).map(|s| grams(&s, pt)).unwrap_or(f64::NAN);
    let (mut tc, mut ts) = (0.0, 0.0);
    for (label, cup_parts, straight) in [
        ("hard shells", vec![Part::KneeCap, Part::KneeTop, Part::ThighGuard], vec![Part::KneeCap, Part::ThighGuard]),
        ("suit foam", vec![Part::KneeLiner, Part::ThighFoam], vec![Part::KneeLiner, Part::ThighFoam]),
    ] {
        let (gc, gs) = (cup_parts.iter().map(|pt| g_of(*pt)).sum::<f64>(), straight.iter().map(|pt| straight_g(*pt)).sum::<f64>());
        tc += gc;
        ts += gs;
        let split = cup_parts.iter().map(|pt| format!("{} {:.1}", pt.name(), g_of(*pt))).collect::<Vec<_>>().join(", ");
        say(format!("  {label:<12} {gc:>6.1} g (straight {gs:>5.1} g): {split}"));
    }
    say(format!("  {:<12} {:>6.1} g (straight {:>5.1} g)", "the cup", tc, ts));
    for (part, m, _) in &parts {
        save_mesh(m, out.join(stl_name(*part))).map_err(|e| e.to_string())?;
    }

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
    // The parts it laps, against the cup's knee cap and (for comparison) the straight shield.
    let straight = load_mesh(out.join(flush_name(Part::KneeCap))).map_err(|e| e.to_string())?;
    let shields = [("cup", fields.iter().find(|(pt, _)| *pt == Part::KneeCap).unwrap().1.clone()), ("straight", NearMesh::new(&straight, "straight shield")?)];
    for (pt, m) in lower.iter().filter(|(pt, _)| lapped.contains(pt)) {
        let mut row = format!("  the {} to the knee cap:", pt.name());
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
    // The top over the cap, as built: how close the cap comes to the top (`slide_clear` off it).
    let top_f = fields.iter().find(|(pt, _)| *pt == Part::KneeTop).ok_or("no knee top")?.1.clone();
    let cap_m = &parts.iter().find(|x| x.0 == Part::KneeCap).ok_or("no knee cap")?.1;
    let top_clear = |rel: &Motion| -> f64 {
        // The cap's vertices moved by `rel` (the standing frame, from the build pose), read back in
        // the shin's frame where the top's mesh is.
        cap_m.vertices.iter().step_by(7).map(|v| top_f.eval(in_frame(Point3::from(rel.apply(mv[shin].apply(ll.unlean(*v).coords)))))).fold(f64::MAX, f64::min)
    };
    say(format!("  the knee cap to the top over it: {:.2} mm at the closest", top_clear(&Motion::identity())));

    // How it sits through the knee's range: each piece resting on the skin under it, the cap
    // held to the front shell by its hinge, the top sliding on the cap (`place`).
    let fs = front_of_shin(&b);
    let hd = hold(&b, &ll, &o, &p, &pm, &normals(&pm, fs));
    let (overlap, headroom) = (CAP_TOP - TOP_BOTTOM, THIGH_TOP - CAP_TOP);
    say(format!(
        "through the knee's range (hip {HIP_BUILD}): the knee cap and the top each resting on the skin under them ({} and {} points; the skin pushing into the foam {PUSH_OVER_PULL} times as hard as the sock pulls where it stands off), the cap held to the front shell at its bottom by a flex hinge tied along its middle ({HINGE_HALF} mm each side of the front: {} points, {HINGE_HOLD} each), the top sliding on the cap. Per piece: the skin into the foam / standing off it, most, mm; the hinge: how far it turns, deg, and how its middle gives, mm (opening, closing, lifting off, sideways); the cap's top edge under the top (its middle, {} points): how far it slides up under the top and down out of it, mm, the overlap left of the {overlap:.0} mm built (none: a gap opens), the room left below the top's upper edge ({headroom:.0} mm built), and how close the two shells come (negative: the cap pushes into the top):",
        hd.anchors[0].len(),
        hd.anchors[1].len(),
        hd.lower.len(),
        hd.upper.len()
    ));
    let b_from = pm;
    let mut range = Vec::new();
    for knee in [0.0, 15.0, 30.0, 45.0, 60.0, 70.0, 90.0, 105.0, 120.0, 135.0] {
        let q = LegPose { hip: HIP_BUILD, knee, ankle: 0.0 };
        let (qm, _, qv) = posed(&b, q)?;
        let n = normals(&qm, fs);
        let shell_m = qv[shin].after(&mv[shin].inverse());
        let ms = place(&hd, &b_from, &qm, &n, &shell_m);
        let mut row = format!("  knee {knee:>4.0}:");
        for pc in Piece::ALL {
            let (into, off, _) = sit(&hd.anchors[pc.index()], &b_from, &qm, &ms[pc.index()], &n);
            row += &format!("  {} {into:>4.1} in / {off:>4.1} off;", pc.name());
        }
        let (lt, _) = hinge(&hd.lower, &ms[0], &shell_m);
        let lg = give(&hd.lower, &hd.lower_dirs, &ms[0], &shell_m);
        // The cap's top edge in the top's frame: up the thigh (into the top) is `open`.
        let sl = give(&hd.upper, &hd.upper_dirs, &ms[0], &ms[1]);
        let clear = top_clear(&ms[1].inverse().after(&ms[0]));
        row += &format!(
            "  hinge {lt:>4.1} deg, open {:>4.1} close {:>4.1} lift {:>4.1} side {:>4.1};  the cap under the top: up {:>4.1}, down {:>4.1}, overlap {:>5.1}, room {:>5.1}, shells {clear:>5.1}",
            lg.open,
            lg.close,
            lg.lift,
            lg.side,
            sl.open,
            sl.close,
            overlap - sl.close,
            headroom - sl.open
        );
        say(row);
        range.push((qm, n, shell_m));
    }
    // The hinges from soft to firm: the hold isn't a measured stiffness (the skin's push and the
    // sock's pull are relative weights too), so the trade-off it sets is shown whole.
    say(format!("the cap's flex hinge from soft to firm (hold per tied point; the default above is the {HINGE_HOLD} row): over the range above, the most the skin presses into the foam under each shell, mm, the most the hinge's middle gives, and the most the cap slides up under the top and down out of it, mm:"));
    for hold_k in [0.0, 5.0, 20.0, 100.0, 500.0, 5000.0] {
        let (mut ins, mut lw, mut sw) = ([0.0f64; 2], Give::default(), Give::default());
        for (qm, n, shell_m) in &range {
            let ms = place_with(&hd, &b_from, qm, n, shell_m, [hold_k, 0.0]);
            for pc in Piece::ALL {
                ins[pc.index()] = ins[pc.index()].max(sit(&hd.anchors[pc.index()], &b_from, qm, &ms[pc.index()], n).0);
            }
            for (w, g) in [(&mut lw, give(&hd.lower, &hd.lower_dirs, &ms[0], shell_m)), (&mut sw, give(&hd.upper, &hd.upper_dirs, &ms[0], &ms[1]))] {
                *w = Give { open: w.open.max(g.open), close: w.close.max(g.close), lift: w.lift.max(g.lift), side: w.side.max(g.side) };
            }
        }
        say(format!(
            "  hold {hold_k:>6.0}: skin into the cap's foam {:>4.1}, the top's {:>4.1};  hinge open {:>4.1} close {:>4.1} lift {:>4.1} side {:>4.1};  the cap up under the top {:>4.1}, down out of it {:>4.1}",
            ins[0], ins[1], lw.open, lw.close, lw.lift, lw.side, sw.open, sw.close
        ));
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
