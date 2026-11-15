//! U2, the elbow pad's first step (2026-11-14): what the elbow and the forearm do under a pad and
//! a slash guard, on the stand-in arm (U1, fitted to ANSUR II).
//!
//! - The elbow bent from straight to its full flexion (the forearm palm in, as U2 holds it): how
//!   far the back of the elbow stands behind the joint, and how much the skin over the back of
//!   the elbow lengthens between marks on the upper arm and the forearm. An elbow pad's cap rides
//!   there, as the knee's does (G3). Read twice: on MakeHuman's elbow, whose back is blended skin
//!   and pulls in as it bends, and with an olecranon added (`add_olecranon`), as G3 added a
//!   kneecap.
//! - The forearm turned at the elbow's 90 deg, from palm in to its full pronation and supination:
//!   how far the skin along the back of the forearm turns about the forearm's axis, which says
//!   where a rigid plate (the slash guard) can ride.
//!
//! The ranges: CDC's Joint Range of Motion Study, men 20-44 (RESEARCH.md 29).
//! Frame: the foot's (x forward, y left, z up, mm).

use std::path::Path;
use std::time::Instant;

use cortenforge::cf_scan_prep_core::clip_mesh_against_plane;
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Rotation3, Vector3};

use crate::body::{self, Body};
use crate::glove::{quad_mesh, section_lines};
use crate::pad::{draw, smoothstep};
use crate::pose::Motion;
use crate::render::{Layer, Style, View};
use crate::upper::{self, Pose};

/// Men 20-44, mean, deg (CDC; RESEARCH.md 29).
pub const ELBOW_FLEXION: f64 = 144.6;
pub const PRONATION: f64 = 76.9;
pub const SUPINATION: f64 = 85.0;

/// The olecranon's size: its tip to the ulna's proximal edge, 24.7 mm (Puchwein et al. 2012, CT
/// of 40 elbows, mean; RESEARCH.md 29). The skin over it moves with it all within half that of the
/// skin straight behind the elbow joint (the arm straight), none past it, smoothly between (as the
/// kneecap's, `body::kneecap`); and none more than `OLECRANON_ABOVE` above the joint's level: the
/// olecranon's tip sits a little above the joint's axis with the arm straight, and above it the
/// skin lies over the triceps' tendon, which stays with the upper arm. (Centred first on the
/// skin standing furthest back within 15 mm of the joint's level, it landed on the triceps 13 mm
/// above it, and only 2 of the skin's control vertices moved with it.)
pub const OLECRANON: f64 = 24.7;
/// Assumed, not found in a source.
pub const OLECRANON_ABOVE: f64 = 10.0;

/// The elbow's sweep, deg.
const BENDS: [f64; 6] = [0.0, 30.0, 60.0, 90.0, 120.0, ELBOW_FLEXION];
/// The marks on the skin over the back of the elbow: this far from the joint along the upper arm
/// and along the forearm, mm.
const MARK: f64 = 80.0;
/// Where along the forearm the twist is read, as fractions of elbow joint to wrist joint.
const ALONG: [f64; 5] = [0.1, 0.25, 0.5, 0.75, 0.9];

/// The smoothed skin (mm) and the bones' motions in a pose.
pub(crate) fn skin_in(b: &Body, raw: &[Vector3<f64>], pose: &Pose) -> (Vec<Point3<f64>>, Vec<Motion>) {
    let m = &b.model;
    let mo = m.rig.motions(raw, pose);
    let praw = m.rig.skin(raw, &mo);
    let skin = m.s2.apply(&m.s1.apply(&praw[..m.n_body])).iter().map(|&v| body::to_mm(v, b.floor, b.shift)).collect();
    (skin, mo)
}

/// Adds an olecranon to the right arm (`pose::Rig::add_olecranon`), its skin as `OLECRANON` says;
/// returns how many of the skin's control vertices move with it, and how many fully.
pub(crate) fn add_olecranon(b: &mut Body, raw: &[Vector3<f64>]) -> Result<(usize, usize), String> {
    let m = &b.model;
    let rig = &m.rig;
    let j = |name: &str| rig.joint(raw, name).ok_or(format!("no joint {name}"));
    let (el, sh, wr) = (j("lowerarm01.R____head")?, j("upperarm01.R____head")?, j("wrist.R____head")?);
    // Down the arm, and its back (MakeHuman's frame: z forward), in the rest pose.
    let axis = ((wr - el).normalize() - (sh - el).normalize()).normalize();
    let back = {
        let z = -Vector3::z();
        (z - axis * z.dot(&axis)).normalize()
    };
    let side = axis.cross(&back);
    let skin = m.s2.apply(&m.s1.apply(&raw[..m.n_body]));
    let point = skin
        .iter()
        .filter(|v| {
            let d = *v - el;
            d.dot(&axis).abs() < 0.06 && d.dot(&side).abs() < 0.10
        })
        .max_by(|a, c| (*a - el).dot(&back).total_cmp(&(*c - el).dot(&back)))
        .copied()
        .ok_or("no skin behind the elbow")?;
    let share: Vec<(u32, f64)> = raw[..m.n_body]
        .iter()
        .enumerate()
        .filter_map(|(i, v)| {
            let d = v - el;
            let a = 1.0 - smoothstep(0.5 * OLECRANON, OLECRANON, 100.0 * (v - point).norm());
            (d.dot(&back) > 0.0 && d.dot(&axis) > -OLECRANON_ABOVE / 100.0 && a > 0.0).then_some((i as u32, a))
        })
        .collect();
    let full = share.iter().filter(|s| s.1 >= 1.0).count();
    let n = share.len();
    if std::env::var("ELBOW_DEBUG").is_ok() {
        let dp = point - el;
        eprintln!("olecranon point: along {:.1} back {:.1} side {:.1} mm", 100.0 * dp.dot(&axis), 100.0 * dp.dot(&back), 100.0 * dp.dot(&side));
        for r in [10.0, 25.0, 40.0, 60.0] {
            let all = raw[..m.n_body].iter().filter(|v| 100.0 * (*v - point).norm() < r).count();
            let behind = raw[..m.n_body].iter().filter(|v| 100.0 * (*v - point).norm() < r && (*v - el).dot(&back) > 0.0).count();
            eprintln!("  control vertices within {r} mm: {all}, behind the joint {behind}");
        }
        let nearest = raw[..m.n_body].iter().map(|v| 100.0 * (v - point).norm()).fold(f64::MAX, f64::min);
        eprintln!("  nearest control vertex {nearest:.1} mm");
    }
    std::sync::Arc::get_mut(&mut b.model).ok_or("the body's model is shared")?.rig.add_olecranon("R", &share)?;
    Ok((n, full))
}

/// How far along a polyline from `from` (by index, either way) until `hit` turns true, mm; None
/// if it ends first. Interpolated at the crossing on `g` (negative before it, positive after).
fn walk(pts: &[Point3<f64>], closed: bool, from: usize, step: isize, g: &dyn Fn(&Point3<f64>) -> f64, side: &dyn Fn(&Point3<f64>) -> bool) -> Option<f64> {
    let n = pts.len() as isize;
    let mut len = 0.0;
    let mut i = from as isize;
    for _ in 0..n {
        let j = i + step;
        let j = if closed { j.rem_euclid(n) } else if j < 0 || j >= n { return None } else { j };
        let (a, c) = (pts[i as usize], pts[j as usize]);
        let (ga, gc) = (g(&a), g(&c));
        if ga < 0.0 && gc >= 0.0 && side(&c) {
            return Some(len + (c - a).norm() * (-ga / (gc - ga)));
        }
        len += (c - a).norm();
        i = j;
    }
    None
}

/// The share of each smoothed skin vertex owned by some bones (a vertex between two counts for
/// the one with more of it), of those bones the rig has.
pub(crate) fn owned(b: &Body, bones: &[&str]) -> Vec<f64> {
    let m = &b.model;
    let dom = upper::dominant(b);
    let ids: Vec<usize> = bones.iter().filter_map(|n| m.rig.index.get(*n).copied()).collect();
    let vals: Vec<Vector3<f64>> = (0..m.n_body).map(|i| if ids.contains(&dom[i]) { Vector3::x() } else { Vector3::zeros() }).collect();
    m.s2.apply(&m.s1.apply(&vals)).iter().map(|v| v.x).collect()
}

pub(crate) const ARM_BONES: [&str; 5] = ["upperarm01.R", "upperarm02.R", "lowerarm01.R", "lowerarm02.R", "olecranon.R"];

/// One bend of the elbow: how far its back stands behind the joint, the skin's length over it.
struct Bend {
    angle: f64,
    back: f64,
    length: Option<f64>,
}

/// The elbow swept through `BENDS`, palm in: each bend, and for the picture the arm's far half and
/// its section in the plane it bends in (straight, 90 and full); the elbow's axis; up the arm.
struct Sweep {
    bends: Vec<Bend>,
    arms: Vec<IndexedMesh>,
    strips: Vec<IndexedMesh>,
    n: Vector3<f64>,
    up0: Vector3<f64>,
    uarm: f64,
}

fn sweep(b: &Body, raw: &[Vector3<f64>]) -> Sweep {
    let m = &b.model;
    let arm_share = owned(b, &ARM_BONES);
    let arm_quads: Vec<[u32; 4]> = m.quads.iter().copied().filter(|q| q.iter().all(|&i| arm_share[i as usize] > 0.5)).collect();
    let at = |mo: &[Motion], bone: &str| upper::posed_end(b, raw, mo, bone, false);
    // The elbow's bending axis from the forearm's swing; the back of the elbow along the bend's
    // bisector (straight: square to the arm, away from the bend).
    let poses: Vec<(Vec<Point3<f64>>, Vec<Motion>)> = BENDS.iter().map(|&a| skin_in(b, raw, &upper::forearm_pose(b, raw, a, false, 0.0))).collect();
    let fdir = |mo: &[Motion]| (at(mo, "wrist.R") - at(mo, "lowerarm01.R")).normalize();
    let f0 = fdir(&poses[0].1);
    let f90 = fdir(&poses[3].1);
    let n = f0.cross(&f90).normalize();
    let mut flex = n.cross(&f0).normalize();
    if flex.dot(&f90) < 0.0 {
        flex = -flex;
    }
    let mut bends = Vec::new();
    let mut strips: Vec<IndexedMesh> = Vec::new();
    let mut arms: Vec<IndexedMesh> = Vec::new();
    for (k, &angle) in BENDS.iter().enumerate() {
        let (skin, mo) = &poses[k];
        let el = at(mo, "lowerarm01.R");
        let up = (at(mo, "upperarm01.R") - el).normalize();
        let f = fdir(mo);
        let turn = Rotation3::rotation_between(&f0, &f).unwrap_or_else(Rotation3::identity);
        let back_dir = turn.powf(0.5) * -flex;
        let fore_back = turn * -flex;
        let back = (0..skin.len())
            .filter(|&i| arm_share[i] > 0.5)
            .map(|i| skin[i] - el)
            .filter(|d| d.dot(&n).abs() < 15.0 && (d - back_dir * d.dot(&back_dir) - n * d.dot(&n)).norm() < 15.0)
            .map(|d| d.dot(&back_dir))
            .fold(f64::MIN, f64::max);
        // The skin's length over the back of the elbow, in the plane the arm bends in: from the
        // back point, along the section both ways to the marks.
        let mesh = quad_mesh(skin, &arm_quads);
        let cs = crate::scan::section(&mesh, el, n);
        let target = el + back_dir * back;
        let near = |c: &cortenforge::mesh::measure::Contour| c.points.iter().map(|p| (p - target).norm()).fold(f64::MAX, f64::min);
        let length = cs.contours.iter().filter(|c| c.points.len() > 2).min_by(|a, c| near(a).total_cmp(&near(c))).and_then(|c| {
            let pts = &c.points;
            let closed = (pts[0] - pts[pts.len() - 1]).norm() < 1.0;
            let i0 = (0..pts.len()).min_by(|&a, &c| (pts[a] - target).norm().total_cmp(&(pts[c] - target).norm()))?;
            let g_up = |p: &Point3<f64>| (p - el).dot(&up) - MARK;
            let g_fore = |p: &Point3<f64>| (p - el).dot(&f) - MARK;
            let side_up = |p: &Point3<f64>| (p - el).dot(&-flex) > 0.0;
            let side_fore = |p: &Point3<f64>| (p - el).dot(&fore_back) > 0.0;
            let a = walk(pts, closed, i0, 1, &g_up, &side_up).zip(walk(pts, closed, i0, -1, &g_fore, &side_fore));
            let c2 = walk(pts, closed, i0, -1, &g_up, &side_up).zip(walk(pts, closed, i0, 1, &g_fore, &side_fore));
            a.or(c2).map(|(x, y)| x + y)
        });
        bends.push(Bend { angle, back, length });
        // For the picture (straight, 90 deg and full): the arm's far half from the plane it bends
        // in, so the section over the back of the elbow shows, the poses side by side.
        if let Some(slot) = [0usize, 3, 5].iter().position(|&x| x == k) {
            let shift = Motion { r: Rotation3::identity(), t: flex * (330.0 * slot as f64) - el.coords };
            arms.push(crate::glove::moved(&clip_mesh_against_plane(&mesh, el, -n), &shift));
            strips.push(crate::glove::moved(&section_lines(&mesh, el, n, 1.5), &shift));
        }
    }
    let up0 = (at(&poses[0].1, "upperarm01.R") - at(&poses[0].1, "lowerarm01.R")).normalize();
    let uarm = (at(&poses[0].1, "upperarm01.R") - at(&poses[0].1, "lowerarm01.R")).norm();
    Sweep { bends, arms, strips, n, up0, uarm }
}

/// `cargo run --release -- elbow <scan>`: the elbow pad's first step (docs/elbow/).
pub fn run(scan: &Path) -> Result<(), String> {
    let t0 = Instant::now();
    let docs = Path::new("docs/elbow");
    std::fs::create_dir_all(docs).map_err(|e| e.to_string())?;
    let mut b = body::build(scan, 1250.0)?;
    let (u, _) = upper::fit(&b)?;
    let raw = &u.raw;
    println!("  body fitted ({:.0} s)", t0.elapsed().as_secs_f64());
    let plain = sweep(&b, raw);
    let (ole_n, ole_full) = add_olecranon(&mut b, raw)?;
    let with = sweep(&b, raw);
    println!("  elbow swept, without and with an olecranon ({:.0} s)", t0.elapsed().as_secs_f64());
    let m = &b.model;
    let at = |mo: &[Motion], bone: &str| upper::posed_end(&b, raw, mo, bone, false);
    let fore_share = owned(&b, &["lowerarm01.R", "lowerarm02.R", "olecranon.R"]);
    let fore: Vec<usize> = (0..fore_share.len()).filter(|&i| fore_share[i] > 0.5).collect();

    // The forearm's turn, the elbow at 90 deg. Along the back of the forearm (the back of the
    // hand's side, palm in), the skin's turn about the forearm's axis at each station.
    let turns = [0.0, PRONATION, -SUPINATION];
    let twisted: Vec<(Vec<Point3<f64>>, Vec<Motion>)> = turns.iter().map(|&t| skin_in(&b, raw, &upper::forearm_pose(&b, raw, 90.0, false, t))).collect();
    let (skin0, mo0) = &twisted[0];
    let el0 = at(mo0, "lowerarm01.R");
    let wr0 = at(mo0, "wrist.R");
    let af0 = (wr0 - el0).normalize();
    let len0 = (wr0 - el0).norm();
    let flat = |v: Vector3<f64>, a: &Vector3<f64>| (v - a * v.dot(a)).normalize();
    let hand0 = upper::hand_frame(&b, raw, mo0);
    let dorsal = flat(-hand0.palm, &af0);
    let marks: Vec<usize> = ALONG
        .iter()
        .map(|&s| {
            fore.iter()
                .copied()
                .filter(|&i| ((skin0[i] - el0).dot(&af0) - s * len0).abs() < 4.0)
                .max_by(|&a, &c| flat(skin0[a] - el0, &af0).dot(&dorsal).total_cmp(&flat(skin0[c] - el0, &af0).dot(&dorsal)))
                .unwrap_or(0)
        })
        .collect();
    let angle_about = |from: Vector3<f64>, to: Vector3<f64>, a: &Vector3<f64>| from.cross(&to).dot(a).atan2(from.dot(&to)).to_degrees();
    // Each turn: the hand's own turn (the palm about the forearm's axis) and the skin's at each
    // station, signed as the hand's.
    let mut twist_rows = Vec::new();
    for (k, &t) in turns.iter().enumerate().skip(1) {
        let (skin, mo) = &twisted[k];
        let el = at(mo, "lowerarm01.R");
        let af = (at(mo, "wrist.R") - el).normalize();
        let hand = upper::hand_frame(&b, raw, mo);
        let palm_turn = angle_about(flat(hand0.palm, &af0), flat(hand.palm, &af), &af);
        let sign = palm_turn.signum();
        let skin_turns: Vec<f64> = marks.iter().map(|&i| sign * angle_about(flat(skin0[i] - el0, &af0), flat(skin[i] - el, &af), &af)).collect();
        twist_rows.push((t, palm_turn.abs(), skin_turns, hand.palm.z));
    }
    // A band along the back of the forearm (palm in), to see it twist.
    let band: Vec<bool> = (0..skin0.len())
        .map(|i| fore_share[i] > 0.5 && {
            let d = skin0[i] - el0;
            let s = d.dot(&af0) / len0;
            (0.05..0.95).contains(&s) && flat(d, &af0).dot(&dorsal) > 12f64.to_radians().cos()
        })
        .collect();
    let band_quads: Vec<[u32; 4]> = m.quads.iter().copied().filter(|q| q.iter().all(|&i| band[i as usize])).collect();
    let fore_quads: Vec<[u32; 4]> = m.quads.iter().copied().filter(|q| q.iter().all(|&i| fore_share[i as usize] > 0.5) && !q.iter().all(|&i| band[i as usize])).collect();

    // Report.
    let mut lines = Vec::new();
    let mut say = |s: String| {
        println!("{s}");
        lines.push(s);
    };
    say(format!("the elbow pad's first step (track U2): the stand-in's right elbow and forearm ({})", scan.display()));
    say(format!(
        "  ranges: elbow flexion {ELBOW_FLEXION}, pronation {PRONATION}, supination {SUPINATION} deg (CDC's Joint Range of Motion Study, men 20-44, means)"
    ));
    say(format!(
        "  the elbow bent from straight, palm in: how far the back of the elbow stands behind the joint's centre (along the bend's bisector), and the skin's length over the back of the elbow between marks {MARK:.0} mm up the upper arm and down the forearm, in the plane the arm bends in; MakeHuman's elbow, then with an olecranon (its skin moving with the forearm: {ole_n} of the skin's control vertices, {ole_full} of them fully; OLECRANON {OLECRANON} mm):"
    ));
    for (label, sw) in [("MakeHuman's", &plain), ("with the olecranon", &with)] {
        say(format!("    {label}:"));
        let base = sw.bends[0].length;
        for bd in &sw.bends {
            say(format!(
                "      {:>5.1} deg: back {:>5.1} mm; skin over the back {}{}",
                bd.angle,
                bd.back,
                bd.length.map_or("not found".to_string(), |l| format!("{l:.1} mm")),
                match (bd.length, base) {
                    (Some(l), Some(l0)) if bd.angle > 0.0 => format!(" (+{:.1}: {:.2} mm per deg)", l - l0, (l - l0) / bd.angle),
                    _ => String::new(),
                }
            ));
        }
    }
    say(format!("    (the shoulder joint to the elbow joint {:.1} mm)", with.uarm));
    say(format!(
        "  the forearm turned, the elbow at 90 deg: the hand's own turn, and the skin's along the back of the forearm (as it lies palm in) about the forearm's axis, at {} of the way from the elbow joint to the wrist joint ({len0:.1} mm):",
        ALONG.iter().map(|s| format!("{s:.2}")).collect::<Vec<_>>().join(" / ")
    ));
    for (t, palm, sk, pz) in &twist_rows {
        say(format!(
            "    {} {:>4.1} deg: the hand turns {:.1} (the palm {}); the skin {} deg",
            if *t > 0.0 { "pronated " } else { "supinated" },
            t.abs(),
            palm,
            if *pz < 0.0 { "down" } else { "up" },
            sk.iter().map(|a| format!("{a:.1}")).collect::<Vec<_>>().join(" / ")
        ));
    }
    std::fs::write(docs.join("summary.txt"), lines.join("\n") + "\n").map_err(|e| e.to_string())?;

    // Pictures: the arm bent (seen along the elbow's axis), the section over its back dark;
    // the forearm turned, a band along its back (as it lies palm in) dark.
    let skin_c = [0.85, 0.70, 0.60];
    let dark = [0.25, 0.18, 0.15];
    let mut layers: Vec<Layer> = with.arms.iter().map(|a| Layer::solid(a, skin_c)).collect();
    layers.extend(with.strips.iter().map(|s| Layer::solid(s, dark)));
    draw(&layers, &[View::looking(-with.n, with.up0)], 1.5, &docs.join("elbow_bend.png"), Style::pretty())?;
    let mut keep = Vec::new();
    for (k, (skin, mo)) in twisted.iter().enumerate() {
        let el = at(mo, "lowerarm01.R");
        let side = af0.cross(&dorsal).normalize();
        let shift = Motion { r: Rotation3::identity(), t: side * (130.0 * k as f64) - el.coords };
        keep.push((crate::glove::moved(&quad_mesh(skin, &fore_quads), &shift), skin_c));
        keep.push((crate::glove::moved(&quad_mesh(skin, &band_quads), &shift), dark));
    }
    let layers: Vec<Layer> = keep.iter().map(|(mm, c)| Layer::solid(mm, *c)).collect();
    draw(&layers, &[View::looking(-dorsal, af0)], 1.5, &docs.join("forearm_twist.png"), Style::pretty())?;
    println!("drew {}/elbow_bend.png, forearm_twist.png ({:.0} s)", docs.display(), t0.elapsed().as_secs_f64());
    Ok(())
}
