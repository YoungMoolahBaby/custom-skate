//! U2, the elbow pad as a padded sleeve, step 1 (2026-11-14): Jon prefers a lacrosse elbow pad, a
//! padded compression sleeve, and a slash guard as a padded plate in a sleeve along the forearm
//! (PLAN.md, open questions). Before the pads, what the skin under the sleeve does, on the
//! stand-in arm with an olecranon (`elbow::add_olecranon`):
//!
//! - **Its stretch.** Each triangle of the smoothed skin's principal stretches, from the arm
//!   straight to each step of the elbow's bend, and from the stance (90 deg, palm in) to the
//!   forearm's full pronation and supination (CDC's means, RESEARCH.md 29). A foam tile bonded to
//!   the sleeve can't stretch: where the skin under it stretches, the channels round it open by
//!   the excess times the tile's pitch.
//! - **The zones.** Over the elbow's padded area (its back, round to just past the elbow's axis):
//!   tiles where the skin stretches little at every step; the stretch zone, where it stretches too
//!   much for bonded tiles; and the crease, where it squeezes (the sleeve's front, left as fabric).
//! - **The main pad** over the stretch zone: anchored on the skin that moves with the forearm at the
//!   elbow's point (the olecranon), its inside not stretching, so the rest of it slides over the
//!   skin. Along the elbow's back (the plane it bends in), how long it must reach either side of the
//!   anchor to cover the stretch zone at every step, how far its ends slide over the skin, and how
//!   much its outside must open (in grooves) as it bends; and how far the skin round the anchor
//!   strays from moving rigidly (a hard insert there, if the fall check wants one).
//! - **The slash guard's plate** along the back of the forearm (the back of the hand's side), in
//!   three lengthwise segments: in bands along the forearm, how far the skin under each strays from
//!   moving rigidly as the elbow bends and the forearm turns, which says where a hard insert can go.
//!
//! The skin is MakeHuman's, bent by skin weights: it stretches where a real arm's skin might fold
//! or slide over the bone, so the stretch is where the sleeve has to give, not a measured strain.
//! Frame: the foot's (x forward, y left, z up, mm).

use std::path::Path;
use std::time::Instant;

use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Matrix2, Matrix3, Point3, Vector3};

use crate::body;
use crate::elbow::{self, ARM_BONES, ELBOW_FLEXION, PRONATION, SUPINATION};
use crate::glove::{moved, normals, quad_mesh};
use crate::pad::draw;
use crate::pose::Motion;
use crate::render::{Layer, Style, View};
use crate::upper;

/// The elbow's bend (deg), palm in; the stance is 90 (U2).
const STEPS: [f64; 8] = [0.0, 20.0, 40.0, 60.0, 90.0, 110.0, 130.0, ELBOW_FLEXION];
const STANCE: usize = 4;
/// A tile bonded to the sleeve where the skin under it stretches at most this at every step (its
/// channels then open by at most 15% of its pitch). A placeholder.
const TILE_MAX: f64 = 1.15;
/// The tiles' pitch, mm (a placeholder: about a G-Form pad's squares, from Jon's photo, not
/// measured).
const PITCH: f64 = 20.0;
/// Where the skin squeezes below this at full bend: the crease.
const CREASE: f64 = 0.85;
/// The elbow's padded area: from this far up the upper arm to this far down the forearm (mm),
/// round the back to this far past the elbow's axis toward the front (as the hard pad's).
const PAD_UP: f64 = 120.0;
const PAD_DOWN: f64 = 100.0;
const PAD_ROUND: f64 = 10.0;
/// The main pad reaches this much past the stretch zone either side, at its tightest step.
const MARGIN: f64 = 10.0;
/// A hard insert in the main pad, if one: this far round its anchor.
const INSERT_R: f64 = 25.0;
/// The slash guard's plate: from and to these fractions of the way from the elbow joint to the
/// wrist joint (under the elbow pad's lower edge, under the glove's short cuff), this far either
/// side of the forearm's back (deg about its axis), in three segments.
const PLATE_FROM: f64 = 0.30;
const PLATE_TO: f64 = 0.90;
const PLATE_HALF: f64 = 50.0;
const BANDS: usize = 4;

/// A triangle's principal stretches from `p0` to `p1`, largest first.
fn stretches(p0: [Point3<f64>; 3], p1: [Point3<f64>; 3]) -> Option<(f64, f64)> {
    let local = |p: &[Point3<f64>; 3]| {
        let (e1, e2) = (p[1] - p[0], p[2] - p[0]);
        let n = e1.cross(&e2);
        if n.norm() < 1e-9 {
            return None;
        }
        let u = e1.normalize();
        let v = n.normalize().cross(&u);
        Some(Matrix2::new(e1.dot(&u), e2.dot(&u), e1.dot(&v), e2.dot(&v)))
    };
    let f = local(&p1)? * local(&p0)?.try_inverse()?;
    let ev = (f.transpose() * f).symmetric_eigenvalues();
    let (a, b) = (ev[0].max(0.0).sqrt(), ev[1].max(0.0).sqrt());
    Some((a.max(b), a.min(b)))
}

/// How far points stray from the rigid motion that best carries `a` to `b` (Kabsch): RMS and
/// largest, mm.
fn rigid_residual(a: &[Point3<f64>], b: &[Point3<f64>]) -> (f64, f64) {
    if a.len() < 3 {
        return (f64::NAN, f64::NAN);
    }
    let n = a.len() as f64;
    let ca = a.iter().fold(Vector3::zeros(), |s, p| s + p.coords) / n;
    let cb = b.iter().fold(Vector3::zeros(), |s, p| s + p.coords) / n;
    let mut h = Matrix3::zeros();
    for (p, q) in a.iter().zip(b) {
        h += (p.coords - ca) * (q.coords - cb).transpose();
    }
    let svd = h.svd(true, true);
    let (u, vt) = (svd.u.unwrap(), svd.v_t.unwrap());
    let mut v = vt.transpose();
    if (v * u.transpose()).determinant() < 0.0 {
        let c = -v.column(2);
        v.set_column(2, &c);
    }
    let r = v * u.transpose();
    let errs: Vec<f64> = a.iter().zip(b).map(|(p, q)| (r * (p.coords - ca) + cb - q.coords).norm()).collect();
    ((errs.iter().map(|e| e * e).sum::<f64>() / n).sqrt(), errs.iter().copied().fold(0.0, f64::max))
}

/// `cargo run --release -- sleeve <scan>`: the padded sleeve, step 1 (docs/elbow/sleeve/).
pub fn run(scan: &Path) -> Result<(), String> {
    let t0 = Instant::now();
    let docs = Path::new("docs/elbow/sleeve");
    std::fs::create_dir_all(docs).map_err(|e| e.to_string())?;
    let mut b = body::build(scan, 1250.0)?;
    let (u, _) = upper::fit(&b)?;
    let raw = &u.raw;
    let (ole_n, _) = elbow::add_olecranon(&mut b, raw)?;
    let m = &b.model;
    println!("  body fitted, olecranon added ({:.0} s)", t0.elapsed().as_secs_f64());

    // The arm bent, palm in; and at the stance, turned.
    let bends: Vec<(Vec<Point3<f64>>, Vec<Motion>)> = STEPS.iter().map(|&a| elbow::skin_in(&b, raw, &upper::forearm_pose(&b, raw, a, false, 0.0))).collect();
    let turn_deg = [PRONATION, -SUPINATION];
    let turns: Vec<(Vec<Point3<f64>>, Vec<Motion>)> = turn_deg.iter().map(|&t| elbow::skin_in(&b, raw, &upper::forearm_pose(&b, raw, 90.0, false, t))).collect();
    let last = STEPS.len() - 1;
    let at = |mo: &[Motion], bone: &str| upper::posed_end(&b, raw, mo, bone, false);
    let (sk0, mo0) = &bends[0];
    let (sks, mos) = &bends[STANCE];
    let el = at(mo0, "lowerarm01.R");
    let wr = at(mo0, "wrist.R");
    let ap = (wr - el).normalize();
    let fore_len = (wr - el).norm();
    let up = (at(mo0, "upperarm01.R") - el).normalize();
    let f90 = (at(mos, "wrist.R") - at(mos, "lowerarm01.R")).normalize();
    // The elbow's back (straight: square to the arm, away from the bend) and its axis.
    let d = -(f90 - ap * f90.dot(&ap)).normalize();
    let w = d.cross(&ap).normalize();

    // The arm's skin from 160 mm up the upper arm to the wrist, straight.
    let arm_share = elbow::owned(&b, &ARM_BONES);
    let fore_share = elbow::owned(&b, &["lowerarm01.R", "lowerarm02.R", "olecranon.R"]);
    let s_of = |q: &Point3<f64>| (q - el).dot(&ap);
    let near: Vec<bool> = (0..sk0.len()).map(|i| arm_share[i] > 0.5 && (-160.0..fore_len).contains(&s_of(&sk0[i]))).collect();
    let quads: Vec<[u32; 4]> = m.quads.iter().copied().filter(|q| q.iter().all(|&i| near[i as usize])).collect();
    let tris: Vec<[usize; 3]> = quads
        .iter()
        .flat_map(|q| {
            let q = q.map(|i| i as usize);
            [[q[0], q[1], q[2]], [q[0], q[2], q[3]]]
        })
        .collect();
    let verts: Vec<usize> = (0..near.len()).filter(|&i| near[i]).collect();

    // Each vertex's stretches (the mean of its triangles'), from one pose to another.
    let field = |from: &[Point3<f64>], to: &[Point3<f64>]| -> (Vec<f64>, Vec<f64>) {
        let n = from.len();
        let (mut s1, mut s2, mut c) = (vec![0.0f64; n], vec![0.0f64; n], vec![0usize; n]);
        for t in &tris {
            if let Some((a, bb)) = stretches(t.map(|i| from[i]), t.map(|i| to[i])) {
                for &i in t {
                    s1[i] += a;
                    s2[i] += bb;
                    c[i] += 1;
                }
            }
        }
        let l1 = (0..n).map(|i| if c[i] > 0 { s1[i] / c[i] as f64 } else { 1.0 }).collect();
        let l2 = (0..n).map(|i| if c[i] > 0 { s2[i] / c[i] as f64 } else { 1.0 }).collect();
        (l1, l2)
    };
    let bend_f: Vec<(Vec<f64>, Vec<f64>)> = bends.iter().map(|(sk, _)| field(sk0, sk)).collect();
    let twist_f: Vec<(Vec<f64>, Vec<f64>)> = turns.iter().map(|(sk, _)| field(sks, sk)).collect();
    // The most each vertex stretches, and squeezes, over the bend and the turns.
    let l1_max: Vec<f64> = (0..sk0.len()).map(|i| bend_f.iter().chain(&twist_f).map(|f| f.0[i]).fold(1.0, f64::max)).collect();
    println!("  the skin's stretch over {} poses ({:.0} s)", bends.len() + turns.len(), t0.elapsed().as_secs_f64());

    // The elbow's padded area, and its zones.
    let in_pad = |i: usize| {
        let q = sk0[i] - el;
        (-PAD_UP..=PAD_DOWN).contains(&q.dot(&ap)) && q.dot(&d) > -PAD_ROUND
    };
    let crease: Vec<bool> = (0..sk0.len()).map(|i| near[i] && bend_f[last].1[i] < CREASE).collect();
    let stretch_zone: Vec<bool> = (0..sk0.len()).map(|i| near[i] && in_pad(i) && l1_max[i] > TILE_MAX && !crease[i]).collect();
    let tile_zone: Vec<bool> = (0..sk0.len()).map(|i| near[i] && in_pad(i) && !stretch_zone[i] && !crease[i]).collect();
    let area_of = |sel: &dyn Fn(usize) -> bool, pts: &[Point3<f64>]| -> f64 {
        quads
            .iter()
            .filter(|q| q.iter().all(|&i| sel(i as usize)))
            .map(|q| {
                let [a, bq, c, dq] = q.map(|i| pts[i as usize]);
                0.5 * (c - a).cross(&(dq - bq)).norm()
            })
            .sum()
    };
    let pad_area = area_of(&|i| in_pad(i) && near[i], sk0);
    let sz_area = area_of(&|i| stretch_zone[i], sk0);
    let tz_area = area_of(&|i| tile_zone[i], sk0);
    let sz_s: Vec<f64> = verts.iter().filter(|&&i| stretch_zone[i]).map(|&i| s_of(&sk0[i])).collect();
    let sz_half = verts.iter().filter(|&&i| stretch_zone[i] && s_of(&sk0[i]).abs() < 10.0 && (sk0[i] - el).dot(&d) > 0.0).map(|&i| (sk0[i] - el).dot(&w).abs()).fold(0.0, f64::max);
    let cr_s: Vec<f64> = verts.iter().filter(|&&i| crease[i]).map(|&i| s_of(&sk0[i])).collect();
    let cr_min = verts.iter().filter(|&&i| crease[i]).map(|&i| bend_f[last].1[i]).fold(1.0, f64::min);
    let quant = |v: &mut Vec<f64>, q: f64| {
        if v.is_empty() {
            return f64::NAN;
        }
        v.sort_by(|a, bb| a.partial_cmp(bb).unwrap());
        v[((v.len() - 1) as f64 * q) as usize]
    };
    // The tiles' channels: how far they open, at the stance, at full bend, and at most.
    let chan = |k: Option<usize>| -> (f64, f64) {
        let mut v: Vec<f64> = verts
            .iter()
            .filter(|&&i| tile_zone[i])
            .map(|&i| (k.map_or(l1_max[i], |k| bend_f[k].0[i]) - 1.0).max(0.0) * PITCH)
            .collect();
        (quant(&mut v, 0.9), quant(&mut v, 1.0))
    };

    // The elbow's back, in the plane it bends in: the skin points within 3 mm of it, every 2 mm
    // along the arm, straight; the anchor (the nearest of them to the upper arm that moves with
    // the forearm, the olecranon's skin); the stretch zone's ends along it.
    let mut mid: Vec<usize> = Vec::new();
    {
        let mut cand: Vec<usize> = verts.iter().copied().filter(|&i| (sk0[i] - el).dot(&w).abs() < 3.0 && (sk0[i] - el).dot(&d) > 0.0 && (-150.0..150.0).contains(&s_of(&sk0[i]))).collect();
        cand.sort_by(|&a, &c| s_of(&sk0[a]).partial_cmp(&s_of(&sk0[c])).unwrap());
        for i in cand {
            match mid.last() {
                Some(&j) if s_of(&sk0[i]) - s_of(&sk0[j]) < 2.0 => {
                    if (sk0[i] - el).dot(&w).abs() < (sk0[j] - el).dot(&w).abs() {
                        *mid.last_mut().unwrap() = i;
                    }
                }
                _ => mid.push(i),
            }
        }
    }
    let ms: Vec<f64> = mid.iter().map(|&i| s_of(&sk0[i])).collect();
    let anchor = (0..mid.len()).filter(|&k| fore_share[mid[k]] > 0.9 && ms[k] > -30.0).min_by(|&a, &c| ms[a].partial_cmp(&ms[c]).unwrap()).ok_or("no skin moving with the forearm behind the elbow")?;
    let top = (0..anchor).filter(|&k| ms[k] >= -PAD_UP && l1_max[mid[k]] > TILE_MAX).min_by(|&a, &c| ms[a].partial_cmp(&ms[c]).unwrap()).unwrap_or(anchor);
    let bot = (anchor..mid.len()).filter(|&k| ms[k] <= PAD_DOWN && l1_max[mid[k]] > TILE_MAX).max_by(|&a, &c| ms[a].partial_cmp(&ms[c]).unwrap()).unwrap_or(anchor);
    // Arc length along the back at each step, from the first point.
    let arcs: Vec<Vec<f64>> = bends
        .iter()
        .map(|(sk, _)| {
            let mut a = vec![0.0];
            for k in 1..mid.len() {
                a.push(a[k - 1] + (sk[mid[k]] - sk[mid[k - 1]]).norm());
            }
            a
        })
        .collect();
    let l_up = arcs.iter().map(|a| a[anchor] - a[top]).fold(0.0, f64::max) + MARGIN;
    let l_dn = arcs.iter().map(|a| a[bot] - a[anchor]).fold(0.0, f64::max) + MARGIN;
    // Where the pad's ends lie on the skin (straight, along the arm) at each step.
    let reach = |a: &[f64], target: f64| -> f64 {
        // The straight-arm place of the point at arc length `target` along the back.
        if target <= a[0] {
            return ms[0];
        }
        for k in 1..a.len() {
            if a[k] >= target {
                let f = (target - a[k - 1]) / (a[k] - a[k - 1]).max(1e-9);
                return ms[k - 1] + f * (ms[k] - ms[k - 1]);
            }
        }
        ms[ms.len() - 1]
    };
    let ends: Vec<(f64, f64)> = arcs.iter().map(|a| (reach(a, a[anchor] - l_up), reach(a, a[anchor] + l_dn))).collect();
    // How far the back turns between the pad's ends at each step (rad, about the elbow's axis):
    // the angle between its directions over the last 10 mm at either end. The pad's outside
    // lengthens over its inside by that times its thickness. (Summing the turn from point to
    // point added up the line's zigzag, 2 mm apart: over 300 deg straight.)
    let turning: Vec<f64> = bends
        .iter()
        .zip(&ends)
        .map(|((sk, _), &(s_top, s_bot))| {
            let ks: Vec<usize> = (0..mid.len()).filter(|&k| ms[k] >= s_top && ms[k] <= s_bot).collect();
            if ks.len() < 4 {
                return 0.0;
            }
            let p = |k: usize| sk[mid[k]];
            let first = ks.iter().copied().find(|&k| ms[k] - ms[ks[0]] >= 10.0).unwrap_or(ks[1]);
            let lastk = ks.iter().rev().copied().find(|&k| ms[ks[ks.len() - 1]] - ms[k] >= 10.0).unwrap_or(ks[ks.len() - 2]);
            let t1 = (p(first) - p(ks[0])).normalize();
            let t2 = (p(ks[ks.len() - 1]) - p(lastk)).normalize();
            t1.cross(&t2).dot(&w).atan2(t1.dot(&t2))
        })
        .collect();
    // A hard insert round the anchor, on the skin that moves with the forearm: how far that skin
    // strays from moving rigidly. (Within 25 mm of the anchor, the skin above it stretches with
    // the elbow's back: 12 mm RMS at full bend.)
    let ins: Vec<usize> = verts.iter().copied().filter(|&i| (sk0[i] - sk0[mid[anchor]]).norm() < INSERT_R && fore_share[i] > 0.9).collect();
    let ins_res: Vec<(f64, f64)> = bends.iter().map(|(sk, _)| rigid_residual(&ins.iter().map(|&i| sk0[i]).collect::<Vec<_>>(), &ins.iter().map(|&i| sk[i]).collect::<Vec<_>>())).collect();

    // The slash guard's plate: the back of the forearm (the back of the hand's side), straight.
    let hand0 = upper::hand_frame(&b, raw, mo0);
    let flat = |v: Vector3<f64>| v - ap * v.dot(&ap);
    let dorsal = flat(-hand0.palm).normalize();
    let side = ap.cross(&dorsal).normalize();
    let plate_cell = |i: usize| -> Option<(usize, usize)> {
        if fore_share[i] <= 0.5 {
            return None;
        }
        let q = sk0[i] - el;
        let f = q.dot(&ap) / fore_len;
        let r = flat(q);
        let phi = r.dot(&side).atan2(r.dot(&dorsal)).to_degrees();
        if !(PLATE_FROM..PLATE_TO).contains(&f) || phi.abs() > PLATE_HALF {
            return None;
        }
        let band = (((f - PLATE_FROM) / (PLATE_TO - PLATE_FROM)) * BANDS as f64).floor().min((BANDS - 1) as f64) as usize;
        let seg = (((phi + PLATE_HALF) / (2.0 * PLATE_HALF)) * 3.0).floor().min(2.0) as usize;
        Some((band, seg))
    };
    let cells: Vec<Option<(usize, usize)>> = (0..sk0.len()).map(|i| if near[i] { plate_cell(i) } else { None }).collect();
    // Per cell: how far its skin strays from rigid, the arm straight to the stance and to full
    // bend, and the stance to each turn (RMS, largest).
    let plate: Vec<Vec<[(f64, f64); 4]>> = (0..BANDS)
        .map(|bd| {
            (0..3)
                .map(|sg| {
                    let ids: Vec<usize> = verts.iter().copied().filter(|&i| cells[i] == Some((bd, sg))).collect();
                    let pts = |sk: &[Point3<f64>]| ids.iter().map(|&i| sk[i]).collect::<Vec<_>>();
                    [rigid_residual(&pts(sk0), &pts(sks)), rigid_residual(&pts(sk0), &pts(&bends[last].0)), rigid_residual(&pts(sks), &pts(&turns[0].0)), rigid_residual(&pts(sks), &pts(&turns[1].0))]
                })
                .collect()
        })
        .collect();
    // A segment in one piece: its whole length, and the half nearer the elbow, as the forearm
    // turns (RMS, largest).
    let whole: Vec<[(f64, f64); 4]> = (0..3)
        .map(|sg| {
            let ids = |bands: std::ops::Range<usize>| -> Vec<usize> { verts.iter().copied().filter(|&i| cells[i].is_some_and(|c| c.1 == sg && bands.contains(&c.0))).collect() };
            let res = |ids: &[usize], k: usize| rigid_residual(&ids.iter().map(|&i| sks[i]).collect::<Vec<_>>(), &ids.iter().map(|&i| turns[k].0[i]).collect::<Vec<_>>());
            let (all, near_half) = (ids(0..BANDS), ids(0..BANDS / 2));
            [res(&all, 0), res(&all, 1), res(&near_half, 0), res(&near_half, 1)]
        })
        .collect();
    let plate_l1: Vec<[f64; 2]> = (0..BANDS)
        .map(|bd| {
            let ids: Vec<usize> = verts.iter().copied().filter(|&i| cells[i].is_some_and(|c| c.0 == bd)).collect();
            [0, 1].map(|k| ids.iter().map(|&i| twist_f[k].0[i]).fold(1.0, f64::max))
        })
        .collect();
    println!("  zones, the main pad, the plate ({:.0} s)", t0.elapsed().as_secs_f64());

    // Report.
    let mut lines = Vec::new();
    let mut say = |s: String| {
        println!("{s}");
        lines.push(s);
    };
    let range = |v: &[f64]| (v.iter().copied().fold(f64::MAX, f64::min), v.iter().copied().fold(f64::MIN, f64::max));
    say(format!("the elbow pad as a padded sleeve, step 1 (track U2): the right arm with an olecranon ({ole_n} skin control vertices move with it) ({})", scan.display()));
    say(format!(
        "  the elbow bent {} deg palm in; at the stance (90) the forearm turned {PRONATION} deg (pronated) and {SUPINATION} (supinated) (CDC's means); along the arm from the elbow joint (+ down the forearm, {fore_len:.1} mm to the wrist joint)",
        STEPS.iter().map(|s| format!("{s:.0}")).collect::<Vec<_>>().join(" / ")
    ));
    say(format!(
        "  the elbow's padded area ({PAD_UP:.0} mm up to {PAD_DOWN:.0} mm down, round the back to {PAD_ROUND:.0} mm past the elbow's axis): {:.0} cm2 of skin, straight",
        pad_area / 100.0
    ));
    let (sz0, sz1) = range(&sz_s);
    say(format!(
        "    the stretch zone, where the skin stretches over {TILE_MAX:.2} at some step (too much for a tile bonded to the sleeve): {:.0} cm2, from {:.0} to {:+.0} mm along the arm, {:.0} mm across at the joint's level",
        sz_area / 100.0,
        sz0,
        sz1,
        2.0 * sz_half
    ));
    let (c90, _) = chan(Some(STANCE));
    let (cfull, _) = chan(Some(last));
    let (cmax90, cmax) = chan(None);
    say(format!(
        "    the tiles, the rest: {:.0} cm2; at a {PITCH:.0} mm pitch their channels open (90th percentile) {c90:.1} mm at the stance, {cfull:.1} at full bend, {cmax90:.1} at the most of any step or turn ({cmax:.1} the most anywhere)",
        tz_area / 100.0
    ));
    let (cr0, cr1) = range(&cr_s);
    say(format!("    the crease, where the skin squeezes under {CREASE:.2} at full bend (the sleeve's front, left as fabric): from {cr0:.0} to {cr1:+.0} mm along the arm, squeezing to {cr_min:.2} (where the model's skin folds through itself)"));
    say(format!(
        "  the main pad over the stretch zone, along the elbow's back: anchored {:+.0} mm along the arm (the skin there moves with the forearm, {:.0}%); the stretch zone along the back from {:+.0} to {:+.0} mm; its inside reaches {:.0} mm up and {:.0} mm down from the anchor (covering the stretch zone at every step, {MARGIN:.0} mm past it at the tightest)",
        ms[anchor],
        100.0 * fore_share[mid[anchor]],
        ms[top],
        ms[bot],
        l_up,
        l_dn
    ));
    say(format!(
        "    where its ends lie on the skin (straight, along the arm) at each step, up / down: {}; so its top slides {:.0} mm over the skin from straight to full bend, its bottom {:.0}",
        STEPS.iter().zip(&ends).map(|(a, (t, bt))| format!("{a:.0}: {t:+.0} / {bt:+.0}")).collect::<Vec<_>>().join(", "),
        (ends[0].0 - ends[last].0).abs(),
        (ends[0].1 - ends[last].1).abs()
    ));
    say(format!(
        "    the back turns between its ends (the angle between its directions over the last 10 mm at either end): {} deg at each step; its outside lengthens over its inside by {:.2} mm per mm of its thickness at the stance, {:.2} at full bend, from straight (in grooves across it)",
        turning.iter().map(|t| format!("{:.0}", t.to_degrees())).collect::<Vec<_>>().join(" / "),
        (turning[STANCE] - turning[0]).abs(),
        (turning[last] - turning[0]).abs()
    ));
    say(format!(
        "    a hard insert within {INSERT_R:.0} mm of the anchor, on the skin that moves with the forearm ({} skin points): how far the skin there strays from moving rigidly from straight, RMS / most (mm), at each step: {}",
        ins.len(),
        STEPS.iter().zip(&ins_res).map(|(a, (r, mx))| format!("{a:.0}: {r:.1}/{mx:.1}")).collect::<Vec<_>>().join(", ")
    ));
    say(format!(
        "  the slash guard's plate, the back of the forearm from {PLATE_FROM:.2} to {PLATE_TO:.2} of the way to the wrist, {PLATE_HALF:.0} deg either side, in 3 lengthwise segments: how far the skin under each strays from moving rigidly, RMS / most (mm), from straight to the stance and to full bend, and from the stance to pronated and to supinated; and the most the skin in the band stretches as the forearm turns (pronated / supinated):"
    ));
    for (bd, row) in plate.iter().enumerate() {
        let f0 = PLATE_FROM + (PLATE_TO - PLATE_FROM) * bd as f64 / BANDS as f64;
        let f1 = f0 + (PLATE_TO - PLATE_FROM) / BANDS as f64;
        say(format!(
            "    {f0:.2}-{f1:.2}: {}; stretch {:.2} / {:.2}",
            row.iter()
                .enumerate()
                .map(|(sg, r)| format!("segment {}: {}", sg + 1, r.iter().map(|(a, mx)| format!("{a:.1}/{mx:.1}")).collect::<Vec<_>>().join(", ")))
                .collect::<Vec<_>>()
                .join("; "),
            plate_l1[bd][0],
            plate_l1[bd][1]
        ));
    }
    say(format!(
        "    each segment in one piece as the forearm turns, RMS / most (mm), pronated, supinated: its whole length {}; the half nearer the elbow ({PLATE_FROM:.2}-{:.2}) {}",
        whole.iter().enumerate().map(|(sg, r)| format!("segment {}: {:.1}/{:.1}, {:.1}/{:.1}", sg + 1, r[0].0, r[0].1, r[1].0, r[1].1)).collect::<Vec<_>>().join("; "),
        PLATE_FROM + 0.5 * (PLATE_TO - PLATE_FROM),
        whole.iter().enumerate().map(|(sg, r)| format!("segment {}: {:.1}/{:.1}, {:.1}/{:.1}", sg + 1, r[2].0, r[2].1, r[3].0, r[3].1)).collect::<Vec<_>>().join("; ")
    ));
    std::fs::write(docs.join("summary.txt"), lines.join("\n") + "\n").map_err(|e| e.to_string())?;

    // Pictures.
    let skin_c = [0.85, 0.70, 0.60];
    let bands_c: [(f64, f64, [f64; 3]); 5] = [(0.0, 1.05, [0.93, 0.90, 0.82]), (1.05, TILE_MAX, [0.95, 0.80, 0.50]), (TILE_MAX, 1.3, [0.92, 0.60, 0.28]), (1.3, 1.6, [0.78, 0.38, 0.14]), (1.6, f64::MAX, [0.50, 0.20, 0.08])];
    let crease_c = [0.40, 0.55, 0.85];
    let offset = |sk: &[Point3<f64>], by: f64| -> Vec<Point3<f64>> {
        let n = normals(sk, &quads);
        (0..sk.len()).map(|i| sk[i] + n[i] * by).collect()
    };
    let sel_quads = |f: &dyn Fn(usize) -> bool| -> Vec<[u32; 4]> { quads.iter().copied().filter(|q| q.iter().all(|&i| f(i as usize))).collect() };
    let mean4 = |q: &[u32; 4], v: &[f64]| q.iter().map(|&i| v[i as usize]).sum::<f64>() / 4.0;
    // 1. The stretch at full bend (or the most, over the bend and the turns), on the arm at the
    // stance: from the side and from behind either side.
    let views = [View::looking(-w, up), View::looking((-d - w).normalize(), up), View::looking((-d + w).normalize(), up)];
    {
        let lifted = offset(sks, 0.5);
        let mut meshes: Vec<(IndexedMesh, [f64; 3])> = vec![(quad_mesh(sks, &quads), skin_c)];
        for &(lo, hi, c) in &bands_c {
            let qs: Vec<[u32; 4]> = quads.iter().copied().filter(|q| !q.iter().any(|&i| crease[i as usize]) && (lo..hi).contains(&mean4(q, &l1_max))).collect();
            meshes.push((quad_mesh(&lifted, &qs), c));
        }
        meshes.push((quad_mesh(&lifted, &sel_quads(&|i| crease[i])), crease_c));
        let layers: Vec<Layer> = meshes.iter().map(|(mm, c)| Layer::solid(mm, *c)).collect();
        draw(&layers, &views, 1.4, &docs.join("stretch.png"), Style::pretty())?;
    }
    // 2. The layout, straight, at the stance and at full bend: the main pad where it lies at that
    // step, the tiles (alternate squares, moving with the skin), the plate's segments.
    let checker: Vec<bool> = (0..sk0.len())
        .map(|i| {
            let q = sk0[i];
            let s = s_of(&q);
            let centre = if s < 0.0 { el + up * (q - el).dot(&up) } else { el + ap * s };
            let r = q - centre;
            let u = r.dot(&w).atan2(r.dot(&d)) * r.norm();
            ((s / PITCH).floor() as i64 + (u / PITCH).floor() as i64).rem_euclid(2) == 0
        })
        .collect();
    let pad_c = [0.85, 0.45, 0.15];
    let tile_c = [[0.30, 0.30, 0.34], [0.45, 0.45, 0.50]];
    let plate_c = [[0.10, 0.10, 0.12], [0.22, 0.22, 0.26], [0.10, 0.10, 0.12]];
    let forward = -d;
    let mut meshes: Vec<(IndexedMesh, [f64; 3])> = Vec::new();
    for (slot, ti) in [0usize, STANCE, last].into_iter().enumerate() {
        let sk = &bends[ti].0;
        let shift = Motion { r: nalgebra::Rotation3::identity(), t: forward * (330.0 * slot as f64) - el.coords };
        let (s_top, s_bot) = ends[ti];
        let under_pad = |i: usize| {
            let q = sk0[i] - el;
            (s_top..=s_bot).contains(&q.dot(&ap)) && q.dot(&w).abs() <= sz_half + MARGIN && q.dot(&d) > 0.0
        };
        let lifted = offset(sk, 3.0);
        let raised = offset(sk, 6.0);
        meshes.push((moved(&quad_mesh(sk, &quads), &shift), skin_c));
        meshes.push((moved(&quad_mesh(&raised, &sel_quads(&under_pad)), &shift), pad_c));
        for (k, c) in tile_c.iter().enumerate() {
            meshes.push((moved(&quad_mesh(&lifted, &sel_quads(&|i| tile_zone[i] && !under_pad(i) && checker[i] == (k == 0))), &shift), *c));
        }
        for (sg, c) in plate_c.iter().enumerate() {
            meshes.push((moved(&quad_mesh(&lifted, &sel_quads(&|i| cells[i].is_some_and(|cl| cl.1 == sg) && !under_pad(i))), &shift), *c));
        }
    }
    let layers: Vec<Layer> = meshes.iter().map(|(mm, c)| Layer::solid(mm, *c)).collect();
    draw(&layers, &[View::looking(-w, up), View::looking(w, up)], 1.0, &docs.join("layout.png"), Style::pretty())?;
    // 3. The forearm turned at the stance, from the back of the hand's side: pronated, palm in,
    // supinated, the skin's stretch from palm in banded, the plate's segments outlined by colour
    // in the middle one.
    {
        let hs = upper::hand_frame(&b, raw, mos);
        let af = (at(mos, "wrist.R") - at(mos, "lowerarm01.R")).normalize();
        let dors = (-hs.palm - af * (-hs.palm).dot(&af)).normalize();
        let across = af.cross(&dors).normalize();
        let fore_q = sel_quads(&|i| fore_share[i] > 0.5);
        let poses: [(&Vec<Point3<f64>>, Option<usize>); 3] = [(&turns[0].0, Some(0)), (sks, None), (&turns[1].0, Some(1))];
        let mut meshes: Vec<(IndexedMesh, [f64; 3])> = Vec::new();
        for (slot, (sk, k)) in poses.iter().enumerate() {
            let shift = Motion { r: nalgebra::Rotation3::identity(), t: across * (130.0 * slot as f64) - at(mos, "lowerarm01.R").coords };
            let lifted = offset(sk, 0.5);
            meshes.push((moved(&quad_mesh(sk, &fore_q), &shift), skin_c));
            match k {
                Some(k) => {
                    for &(lo, hi, c) in &bands_c {
                        let qs: Vec<[u32; 4]> = fore_q.iter().copied().filter(|q| (lo..hi).contains(&mean4(q, &twist_f[*k].0))).collect();
                        meshes.push((moved(&quad_mesh(&lifted, &qs), &shift), c));
                    }
                }
                None => {
                    for (sg, c) in plate_c.iter().enumerate() {
                        meshes.push((moved(&quad_mesh(&lifted, &sel_quads(&|i| cells[i].is_some_and(|cl| cl.1 == sg))), &shift), *c));
                    }
                }
            }
        }
        let layers: Vec<Layer> = meshes.iter().map(|(mm, c)| Layer::solid(mm, *c)).collect();
        draw(&layers, &[View::looking(-dors, af)], 1.5, &docs.join("twist.png"), Style::pretty())?;
    }
    println!("drew {}/stretch.png, layout.png, twist.png ({:.0} s)", docs.display(), t0.elapsed().as_secs_f64());
    Ok(())
}
