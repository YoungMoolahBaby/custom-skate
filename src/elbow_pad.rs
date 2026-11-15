//! U2, the elbow pad (PLAN.md): round 1 (2026-11-14), the knee's design at the elbow; round 2 (the
//! same day), the space under the scales filled from the suit, the scales held by stops, and the
//! straps.
//!
//! Hard pieces over a foam liner (in the compression suit's sleeve), on the stand-in arm with an
//! olecranon (`elbow::add_olecranon`): a shell over the back of the upper arm and a cap over the
//! end of the forearm (the olecranon), with scales between. Over the elbow each is a cylinder
//! about its axis, the piece nearer the shoulder over the next, so they slide without binding
//! (the glove's laps, `glove`). Built on the arm as it holds the stick (the elbow at 90 deg, U2),
//! checked from straight to the elbow's full flexion.
//!
//! Round 2:
//! - **The stops.** The scales are not held at exact shares of the bend: a stop at each end of
//!   every lap keeps it from opening past its span less `stop_overlap`, or closing past nested.
//!   The laps are checked closed at every split of the bend the stops allow (its extremes), and
//!   the pieces clear of each other over every turn between two of them the stops allow.
//! - **The patch.** The suit's foam made thicker behind the elbow: at each point of the skin, as
//!   thick as the least room over it (less the clearance) through the bend and every turn of the
//!   scales the stops allow. Foam fixed inside the scales can't do it: straight, they nest, each
//!   in the space under the one before.
//! - **The straps.** One round the upper arm (the shell's), one round the forearm (the cap's),
//!   over the arm's front where the pieces leave it: their length, and how near the arm's front
//!   folds onto them as the elbow bends.
//!
//! Frame: the foot's (x forward, y left, z up, mm). Each piece is defined where it is with the
//! arm straight ("pose 0") and moves with its bone.

use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

use cortenforge::cf_design::{Aabb, Solid};
use cortenforge::cf_scan_prep_core::auto_cap_open_boundaries;
use cortenforge::mesh::io::save_mesh;
use cortenforge::mesh::sdf::{ParitySign, Signed, TriMeshDistance, UnsignedDistance};
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::body;
use crate::elbow::{self, ARM_BONES, ELBOW_FLEXION};
use crate::glove::{EXT, LapZone, Mover, Piece, Rev, Sdf, motion_of, moved, normals, par_map, quad_mesh, section_lines};
use crate::hockey_boot::mesh_volume;
use crate::materials::{LINER_FOAM, PA12_MJF, STRAP};
use crate::pad::draw;
use crate::pose::Motion;
use crate::render::{Layer, Style, View};
use crate::round1::mesh_of;
use crate::upper;

/// The pad's sizes, mm (placeholders where marked).
#[derive(Clone, Copy, Debug)]
pub struct PadParams {
    /// The liner's foam on the skin (the knee's liner, `pad::PadParams::knee_liner`; a
    /// placeholder).
    pub foam: f64,
    /// The hard pieces' wall, PA 12 (as the pads', a placeholder).
    pub wall: f64,
    /// Between two pieces where one slides over the other, and between a piece and the patch.
    pub clear: f64,
    /// How far each lap still overlaps at the elbow's full bend, deg, its turn an even share.
    pub margin: f64,
    /// How far each lap still overlaps at its stop, deg (round 2; a placeholder): each lap opens
    /// at most its span less this.
    pub stop_overlap: f64,
    /// Scales between the upper arm's shell and the cap.
    pub scales: usize,
    /// The upper arm's shell reaches this far up the arm from the elbow joint.
    pub up_len: f64,
    /// The cap reaches this far down the forearm from the elbow joint.
    pub cap_len: f64,
    /// How far round the arm the pieces come: past the elbow's axis toward the front.
    pub down: f64,
    /// The straps' middles: the shell's this far up the arm from the joint (its top end), the
    /// cap's this far down the forearm (just past its lower end, where the slash guard's top lies
    /// under it). Round 2 first put them 20 mm in from each piece's end (80, 60): the forearm's
    /// then sat 6 mm from the upper arm's skin at the stance (90 deg), in the elbow's crease.
    pub strap_up: f64,
    pub strap_down: f64,
    /// The straps' width and thickness (placeholders).
    pub strap_w: f64,
    pub strap_t: f64,
    pub cell: f64,
}

impl Default for PadParams {
    fn default() -> Self {
        PadParams {
            foam: 9.0,
            wall: 2.0,
            clear: 1.0,
            margin: 10.0,
            stop_overlap: 5.0,
            scales: 2,
            up_len: 100.0,
            cap_len: 80.0,
            down: 10.0,
            strap_up: 100.0,
            strap_down: 90.0,
            strap_w: 25.0,
            strap_t: 1.5,
            cell: 0.75,
        }
    }
}

/// The sweep, the elbow's bend (deg): straight to full flexion; the pieces are built at 90.
const STEPS: [f64; 8] = [0.0, 20.0, 40.0, 60.0, 90.0, 110.0, 130.0, ELBOW_FLEXION];
const BUILD: usize = 4;
/// How far out from the liner a piece is looked for, mm (round 1 looked 20 mm and lost the
/// scales' points further off).
const REACH: f64 = 40.0;
/// The scales' turns looked at, across what the stops allow, at each step.
const TURNS: usize = 9;

/// A lap over the elbow, the `s`th from the shoulder: piece `s` over piece `s + 1`. Pose 0: the joint's
/// centre and axis, the elbow's back `d` and `ap` (down the forearm), square to the axis; its
/// surfaces reach `beta` back and `alpha` forward of `d` (rad), plus `EXT`.
struct Lap {
    alpha: f64,
    beta: f64,
    rho: f64,
}

/// The splits of a bend `phi` (rad) over `n` laps the stops allow, each lap's turn within
/// `0..=tm`: even shares first, then the extremes (every lap at a stop but one).
fn shares(phi: f64, n: usize, tm: f64) -> Vec<Vec<f64>> {
    let mut out = vec![vec![phi / n as f64; n]];
    for code in 0..3usize.pow(n as u32) {
        let (mut th, mut free, mut c, mut ok) = (vec![0.0; n], None, code, true);
        for (k, t) in th.iter_mut().enumerate() {
            match c % 3 {
                0 => {}
                1 => *t = tm,
                _ => {
                    ok &= free.is_none();
                    free = Some(k);
                }
            }
            c /= 3;
        }
        let fixed: f64 = th.iter().sum();
        match free {
            Some(k) if ok && (-1e-9..=tm + 1e-9).contains(&(phi - fixed)) => th[k] = (phi - fixed).clamp(0.0, tm),
            None if (fixed - phi).abs() < 1e-9 => {}
            _ => continue,
        }
        if !out.iter().any(|o| o.iter().zip(&th).all(|(a, b)| (a - b).abs() < 1e-6)) {
            out.push(th);
        }
    }
    out
}

/// The length of a mesh's section by a plane, mm (its contours' polylines).
fn cut_length(mesh: &IndexedMesh, p: Point3<f64>, n: Vector3<f64>, keep: impl Fn(&Point3<f64>) -> bool) -> f64 {
    crate::scan::section(mesh, p, n).contours.iter().map(|c| c.points.windows(2).filter(|s| keep(&s[0]) && keep(&s[1])).map(|s| (s[1] - s[0]).norm()).sum::<f64>()).sum()
}

/// `cargo run --release -- elbowpad <scan>`: the elbow pad, round 2 (docs/elbow/pad/).
pub fn run(scan: &Path) -> Result<(), String> {
    let t0 = Instant::now();
    let mut p = PadParams::default();
    if let Ok(v) = std::env::var("ELBOW_SCALES") {
        p.scales = v.trim().parse().map_err(|_| format!("ELBOW_SCALES={v:?}: a count"))?;
    }
    let docs_env = std::env::var("ELBOW_DOCS").ok().map(std::path::PathBuf::from);
    let docs = docs_env.as_deref().unwrap_or(Path::new("docs/elbow/pad"));
    let out = Path::new("out/elbow_pad");
    std::fs::create_dir_all(docs).map_err(|e| e.to_string())?;
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let threads = std::env::var("PAD_THREADS").ok().and_then(|v| v.parse::<usize>().ok()).unwrap_or(2).max(1);
    let mut b = body::build(scan, 1250.0)?;
    let (u, _) = upper::fit(&b)?;
    let raw = &u.raw;
    let (ole_n, _) = elbow::add_olecranon(&mut b, raw)?;
    let m = &b.model;
    let rig = &m.rig;
    let bi = |n: &str| rig.index[n];
    let (ub, fb) = (bi("upperarm02.R"), bi("lowerarm01.R"));
    println!("  body fitted, olecranon added ({:.0} s)", t0.elapsed().as_secs_f64());

    // The arm at each step: its skin (mm), and each bone's motion (mm).
    struct ArmAt {
        skin: Vec<Point3<f64>>,
        mo: Vec<Motion>,
        mm: Vec<Motion>,
    }
    let arms: Vec<ArmAt> = STEPS
        .iter()
        .map(|&a| {
            let (skin, mo) = elbow::skin_in(&b, raw, &upper::forearm_pose(&b, raw, a, false, 0.0));
            let mm = mo.iter().map(|x| body::motion_to_mm(x, b.floor, b.shift)).collect();
            ArmAt { skin, mo, mm }
        })
        .collect();
    let (a0, ab) = (&arms[0], &arms[BUILD]);
    let last = arms.len() - 1;
    let at = |h: &ArmAt, bone: &str| upper::posed_end(&b, raw, &h.mo, bone, false);
    let el = at(a0, "lowerarm01.R");
    let ap = (at(a0, "wrist.R") - el).normalize();
    let up = (at(a0, "upperarm01.R") - el).normalize();
    let f90 = (at(&arms[BUILD], "wrist.R") - at(&arms[BUILD], "lowerarm01.R")).normalize();
    // The elbow's back (straight: square to the arm, away from the bend) and its axis, the way
    // that turns the back toward `ap` as the elbow bends (the forearm's back faces down at 90).
    let d = -(f90 - ap * f90.dot(&ap)).normalize();
    let w = d.cross(&ap).normalize();
    // The elbow's bend at each step, as the forearm turns from the upper arm (rad).
    let rel_bone = |h: &ArmAt, bone: usize| h.mm[bone].after(&a0.mm[bone].inverse());
    let phis: Vec<f64> = arms.iter().map(|h| rel_bone(h, ub).inverse().after(&rel_bone(h, fb)).r.angle()).collect();
    if let Some((ti, ph)) = phis.iter().enumerate().find(|(ti, ph)| (ph.to_degrees() - STEPS[*ti]).abs() > 0.5) {
        return Err(format!("the elbow at step {ti} bends {:.1} deg, not {:.1}", ph.to_degrees(), STEPS[ti]));
    }

    // The skin near the elbow: the arm's within 170 mm of the joint along the arm, straight.
    let arm_share = elbow::owned(&b, &ARM_BONES);
    let near: Vec<bool> = (0..a0.skin.len()).map(|i| arm_share[i] > 0.5 && (a0.skin[i] - el).dot(&ap).abs() < 170.0).collect();
    let quads: Vec<[u32; 4]> = m.quads.iter().copied().filter(|q| q.iter().all(|&i| near[i as usize])).collect();
    let verts: Vec<usize> = (0..near.len()).filter(|&i| near[i]).collect();
    let nrms: Vec<Vec<Vector3<f64>>> = arms.iter().map(|h| normals(&h.skin, &quads)).collect();
    let upper_share = elbow::owned(&b, &["upperarm01.R", "upperarm02.R"]);
    let fore_share = elbow::owned(&b, &["lowerarm01.R", "lowerarm02.R", "olecranon.R"]);
    // The strip across the elbow (along its axis): the skin within 60 mm of the joint, out by the
    // foam and the wall either side.
    let xs: Vec<f64> = verts.iter().filter(|&&i| (a0.skin[i] - el).dot(&ap).abs() < 60.0).map(|&i| (a0.skin[i] - el).dot(&w)).collect();
    let pad_x = p.foam + p.wall;
    let (x0, x1) = (xs.iter().copied().fold(f64::MAX, f64::min) - pad_x + el.coords.dot(&w), xs.iter().copied().fold(f64::MIN, f64::max) + pad_x + el.coords.dot(&w));
    // The skin's distance, built pose, signed by the arm's piece closed at its ends.
    let mut closed = quad_mesh(&ab.skin, &quads);
    auto_cap_open_boundaries(&mut closed);
    let sdf: Sdf = Arc::new(Signed {
        distance: TriMeshDistance::new(quad_mesh(&ab.skin, &quads)).map_err(|e| format!("skin distance: {e}"))?,
        sign: ParitySign::new(&closed).map_err(|e| format!("arm inside test: {e}"))?,
    });
    {
        let inside = sdf.evaluate(Point3::from(at(ab, "lowerarm01.R").coords + (at(ab, "wrist.R") - at(ab, "lowerarm01.R")) * 0.3));
        if inside >= 0.0 {
            return Err(format!("the skin's signed distance is the wrong way round: {inside:.1} inside the forearm"));
        }
    }

    // The pieces: the upper arm's shell, the scales, the cap; their motions.
    let n = p.scales + 1;
    let mut names = vec!["upper arm shell".to_string()];
    let mut movers = vec![Mover::Bone(ub)];
    for s in 1..=p.scales {
        names.push(format!("scale {s}"));
        movers.push(Mover::Scale { a: ub, b: fb, c: el, frac: s as f64 / n as f64 });
    }
    names.push("cap".to_string());
    movers.push(Mover::Bone(fb));
    let n_p = names.len();
    let mo = |h: &ArmAt, i: usize| motion_of(&h.mm, &a0.mm, &movers[i]);
    let pivot = {
        let hf = &arms[last];
        let ab_rel = hf.mm[ub].after(&a0.mm[ub].inverse()).inverse().after(&hf.mm[fb].after(&a0.mm[fb].inverse()));
        (Point3::from(ab_rel.apply(el.coords)) - el).norm()
    };
    if pivot > 0.1 {
        return Err(format!("the forearm turns about a point {pivot:.2} mm off the elbow joint"));
    }

    // The laps: each covering an equal share of the elbow's full bend and the margin; each radius
    // clear of the liner (the skin out by the foam) seen from its two pieces at every step, within
    // its reach either side and across the strip.
    // Each lap's span is split either side of the elbow's back where its radius comes out least:
    // the back of the upper arm (the triceps) stands further behind the elbow's axis than the
    // olecranon does, so a lap reaching evenly either side had to clear it (round 1, run 1: the
    // outer lap's radius 64.6 mm).
    let span = (ELBOW_FLEXION / n as f64 + p.margin).to_radians();
    let ang = |q: &Point3<f64>| {
        let dq = q - el;
        dq.dot(&ap).atan2(dq.dot(&d))
    };
    // The liner's points each lap must clear, seen from its two pieces at every step: (angle about
    // the axis from the elbow's back, radius).
    let seen = |a: usize, b: usize| -> Vec<(f64, f64)> {
        let mut out = Vec::new();
        for (ti, h) in arms.iter().enumerate() {
            for piece in [a, b] {
                let back = mo(h, piece).inverse();
                for &i in &verts {
                    let q = Point3::from(back.apply((h.skin[i] + nrms[ti][i] * p.foam).coords));
                    let x = q.coords.dot(&w);
                    if (q - el).dot(&d) < -p.down || x < x0 || x > x1 {
                        continue;
                    }
                    out.push((ang(&q), Rev::radius(&el, &w, &q)));
                }
            }
        }
        out
    };
    let ext = EXT.to_radians();
    let rho_at = |pts: &[(f64, f64)], alpha: f64, beta: f64| pts.iter().filter(|(psi, _)| (-beta - ext..=alpha + ext).contains(psi)).map(|(_, r)| *r).fold(0.0f64, f64::max);
    // The splits are chosen together, nested. With the arm straight (where the pieces stand most
    // over each other: bending turns each inner piece forward, away), every piece's back edge
    // must lie within the upper arm's shell's lap (past it, the shell follows the triceps), and
    // the shell's front edge within the cap's (past it, the cap follows the forearm): every lap's
    // reach back within EXT of the shell's lap's and the cap's lap's. Split lap by lap, the inner
    // laps reached further back than the shell's and bound under it, 1.0 mm (round 1, run 2).
    let over = p.wall + p.clear;
    const GRID: usize = 40;
    let betas: Vec<f64> = (1..GRID).map(|k| span * k as f64 / GRID as f64).collect();
    let pts: Vec<Vec<(f64, f64)>> = (0..n).map(|s| seen(s, s + 1)).collect();
    let table: Vec<Vec<f64>> = pts.iter().map(|pt| betas.iter().map(|&b| rho_at(pt, span - b, b)).collect()).collect();
    let even: Vec<f64> = pts.iter().map(|pt| rho_at(pt, 0.5 * span, 0.5 * span)).collect();
    let mut best: Option<(f64, Vec<usize>)> = None;
    let mut ks = vec![0usize; n];
    loop {
        let bs: Vec<f64> = ks.iter().map(|&k| betas[k]).collect();
        let ok = (1..n).all(|k| bs[k] <= bs[0] + ext) && (0..n - 1).all(|j| bs[n - 1] <= bs[j] + ext);
        if ok {
            let mut rho = table[n - 1][ks[n - 1]];
            for s in (0..n - 1).rev() {
                rho = table[s][ks[s]].max(rho + over);
            }
            if best.as_ref().is_none_or(|b| rho < b.0) {
                best = Some((rho, ks.clone()));
            }
        }
        // The next combination.
        let mut i = 0;
        while i < n {
            ks[i] += 1;
            if ks[i] < betas.len() {
                break;
            }
            ks[i] = 0;
            i += 1;
        }
        if i == n {
            break;
        }
    }
    let (_, ks) = best.ok_or("no split of the laps nests")?;
    let mut laps: Vec<Lap> = (0..n).map(|s| Lap { alpha: span - betas[ks[s]], beta: betas[ks[s]], rho: table[s][ks[s]] }).collect();
    let raw_rho: Vec<f64> = laps.iter().map(|l| l.rho).collect();
    // Nested over the one axis: each over piece a wall and the clearance outside the one under it.
    for s in (0..n - 1).rev() {
        laps[s].rho = laps[s].rho.max(laps[s + 1].rho + over);
    }

    // The pieces' shapes.
    let front = |a: f64| (el, d * a.sin() - ap * a.cos());
    let back = |a: f64| (el, d * a.sin() + ap * a.cos());
    let low = 0.5 * (ELBOW_FLEXION + EXT).to_radians();
    let zone_of = |l: &Lap, r: f64| LapZone { rev: Rev { c: el, w, r }, d, ap, lo: -l.beta - EXT.to_radians(), hi: l.alpha + EXT.to_radians(), x0, x1 };
    let reach = p.up_len.max(p.cap_len) + 60.0;
    let lo = el - Vector3::repeat(reach);
    let hi = el + Vector3::repeat(reach);
    let pieces: Vec<Piece> = (0..n_p)
        .map(|i| {
            let (halves, lz) = if i == 0 {
                let l = &laps[0];
                (vec![front(l.alpha), front(low.max(l.alpha)), (el + up * p.up_len, -up)], vec![zone_of(l, l.rho + over)])
            } else if i == n_p - 1 {
                let l = &laps[n - 1];
                (vec![back(l.beta), back(low.max(l.beta)), (el + ap * p.cap_len, -ap)], vec![zone_of(l, l.rho)])
            } else {
                let (lb, la) = (&laps[i - 1], &laps[i]);
                (vec![back(lb.beta), front(la.alpha)], vec![zone_of(lb, lb.rho), zone_of(la, la.rho + over)])
            };
            Piece {
                name: names[i].clone(),
                skin: sdf.clone(),
                to_build: mo(ab, i),
                halves,
                band: Some((x0, x1)),
                zones: Vec::new(),
                e: w,
                laps: lz,
                down: p.down,
                lo,
                hi,
                foam: p.foam,
                wall: p.wall,
            }
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
    for (i, mm) in meshes.iter().enumerate() {
        if mm.faces.is_empty() {
            return Err(format!("{}: empty", names[i]));
        }
    }
    let grams: Vec<f64> = meshes.iter().map(|mm| mesh_volume(mm).abs() / 1000.0 * PA12_MJF.density / 1000.0).collect();
    println!("  meshed {n_p} pieces ({:.0} s)", t1.elapsed().as_secs_f64());

    // The stops: each lap opens at most `theta_m`, and closes no further than nested (as the
    // pieces lie with the arm straight). A piece turned `c` (rad) from the upper arm, the elbow
    // bent `phi`; the pieces' turns for a split of the bend over the laps; the even split.
    let theta_m = span - p.stop_overlap.to_radians();
    let turned = |i: usize, c: f64, phi: f64| if i == 0 || i == n_p - 1 || phi <= 1e-9 { movers[i] } else { Mover::Scale { a: ub, b: fb, c: el, frac: c / phi } };
    let at_turns = |h: &ArmAt, phi: f64, cs: &[f64]| -> Vec<Motion> { (0..n_p).map(|i| motion_of(&h.mm, &a0.mm, &turned(i, cs[i], phi))).collect() };
    let cum = |th: &[f64]| {
        th.iter().fold(vec![0.0], |mut c, t| {
            c.push(c[c.len() - 1] + t);
            c
        })
    };
    let rels: Vec<Vec<Motion>> = arms.iter().map(|h| (0..n_p).map(|i| mo(h, i)).collect()).collect();
    let posed: Vec<Vec<IndexedMesh>> = rels.iter().map(|rs| meshes.iter().zip(rs).map(|(mm, r)| moved(mm, r)).collect()).collect();

    // Check 1: the laps stay closed, at every split of the bend the stops allow. From the joint's
    // centre (and across the strip), rays over the elbow's back, from where the first under
    // piece's edge is straight to where the cap's is at this step; each must meet a piece within
    // 80 mm.
    let cfgs: Vec<(usize, Vec<f64>)> = (0..arms.len()).flat_map(|ti| shares(phis[ti], n, theta_m).into_iter().map(move |th| (ti, th))).collect();
    let gaps: Vec<f64> = par_map(&cfgs, threads, |(ti, th)| {
        let rs = at_turns(&arms[*ti], phis[*ti], &cum(th));
        let to: Vec<Motion> = (0..n_p).map(|i| rs[i].inverse().after(&rs[0])).collect();
        let step = 1.5f64.to_radians();
        let k = ((laps[0].beta + laps[n - 1].alpha + phis[*ti]) / step).ceil() as usize;
        let mut worst = 0.0f64;
        let half = 0.5 * (x1 - x0) - pad_x;
        for frac in [-0.8, -0.4, 0.0, 0.4, 0.8] {
            let o = el + w * (frac * half + (0.5 * (x0 + x1) - el.coords.dot(&w)));
            let (mut run, mut best) = (0usize, 0usize);
            for j in 0..=k {
                let psi = -laps[0].beta + j as f64 * step;
                let dir = d * psi.cos() + ap * psi.sin();
                let hit = (0..=160).any(|r| {
                    let q = o + dir * (0.5 * r as f64);
                    (0..n_p).any(|i| pieces[i].eval(Point3::from(to[i].apply(q.coords))) < 0.0)
                });
                if hit {
                    best = best.max(run);
                    run = 0;
                } else {
                    run += 1;
                }
            }
            worst = worst.max(best.max(run) as f64 * 1.5);
        }
        worst
    });
    println!("  check 1: {} splits ({:.0} s)", cfgs.len(), t0.elapsed().as_secs_f64());

    // Check 2: binding, every two pieces over every turn between them the stops allow (from
    // nested to theirs laps' stops, or the elbow's full bend). Coaxial, they meet the same
    // whatever the arm does, so each pair is turned on its own.
    let phi_l = phis[last];
    let turn = |g: f64| motion_of(&arms[last].mm, &a0.mm, &Mover::Scale { a: ub, b: fb, c: el, frac: g / phi_l });
    let r0 = turn(0.0);
    let gstep = 3.0f64.to_radians();
    let mut items: Vec<(usize, usize, f64)> = Vec::new();
    for i in 0..n_p {
        for j in i + 1..n_p {
            let gmax = ((j - i) as f64 * theta_m).min(phi_l);
            let k = (gmax / gstep).ceil() as usize;
            items.extend((0..=k).map(|s| (i, j, (s as f64 * gstep).min(gmax))));
        }
    }
    let binds: Vec<(f64, usize)> = par_map(&items, threads, |&(i, j, g)| {
        let to_j = turn(g).inverse().after(&r0);
        let to_i = to_j.inverse();
        let a = meshes[i].vertices.iter().step_by(2).map(|v| pieces[j].eval(Point3::from(to_j.apply(v.coords))));
        let bb = meshes[j].vertices.iter().step_by(2).map(|v| pieces[i].eval(Point3::from(to_i.apply(v.coords))));
        a.chain(bb).fold((f64::MAX, 0usize), |acc, x| (acc.0.min(x), acc.1 + usize::from(x < -0.3)))
    });
    println!("  check 2: {} turns ({:.0} s)", items.len(), t0.elapsed().as_secs_f64());

    // The room over the liner: from each point of the skin behind the elbow's axis, out along its
    // normal from the liner's outside, how far to each piece (negative: how far it is already
    // in); the scales at their even turn and across every turn the stops allow at this step.
    let stand = |pc: &Piece, back: &Motion, liner: &Point3<f64>, nv: &Vector3<f64>| -> Option<f64> {
        let (q, nq) = (Point3::from(back.apply(liner.coords)), back.r * nv);
        if pc.eval(q) < 0.0 {
            let r = (1..=((p.foam / 0.25) as usize)).map(|s| s as f64 * 0.25).find(|&r| pc.eval(q - nq * r) >= 0.0).unwrap_or(p.foam);
            return Some(-r);
        }
        let (mut a, mut r) = (0.0, 1.0);
        while r <= REACH {
            if pc.eval(q + nq * r) < 0.0 {
                let mut bb = r;
                for _ in 0..6 {
                    let mid = 0.5 * (a + bb);
                    if pc.eval(q + nq * mid) < 0.0 {
                        bb = mid;
                    } else {
                        a = mid;
                    }
                }
                return Some(0.5 * (a + bb));
            }
            a = r;
            r += 1.0;
        }
        None
    };
    let cand: Vec<usize> = verts.iter().copied().filter(|&i| (a0.skin[i] - el).dot(&d) > -(p.down + 15.0)).collect();
    struct Room {
        /// Each piece at its even turn.
        even: Vec<Option<f64>>,
        /// The least over every piece and turn.
        least: Option<f64>,
    }
    let rooms: Vec<Vec<Room>> = (0..arms.len())
        .map(|ti| {
            let (h, phi) = (&arms[ti], phis[ti]);
            let mut ms: Vec<(usize, Motion, bool)> = Vec::new();
            for i in 0..n_p {
                let even = i as f64 * phi / n as f64;
                ms.push((i, motion_of(&h.mm, &a0.mm, &turned(i, even, phi)).inverse(), true));
                if i > 0 && i < n_p - 1 {
                    let (lo_c, hi_c) = ((phi - (n - i) as f64 * theta_m).max(0.0), phi.min(i as f64 * theta_m));
                    for k in 0..TURNS {
                        let c = lo_c + (hi_c - lo_c) * k as f64 / (TURNS - 1) as f64;
                        if (c - even).abs() > 1e-4 && hi_c > lo_c + 1e-4 {
                            ms.push((i, motion_of(&h.mm, &a0.mm, &turned(i, c, phi)).inverse(), false));
                        }
                    }
                }
            }
            par_map(&cand, threads, |&v| {
                let nv = nrms[ti][v];
                let mut room = Room { even: vec![None; n_p], least: None };
                if nv.norm() == 0.0 {
                    return room;
                }
                let liner = h.skin[v] + nv * p.foam;
                for (i, back, is_even) in &ms {
                    if let Some(x) = stand(&pieces[*i], back, &liner, &nv) {
                        if *is_even {
                            room.even[*i] = Some(x);
                        }
                        room.least = Some(room.least.map_or(x, |l| l.min(x)));
                    }
                }
                room
            })
        })
        .collect();
    println!("  room over the liner ({:.0} s)", t0.elapsed().as_secs_f64());

    // The patch: where the pad covers the skin at every step, the least room over it, less the
    // clearance (`even`: the scales held at even turns, as round 1, for what the stops cost).
    // Point by point it jumps where neighbouring rays meet different pieces or edges, so it is
    // then eroded: each point no thicker than any within `ERODE` mm of it, straight.
    const ERODE: f64 = 5.0;
    let raw_patch = |even: bool| -> Vec<f64> {
        let mut t = vec![0.0f64; a0.skin.len()];
        for (k, &v) in cand.iter().enumerate() {
            let mut least = f64::MAX;
            let mut covered = true;
            for st in &rooms {
                let r = &st[k];
                if r.even.iter().all(|x| x.is_none()) {
                    covered = false;
                    break;
                }
                let l = if even { r.even.iter().flatten().copied().fold(f64::MAX, f64::min) } else { r.least.unwrap_or(f64::MAX) };
                least = least.min(l);
            }
            if covered {
                t[v] = (least - p.clear).max(0.0);
            }
        }
        t
    };
    let erode = |t: &[f64]| -> Vec<f64> {
        let key = |q: &Point3<f64>| ((q.x / ERODE).floor() as i64, (q.y / ERODE).floor() as i64, (q.z / ERODE).floor() as i64);
        let mut grid: std::collections::HashMap<(i64, i64, i64), Vec<usize>> = std::collections::HashMap::new();
        for &v in &cand {
            grid.entry(key(&a0.skin[v])).or_default().push(v);
        }
        let mut out = t.to_vec();
        for &v in &cand {
            let (kx, ky, kz) = key(&a0.skin[v]);
            let mut m = t[v];
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(js) = grid.get(&(kx + dx, ky + dy, kz + dz)) {
                            for &j in js {
                                if (a0.skin[j] - a0.skin[v]).norm() <= ERODE {
                                    m = m.min(t[j]);
                                }
                            }
                        }
                    }
                }
            }
            out[v] = m;
        }
        out
    };
    let patch_raw = raw_patch(false);
    let patch = erode(&patch_raw);
    let patch_even = erode(&raw_patch(true));
    const PATCH_MIN: f64 = 1.0;
    let quad_area = |pts: &[Point3<f64>], q: &[u32; 4]| {
        let [a, bq, c, dq] = q.map(|i| pts[i as usize]);
        0.5 * (c - a).cross(&(dq - bq)).norm()
    };
    // The patch's quads (every corner at least PATCH_MIN thick), those on the forearm's skin (it
    // moves with the cap: what could be glued in the cap instead), and the rest; area, volume.
    let quads_of = |t: &[f64], keep: &dyn Fn(&[u32; 4]) -> bool| -> Vec<[u32; 4]> { quads.iter().copied().filter(|q| q.iter().all(|&i| t[i as usize] >= PATCH_MIN) && keep(q)).collect() };
    let area_vol = |t: &[f64], qs: &[[u32; 4]]| -> (f64, f64) {
        (qs.iter().map(|q| quad_area(&ab.skin, q)).sum(), qs.iter().map(|q| quad_area(&ab.skin, q) * q.iter().map(|&i| t[i as usize]).sum::<f64>() / 4.0).sum())
    };
    let on_fore = |q: &[u32; 4]| q.iter().all(|&i| fore_share[i as usize] > 0.5);
    let patch_quads = quads_of(&patch, &|_| true);
    let fore_quads = quads_of(&patch, &on_fore);
    let rest_quads = quads_of(&patch, &|q| !on_fore(q));
    let (patch_area, patch_vol) = area_vol(&patch, &patch_quads);
    let (fore_area, fore_vol) = area_vol(&patch, &fore_quads);
    let (_, raw_vol) = area_vol(&patch_raw, &quads_of(&patch_raw, &|_| true));
    let (_, even_vol) = area_vol(&patch_even, &quads_of(&patch_even, &|_| true));
    let patch_g = patch_vol * LINER_FOAM.density * 1e-6;
    let patch_ts: Vec<f64> = cand.iter().map(|&v| patch[v]).filter(|&t| t >= PATCH_MIN).collect();
    let back_v = verts
        .iter()
        .copied()
        .filter(|&i| {
            let q = a0.skin[i] - el;
            q.dot(&ap).abs() < 3.0 && q.dot(&w).abs() < 8.0
        })
        .max_by(|&i, &j| (a0.skin[i] - el).dot(&d).partial_cmp(&(a0.skin[j] - el).dot(&d)).unwrap())
        .ok_or("no skin behind the joint")?;
    let height = (a0.skin[back_v] - el).dot(&d);
    // The suit's outside over the back of the elbow at a step: the liner, and the patch on it.
    let back_quads: Vec<[u32; 4]> = quads.iter().copied().filter(|q| q.iter().all(|&i| cand.binary_search(&(i as usize)).is_ok())).collect();
    let suit_at = |ti: usize, extra: bool, qs: &[[u32; 4]]| -> IndexedMesh {
        let pts: Vec<Point3<f64>> = (0..arms[ti].skin.len()).map(|i| arms[ti].skin[i] + nrms[ti][i] * (p.foam + if extra { patch[i] } else { 0.0 })).collect();
        quad_mesh(&pts, qs)
    };
    // How the patch stretches: its faces' length along the elbow's back (the plane it bends in),
    // all of it, the part on the forearm's skin, and the rest.
    let stretch_of = |qs: &[[u32; 4]]| -> Vec<(f64, f64)> { [0, BUILD, last].iter().map(|&ti| (cut_length(&suit_at(ti, false, qs), el, w, |_| true), cut_length(&suit_at(ti, true, qs), el, w, |_| true))).collect() };
    let stretch = stretch_of(&patch_quads);
    let stretch_fore = stretch_of(&fore_quads);
    let stretch_rest = stretch_of(&rest_quads);

    // Foam glued in the cap instead: the cap rides the olecranon and nothing slides under it, so
    // its foam need not stretch with the skin. From each point of its inside, in along its inward
    // normal, how far to the liner at each step (the arm's back moved into the cap's frame); its
    // foam the least of these less the clearance, eroded as the patch.
    let ci = n_p - 1;
    let cap = &pieces[ci];
    let cap_inner: Vec<(usize, Vector3<f64>)> = meshes[ci]
        .vertices
        .iter()
        .enumerate()
        .filter_map(|(k, v)| {
            if cap.envelope(v).abs() > 0.3 || cap.cut(v) > -0.5 {
                return None;
            }
            let h = 0.2;
            let e = |dv: Vector3<f64>| cap.envelope(&(v + dv * h)) - cap.envelope(&(v - dv * h));
            let g = Vector3::new(e(Vector3::x()), e(Vector3::y()), e(Vector3::z()));
            (g.norm() > 1e-9).then(|| (k, -g.normalize()))
        })
        .collect();
    let liner_d: Vec<TriMeshDistance> = (0..arms.len())
        .map(|ti| {
            let back = rels[ti][ci].inverse();
            let pts: Vec<Point3<f64>> = (0..arms[ti].skin.len()).map(|i| Point3::from(back.apply((arms[ti].skin[i] + nrms[ti][i] * p.foam).coords))).collect();
            TriMeshDistance::new(quad_mesh(&pts, &back_quads)).map_err(|e| format!("liner distance: {e}"))
        })
        .collect::<Result<_, _>>()?;
    let cap_rooms: Vec<Vec<Option<f64>>> = par_map(&cap_inner, threads, |&(k, dir)| {
        let o = meshes[ci].vertices[k];
        liner_d
            .iter()
            .map(|ld| {
                let mut r = 0.0;
                while r <= REACH {
                    let dist = ld.distance(o + dir * r);
                    if dist < 0.4 {
                        return Some(r);
                    }
                    r += (dist - 0.3).max(0.3);
                }
                None
            })
            .collect()
    });
    let cap_room: Vec<f64> = cap_rooms
        .iter()
        .map(|rs| {
            let least = rs.iter().flatten().copied().fold(f64::MAX, f64::min);
            if least == f64::MAX { 0.0 } else { (least - p.clear).max(0.0) }
        })
        .collect();
    let cap_foam: Vec<f64> = {
        let mut t = vec![0.0f64; meshes[ci].vertices.len()];
        for (&(k, _), &x) in cap_inner.iter().zip(&cap_room) {
            t[k] = x;
        }
        let vs = &meshes[ci].vertices;
        let key = |q: &Point3<f64>| ((q.x / ERODE).floor() as i64, (q.y / ERODE).floor() as i64, (q.z / ERODE).floor() as i64);
        let mut grid: std::collections::HashMap<(i64, i64, i64), Vec<usize>> = std::collections::HashMap::new();
        for &(k, _) in &cap_inner {
            grid.entry(key(&vs[k])).or_default().push(k);
        }
        let mut out = t.clone();
        for &(k, _) in &cap_inner {
            let (kx, ky, kz) = key(&vs[k]);
            let mut m = t[k];
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        for &j in grid.get(&(kx + dx, ky + dy, kz + dz)).map(|v| v.as_slice()).unwrap_or(&[]) {
                            if (vs[j] - vs[k]).norm() <= ERODE {
                                m = m.min(t[j]);
                            }
                        }
                    }
                }
            }
            out[k] = m;
        }
        out
    };
    let is_inner = {
        let mut f = vec![false; meshes[ci].vertices.len()];
        for &(k, _) in &cap_inner {
            f[k] = true;
        }
        f
    };
    let foam_faces: Vec<[u32; 3]> = meshes[ci].faces.iter().copied().filter(|f| f.iter().all(|&i| is_inner[i as usize] && cap_foam[i as usize] >= PATCH_MIN)).collect();
    let tri_area = |f: &[u32; 3]| {
        let [a, bq, c] = f.map(|i| meshes[ci].vertices[i as usize]);
        0.5 * (bq - a).cross(&(c - a)).norm()
    };
    let cap_foam_area: f64 = foam_faces.iter().map(tri_area).sum();
    let cap_foam_vol: f64 = foam_faces.iter().map(|f| tri_area(f) * f.iter().map(|&i| cap_foam[i as usize]).sum::<f64>() / 3.0).sum();
    let cap_foam_ts: Vec<f64> = cap_inner.iter().map(|&(k, _)| cap_foam[k]).filter(|&t| t >= PATCH_MIN).collect();
    // The cap's foam as a solid, its inside face moved in by its thickness (pose-0 frame).
    let cap_foam_mesh = {
        let mut dir = vec![Vector3::zeros(); meshes[ci].vertices.len()];
        for &(k, d_in) in &cap_inner {
            dir[k] = d_in;
        }
        let vertices = meshes[ci].vertices.iter().enumerate().map(|(k, v)| v + dir[k] * cap_foam[k]).collect();
        IndexedMesh { vertices, faces: foam_faces.clone() }
    };
    println!("  the cap's foam ({:.0} s)", t0.elapsed().as_secs_f64());

    // Check 3: the fit on the liner, the scales at their even turn. From each skin point under a
    // piece (the upper arm's under the shell, the forearm's under the cap, either under a scale),
    // how far the piece stands off the liner, and off the patch.
    let fit: Vec<Vec<(Vec<f64>, f64, Vec<f64>, f64)>> = rooms
        .iter()
        .map(|st| {
            let mut out = vec![(Vec::new(), 0.0f64, Vec::new(), 0.0f64); n_p];
            for (k, &v) in cand.iter().enumerate() {
                let owns = |i: usize| if i == 0 { upper_share[v] > 0.5 } else if i == n_p - 1 { fore_share[v] > 0.5 } else { true };
                let best = (0..n_p).filter(|&i| owns(i)).filter_map(|i| st[k].even[i].map(|x| (i, x))).min_by(|a, bb| a.1.partial_cmp(&bb.1).unwrap());
                if let Some((i, x)) = best {
                    let y = x - patch[v];
                    out[i].0.push(x.max(0.0));
                    out[i].1 = out[i].1.max(-x);
                    out[i].2.push(y.max(0.0));
                    out[i].3 = out[i].3.max(-y);
                }
            }
            out
        })
        .collect();

    // The straps: round the arm's front where the pieces leave it (past the elbow's axis by
    // `down`), the shell's up the upper arm, the cap's down the forearm. Their length (pose 0,
    // on the skin), and at each step how near the other part of the arm comes to it.
    let el_c = el;
    let strap_specs = [("upper arm (the shell's)", up, p.strap_up, &upper_share, &fore_share), ("forearm (the cap's)", ap, p.strap_down, &fore_share, &upper_share)];
    let arm0 = quad_mesh(&a0.skin, &quads);
    struct Strap {
        name: &'static str,
        at: f64,
        ring: Vec<usize>,
        length: f64,
        grams: f64,
        fold: Vec<f64>,
    }
    let ring_at = |axis: &Vector3<f64>, s: f64, own: &[f64]| -> Vec<usize> { verts.iter().copied().filter(|&i| own[i] > 0.5 && ((a0.skin[i] - el_c).dot(axis) - s).abs() < 0.5 * p.strap_w && (a0.skin[i] - el_c).dot(&d) < -p.down).collect() };
    let fold_of = |ring: &[usize], other: &[f64], steps: &[usize]| -> Vec<f64> {
        let others: Vec<usize> = verts.iter().copied().filter(|&i| other[i] > 0.5).collect();
        steps.iter().map(|&ti| ring.iter().map(|&i| others.iter().map(|&j| (arms[ti].skin[i] - arms[ti].skin[j]).norm()).fold(f64::MAX, f64::min)).fold(f64::MAX, f64::min)).collect()
    };
    let all_steps: Vec<usize> = (0..arms.len()).collect();
    let straps: Vec<Strap> = strap_specs
        .iter()
        .map(|&(name, axis, s, own, other)| {
            let ring = ring_at(&axis, s, own);
            let length = cut_length(&arm0, el_c + axis * s, axis, |q| (q - el_c).dot(&d) < -p.down);
            let fold = fold_of(&ring, other, &all_steps);
            let grams = length * p.strap_w * p.strap_t * STRAP.density * 1e-6;
            Strap { name, at: s, ring, length, grams, fold }
        })
        .collect();
    // Where a strap could go: how near the other part of the arm comes to a ring round the front,
    // its middle every 10 mm along each bone (past the pieces too), at 90, 110 and 130 deg.
    let fold_steps = [BUILD, BUILD + 1, BUILD + 2];
    let strap_scan: Vec<Vec<(f64, Vec<f64>)>> = strap_specs.iter().map(|&(_, axis, _, own, other)| (4..=14).map(|k| (10.0 * k as f64, fold_of(&ring_at(&axis, 10.0 * k as f64, own), other, &fold_steps))).collect()).collect();
    let strap_mesh = |ti: usize, s: &Strap| -> IndexedMesh {
        let qs: Vec<[u32; 4]> = quads.iter().copied().filter(|q| q.iter().all(|i| s.ring.binary_search(&(*i as usize)).is_ok())).collect();
        let pts: Vec<Point3<f64>> = (0..arms[ti].skin.len()).map(|i| arms[ti].skin[i] + nrms[ti][i] * (1.0 + p.strap_t)).collect();
        quad_mesh(&pts, &qs)
    };
    println!("  checked ({:.0} s)", t0.elapsed().as_secs_f64());

    // Report.
    let mut lines = Vec::new();
    let mut say = |s: String| {
        println!("{s}");
        lines.push(s);
    };
    let deg = |x: f64| x.to_degrees();
    say(format!("the elbow pad, round 2 (track U2): the right arm with an olecranon ({ole_n} skin control vertices move with it), built at the elbow's 90 deg ({})", scan.display()));
    say(format!(
        "  liner foam {:.0} mm, wall {:.0} mm (PA 12), {:.0} mm between sliding pieces and between a piece and the patch (placeholders); the upper arm's shell {:.0} mm up the arm, the cap {:.0} mm down the forearm, round the arm to {:.0} mm past the elbow's axis; {} scales between, each lap spanning {:.0} deg ({:.1} deg of bend each and a {:.0} deg margin), split either side of the elbow's back where its radius comes out least",
        p.foam,
        p.wall,
        p.clear,
        p.up_len,
        p.cap_len,
        p.down,
        p.scales,
        deg(span),
        ELBOW_FLEXION / n as f64,
        p.margin
    ));
    say(format!("  the sweep, the elbow's bend: {} deg (built at {:.0})", STEPS.iter().map(|s| format!("{s:.0}")).collect::<Vec<_>>().join(" / "), STEPS[BUILD]));
    let strap_g: f64 = straps.iter().map(|s| s.grams).sum();
    say(format!("  {} hard pieces, {:.1} g together (PA 12); with the straps {:.1} g:", n_p, grams.iter().sum::<f64>(), grams.iter().sum::<f64>() + strap_g));
    for i in 0..n_p {
        say(format!("    {:<16} {:>5.1} g", names[i], grams[i]));
    }
    let top = laps[0].rho + over + p.wall;
    say(format!(
        "  over the elbow, each lap (from the upper arm's shell): its reach back up the arm / forward over the forearm (deg), its under piece's inside radius (clear of the liner; split evenly it would need {}): {}; the skin {:.1} mm behind the joint's centre straight; the stack's top {:.1} mm from the axis, {:.1} mm proud of the pieces either side (the liner and a wall over the skin)",
        even.iter().map(|r| format!("{r:.1}")).collect::<Vec<_>>().join(" / "),
        laps.iter().zip(&raw_rho).map(|(l, r)| format!("{:.0} / {:.0}: {:.1} mm{}", deg(l.beta), deg(l.alpha), l.rho, if l.rho > *r + 0.05 { format!(" (raised from {r:.1} to nest)") } else { String::new() })).collect::<Vec<_>>().join("; "),
        height,
        top,
        top - (height + p.foam + p.wall)
    ));
    say(format!(
        "  the stops: each lap turns from nested (0) to at most {:.1} deg (its span less {:.0} deg still overlapping), so the {} laps take up to {:.1} deg of bend between them, the elbow's {:.1} and {:.1} to spare; the scales at any split of the bend within that",
        deg(theta_m),
        p.stop_overlap,
        n,
        deg(theta_m) * n as f64,
        ELBOW_FLEXION,
        deg(theta_m) * n as f64 - ELBOW_FLEXION
    ));
    say("  check 1, laps closed: the widest gap seen over the elbow's back (deg) at each step, over the splits the stops allow (how many: the even one and the extremes):".into());
    let mut per_step = vec![(0.0f64, 0usize, 0usize); arms.len()];
    for (k, (ti, _)) in cfgs.iter().enumerate() {
        let g = &mut per_step[*ti];
        g.2 += 1;
        if gaps[k] > g.0 {
            (g.0, g.1) = (gaps[k], k);
        }
    }
    say(format!(
        "    {}",
        per_step
            .iter()
            .enumerate()
            .map(|(ti, (g, k, c))| if *g > 0.0 { format!("{:.0}: {g:.1} ({c}; at {})", STEPS[ti], cfgs[*k].1.iter().map(|t| format!("{:.0}", deg(*t))).collect::<Vec<_>>().join("/")) } else { format!("{:.0}: 0 ({c})", STEPS[ti]) })
            .collect::<Vec<_>>()
            .join(", ")
    ));
    say(format!("  check 2, binding: each two pieces turned from nested to the most the stops allow between them, every {:.0} deg: the deepest one runs into the other (mm), or how close they come, and at what turn:", deg(gstep)));
    for i in 0..n_p {
        for j in i + 1..n_p {
            let mine: Vec<(f64, (f64, usize))> = items.iter().zip(&binds).filter(|((a, bb, _), _)| *a == i && *bb == j).map(|((_, _, g), r)| (*g, *r)).collect();
            let gmax = mine.iter().map(|x| x.0).fold(0.0, f64::max);
            let (g, (v, cnt)) = mine.iter().copied().min_by(|a, bb| a.1.0.partial_cmp(&bb.1.0).unwrap()).unwrap();
            say(format!(
                "    {} / {} (0-{:.0} deg): {}",
                names[i],
                names[j],
                deg(gmax),
                if v < 0.0 { format!("{:.1} mm in at {:.0} deg ({cnt} surface points over 0.3 mm in)", -v, deg(g)) } else { format!("none; closest {:.1} mm at {:.0} deg", v, deg(g)) }
            ));
        }
    }
    let stats = |v: &[f64]| {
        if v.is_empty() {
            return (f64::NAN, f64::NAN);
        }
        let mut s = v.to_vec();
        s.sort_by(|a, bb| a.partial_cmp(bb).unwrap());
        (s.iter().sum::<f64>() / s.len() as f64, s[(0.9 * (s.len() - 1) as f64) as usize])
    };
    let (pt_mean, _) = stats(&patch_ts);
    say(format!(
        "  the patch, the suit's foam thicker behind the elbow: at each point the least room over it through the bend and every turn of the scales the stops allow, less {:.0} mm, then no thicker than any point within {ERODE:.0} mm of it (point by point, {:.0} cm3); where at least {:.0} mm: {:.0} cm2 of the skin, up to {:.1} mm more foam, {:.1} on average, {:.1} at the back of the elbow; {:.0} cm3, {:.1} g (liner foam, {:.0} kg/m3 assumed); with the scales held at even turns instead, {:.0} cm3",
        p.clear,
        raw_vol / 1000.0,
        PATCH_MIN,
        patch_area / 100.0,
        patch_ts.iter().copied().fold(0.0, f64::max),
        pt_mean,
        patch[back_v],
        patch_vol / 1000.0,
        patch_g,
        LINER_FOAM.density,
        even_vol / 1000.0
    ));
    say(format!("    of it on the forearm's skin (which moves with the cap): {:.0} cm2, {:.0} cm3", fore_area / 100.0, fore_vol / 1000.0));
    let (cf_mean, _) = stats(&cap_foam_ts);
    // How far the cap's foam rides off the liner at each step, where it is.
    let cap_gaps: Vec<(f64, f64)> = (0..arms.len())
        .map(|ti| {
            let v: Vec<f64> = cap_inner.iter().zip(&cap_rooms).filter(|((k, _), _)| cap_foam[*k] >= PATCH_MIN).filter_map(|((k, _), rs)| rs[ti].map(|r| r - cap_foam[*k])).collect();
            stats(&v)
        })
        .collect();
    say(format!(
        "  the cap's own foam instead (glued in it: the cap rides the olecranon and nothing slides under it, so its foam need not stretch): at each point of the cap's inside the least room in to the liner through the bend, less {:.0} mm, eroded as the patch; where at least {:.0} mm: {:.0} cm2 of its inside, up to {:.1} mm, {:.1} on average; {:.0} cm3, {:.1} g (liner foam, assumed); the pad with it and the straps, {:.1} g",
        p.clear,
        PATCH_MIN,
        cap_foam_area / 100.0,
        cap_foam_ts.iter().copied().fold(0.0, f64::max),
        cf_mean,
        cap_foam_vol / 1000.0,
        cap_foam_vol * LINER_FOAM.density * 1e-6,
        grams.iter().sum::<f64>() + strap_g + cap_foam_vol * LINER_FOAM.density * 1e-6
    ));
    say(format!("    the cap's foam off the liner, mean / 90th percentile (mm), at each step: {}", STEPS.iter().zip(&cap_gaps).map(|(s, (m, q))| format!("{s:.0}: {m:.1}/{q:.1}")).collect::<Vec<_>>().join(", ")));
    let st = |s: &[(f64, f64)]| {
        if s[0].0 < 1.0 {
            return "not cut".to_string();
        }
        format!(
            "{}, its inside lengthening {:.1} mm ({:.0}%), its outside {:.1} ({:.0}%)",
            s.iter().map(|(a, bb)| format!("{a:.1} / {bb:.1}")).collect::<Vec<_>>().join(", "),
            s[2].0 - s[0].0,
            100.0 * (s[2].0 / s[0].0 - 1.0),
            s[2].1 - s[0].1,
            100.0 * (s[2].1 / s[0].1 - 1.0)
        )
    };
    say(format!(
        "    along the elbow's back (the plane it bends in), the patch's inside on the liner / its outside (mm), straight, at 90 and at full bend: all of it {}; on the forearm's skin {}; the rest {}",
        st(&stretch),
        st(&stretch_fore),
        st(&stretch_rest)
    ));
    say(format!("  check 3, the fit (the scales at their even turn): how far each piece stands off the liner (and off the patch), mean / 90th percentile, and the most it presses in (mm), looking out {REACH:.0} mm, at each step:"));
    say(format!("    {:<26} {}", "", STEPS.iter().map(|s| format!("{:>15}", format!("{s:.0}"))).collect::<Vec<_>>().join("")));
    for (label, pick) in [("off the liner", false), ("off the patch", true)] {
        for i in 0..n_p {
            say(format!(
                "    {:<26} {}",
                format!("{} {label}", names[i]),
                fit.iter()
                    .map(|st| {
                        let (vals, press) = if pick { (&st[i].2, st[i].3) } else { (&st[i].0, st[i].1) };
                        let (mean, p90) = stats(vals);
                        format!("{:>15}", if vals.is_empty() { "-".to_string() } else { format!("{mean:.1}/{p90:.1} +{press:.1}") })
                    })
                    .collect::<Vec<_>>()
                    .join("")
            ));
        }
    }
    say(format!("  the straps, {:.0} mm wide, {:.1} mm thick (elastic with hook-and-loop, {:.0} kg/m3 assumed), round the arm's front from one side of the piece to the other:", p.strap_w, p.strap_t, STRAP.density));
    for s in &straps {
        say(format!(
            "    {} strap, {:.0} mm from the joint: {:.0} mm long on the skin, {:.1} g; the other part of the arm's skin comes within (mm), at each step: {}",
            s.name,
            s.at,
            s.length,
            s.grams,
            STEPS.iter().zip(&s.fold).map(|(a, f)| format!("{a:.0}: {f:.0}")).collect::<Vec<_>>().join(", ")
        ));
    }
    for (spec, scan) in strap_specs.iter().zip(&strap_scan) {
        say(format!(
            "    a strap round the {} at other places, its middle from the joint: the other part's skin within (mm) at {} deg: {}",
            spec.0,
            fold_steps.iter().map(|&t| format!("{:.0}", STEPS[t])).collect::<Vec<_>>().join(" / "),
            scan.iter().map(|(s, f)| format!("{s:.0}: {}", f.iter().map(|x| format!("{x:.0}")).collect::<Vec<_>>().join("/"))).collect::<Vec<_>>().join(", ")
        ));
    }
    // The splits pad_stops.png shows, the elbow at 90: as far toward the cap's end as the stops
    // allow (the shell's lap at its stop first), even, and as far toward the shell's.
    let splits: Vec<Vec<f64>> = {
        let phi = phis[BUILD];
        let fill = |order: Vec<usize>| {
            let (mut th, mut left) = (vec![0.0; n], phi);
            for k in order {
                th[k] = left.min(theta_m);
                left -= th[k];
            }
            th
        };
        vec![fill((0..n).collect()), vec![phi / n as f64; n], fill((0..n).rev().collect())]
    };
    say(format!(
        "  pad_stops.png, the elbow at 90, each lap's turn (deg, from the shell's): {}",
        splits.iter().map(|th| th.iter().map(|t| format!("{:.1}", deg(*t))).collect::<Vec<_>>().join(" / ")).collect::<Vec<_>>().join("; ")
    ));
    std::fs::write(docs.join("summary.txt"), lines.join("\n") + "\n").map_err(|e| e.to_string())?;
    for (i, mm) in posed[BUILD].iter().enumerate() {
        save_mesh(mm, out.join(format!("{i:02}_{}.stl", names[i].replace(' ', "_")))).map_err(|e| e.to_string())?;
    }

    // Pictures: straight, 90 and full, seen along the elbow's axis; the same cut in the plane the
    // arm bends in, with the suit's foam and the patch.
    let skin_c = [0.85, 0.70, 0.60];
    let foam_c = [0.90, 0.55, 0.20];
    let strap_c = [0.25, 0.25, 0.28];
    let cols = |i: usize| if i == 0 || i == n_p - 1 { [0.16, 0.20, 0.40] } else { [0.58, 0.70, 0.92] };
    let forward = -d;
    let shift_of = |slot: usize| Motion { r: nalgebra::Rotation3::identity(), t: forward * (330.0 * slot as f64) - el.coords };
    let mut solid: Vec<(IndexedMesh, [f64; 3])> = Vec::new();
    let mut cut: Vec<(IndexedMesh, [f64; 3])> = Vec::new();
    for (slot, ti) in [0usize, BUILD, last].into_iter().enumerate() {
        let shift = shift_of(slot);
        let arm = quad_mesh(&arms[ti].skin, &quads);
        solid.push((moved(&arm, &shift), skin_c));
        cut.push((moved(&section_lines(&arm, el, w, 1.0), &shift), [0.6, 0.4, 0.3]));
        cut.push((moved(&section_lines(&suit_at(ti, false, &back_quads), el, w, 1.0), &shift), foam_c));
        cut.push((moved(&section_lines(&moved(&cap_foam_mesh, &rels[ti][ci]), el, w, 1.0), &shift), foam_c));
        for s in &straps {
            solid.push((moved(&strap_mesh(ti, s), &shift), strap_c));
        }
        for i in 0..n_p {
            solid.push((moved(&posed[ti][i], &shift), cols(i)));
            cut.push((moved(&section_lines(&posed[ti][i], el, w, 1.0), &shift), cols(i)));
        }
    }
    let view = [View::looking(-w, up)];
    let layers: Vec<Layer> = solid.iter().map(|(mm, c)| Layer::solid(mm, *c)).collect();
    draw(&layers, &view, 1.5, &docs.join("pad.png"), Style::pretty())?;
    let layers: Vec<Layer> = cut.iter().map(|(mm, c)| Layer::solid(mm, *c)).collect();
    draw(&layers, &view, 2.0, &docs.join("pad_section.png"), Style::pretty())?;
    // The pad as built (the elbow at 90), with its straps: from the side, and from behind either
    // side.
    let views = [View::looking(-w, up), View::looking((-d - w).normalize(), up), View::looking((-d + w).normalize(), up)];
    {
        let arm = quad_mesh(&ab.skin, &quads);
        let sm: Vec<IndexedMesh> = straps.iter().map(|s| strap_mesh(BUILD, s)).collect();
        let mut layers = vec![Layer::solid(&arm, skin_c)];
        layers.extend(sm.iter().map(|mm| Layer::solid(mm, strap_c)));
        layers.extend((0..n_p).map(|i| Layer::solid(&posed[BUILD][i], cols(i))));
        draw(&layers, &views, 1.5, &docs.join("pad_90.png"), Style::pretty())?;
    }
    // The patch on the arm at 90, the pieces off: the suit's foam behind the elbow, darker where
    // the patch is thicker (1-5, 5-10, 10-15, over 15 mm).
    {
        let arm = quad_mesh(&ab.skin, &quads);
        let bands = [(0.0, PATCH_MIN, [0.96, 0.88, 0.72]), (PATCH_MIN, 5.0, [0.95, 0.75, 0.45]), (5.0, 10.0, [0.90, 0.58, 0.25]), (10.0, 15.0, [0.78, 0.40, 0.12]), (15.0, f64::MAX, [0.55, 0.25, 0.08])];
        let meshes_b: Vec<(IndexedMesh, [f64; 3])> = bands
            .iter()
            .map(|&(lo_t, hi_t, c)| {
                let qs: Vec<[u32; 4]> = back_quads
                    .iter()
                    .copied()
                    .filter(|q| {
                        let t = q.iter().map(|&i| patch[i as usize]).sum::<f64>() / 4.0;
                        t >= lo_t && t < hi_t
                    })
                    .collect();
                (suit_at(BUILD, true, &qs), c)
            })
            .collect();
        let mut layers = vec![Layer::solid(&arm, skin_c)];
        layers.extend(meshes_b.iter().map(|(mm, c)| Layer::solid(mm, *c)));
        draw(&layers, &views, 1.5, &docs.join("patch.png"), Style::pretty())?;
    }
    // The stops: at 90, the scales turned as far toward the cap's end as the stops allow, at
    // even shares, and as far toward the shell's, cut in the plane the arm bends in.
    {
        let phi = phis[BUILD];
        let mut cut: Vec<(IndexedMesh, [f64; 3])> = Vec::new();
        for (slot, th) in splits.iter().enumerate() {
            let shift = shift_of(slot);
            let rs = at_turns(ab, phi, &cum(th));
            let arm = quad_mesh(&ab.skin, &quads);
            cut.push((moved(&section_lines(&arm, el, w, 1.0), &shift), [0.6, 0.4, 0.3]));
            cut.push((moved(&section_lines(&suit_at(BUILD, false, &back_quads), el, w, 1.0), &shift), foam_c));
            cut.push((moved(&section_lines(&moved(&cap_foam_mesh, &rs[ci]), el, w, 1.0), &shift), foam_c));
            for i in 0..n_p {
                cut.push((moved(&section_lines(&moved(&meshes[i], &rs[i]), el, w, 1.0), &shift), cols(i)));
            }
        }
        let layers: Vec<Layer> = cut.iter().map(|(mm, c)| Layer::solid(mm, *c)).collect();
        draw(&layers, &view, 2.0, &docs.join("pad_stops.png"), Style::pretty())?;
    }
    println!("drew {}/pad.png, pad_section.png, pad_90.png, patch.png, pad_stops.png; pieces in {} ({:.0} s)", docs.display(), out.display(), t0.elapsed().as_secs_f64());
    Ok(())
}
