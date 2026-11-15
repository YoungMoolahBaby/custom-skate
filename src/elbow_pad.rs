//! U2, the elbow pad, round 1 (2026-11-14): the knee's design at the elbow (PLAN.md, the elbow
//! pad's first step). Hard pieces over a foam liner (in the compression suit's sleeve), on the
//! stand-in arm with an olecranon (`elbow::add_olecranon`): a shell over the back of the upper arm
//! and a cap over the end of the forearm (the olecranon), with scales between. Over the elbow
//! each is a cylinder about its axis, the piece nearer the shoulder over the next, so they slide
//! without binding (the glove's laps, `glove`): each lap turns a share of the elbow's bend, a
//! scale halfway between its neighbours. Built on the arm as it holds the stick (the elbow at
//! 90 deg, U2), checked from straight to the elbow's full flexion: the laps closed over the
//! elbow's back, the pieces clear of each other, and how each sits on the liner.
//!
//! Frame: the foot's (x forward, y left, z up, mm). Each piece is defined where it is with the
//! arm straight ("pose 0") and moves with its bone.

use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

use cortenforge::cf_design::{Aabb, Solid};
use cortenforge::cf_scan_prep_core::auto_cap_open_boundaries;
use cortenforge::mesh::io::save_mesh;
use cortenforge::mesh::sdf::{ParitySign, Signed, TriMeshDistance};
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::body;
use crate::elbow::{self, ARM_BONES, ELBOW_FLEXION};
use crate::glove::{EXT, LapZone, Mover, Piece, Rev, Sdf, motion_of, moved, normals, par_map, quad_mesh, section_lines};
use crate::hockey_boot::mesh_volume;
use crate::materials::PA12_MJF;
use crate::pad::draw;
use crate::pose::Motion;
use crate::render::{Layer, Style, View};
use crate::round1::mesh_of;
use crate::upper;

/// Round 1's sizes, mm (placeholders where marked).
#[derive(Clone, Copy, Debug)]
pub struct PadParams {
    /// The liner's foam on the skin (the knee's liner, `pad::PadParams::knee_liner`; a
    /// placeholder).
    pub foam: f64,
    /// The hard pieces' wall, PA 12 (as the pads', a placeholder).
    pub wall: f64,
    /// Between two pieces where one slides over the other.
    pub clear: f64,
    /// How far each lap still overlaps at the elbow's full bend, deg.
    pub margin: f64,
    /// Scales between the upper arm's shell and the cap.
    pub scales: usize,
    /// The upper arm's shell reaches this far up the arm from the elbow joint.
    pub up_len: f64,
    /// The cap reaches this far down the forearm from the elbow joint.
    pub cap_len: f64,
    /// How far round the arm the pieces come: past the elbow's axis toward the front.
    pub down: f64,
    pub cell: f64,
}

impl Default for PadParams {
    fn default() -> Self {
        PadParams { foam: 9.0, wall: 2.0, clear: 1.0, margin: 10.0, scales: 2, up_len: 100.0, cap_len: 80.0, down: 10.0, cell: 0.75 }
    }
}

/// The sweep, the elbow's bend (deg): straight to full flexion; the pieces are built at 90.
const STEPS: [f64; 8] = [0.0, 20.0, 40.0, 60.0, 90.0, 110.0, 130.0, ELBOW_FLEXION];
const BUILD: usize = 4;

/// A lap over the elbow, the `s`th from the shoulder: piece `s` over piece `s + 1`. Pose 0: the joint's
/// centre and axis, the elbow's back `d` and `ap` (down the forearm), square to the axis; its
/// surfaces reach `beta` back and `alpha` forward of `d` (rad), plus `EXT`.
struct Lap {
    alpha: f64,
    beta: f64,
    rho: f64,
}

/// `cargo run --release -- elbowpad <scan>`: the elbow pad, round 1 (docs/elbow/pad/).
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
    let at = |h: &ArmAt, bone: &str| upper::posed_end(&b, raw, &h.mo, bone, false);
    let el = at(a0, "lowerarm01.R");
    let ap = (at(a0, "wrist.R") - el).normalize();
    let up = (at(a0, "upperarm01.R") - el).normalize();
    let f90 = (at(&arms[BUILD], "wrist.R") - at(&arms[BUILD], "lowerarm01.R")).normalize();
    // The elbow's back (straight: square to the arm, away from the bend) and its axis, the way
    // that turns the back toward `ap` as the elbow bends (the forearm's back faces down at 90).
    let d = -(f90 - ap * f90.dot(&ap)).normalize();
    let w = d.cross(&ap).normalize();

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
    let mut movers = vec![Mover::Bone(bi("upperarm02.R"))];
    for s in 1..=p.scales {
        names.push(format!("scale {s}"));
        movers.push(Mover::Scale { a: bi("upperarm02.R"), b: bi("lowerarm01.R"), c: el, frac: s as f64 / n as f64 });
    }
    names.push("cap".to_string());
    movers.push(Mover::Bone(bi("lowerarm01.R")));
    let n_p = names.len();
    let mo = |h: &ArmAt, i: usize| motion_of(&h.mm, &a0.mm, &movers[i]);
    let pivot = {
        let hf = &arms[STEPS.len() - 1];
        let ab_rel = hf.mm[bi("upperarm02.R")].after(&a0.mm[bi("upperarm02.R")].inverse()).inverse().after(&hf.mm[bi("lowerarm01.R")].after(&a0.mm[bi("lowerarm01.R")].inverse()));
        (Point3::from(ab_rel.apply(el.coords)) - el).norm()
    };
    if pivot > 0.1 {
        return Err(format!("the forearm turns about a point {pivot:.2} mm off the elbow joint"));
    }

    // The laps: each covering an equal share of the elbow's full bend and the margin, split evenly
    // either side; each radius clear of the liner (the skin out by the foam) seen from its two
    // pieces at every step, within its reach either side and across the strip.
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
    let rels: Vec<Vec<Motion>> = arms.iter().map(|h| (0..n_p).map(|i| mo(h, i)).collect()).collect();
    let posed: Vec<Vec<IndexedMesh>> = rels.iter().map(|rs| meshes.iter().zip(rs).map(|(mm, r)| moved(mm, r)).collect()).collect();

    // Check 1: the laps stay closed. From the joint's centre (and across the strip), rays over the
    // elbow's back, from where the first under piece's edge is straight to where the cap's is at
    // this step; each must meet a piece within 80 mm.
    let gaps: Vec<f64> = par_map(&(0..arms.len()).collect::<Vec<_>>(), threads, |&ti| {
        let theta = STEPS[ti].to_radians();
        let to: Vec<Motion> = (0..n_p).map(|i| rels[ti][i].inverse().after(&rels[ti][0])).collect();
        let step = 1.5f64.to_radians();
        let k = ((laps[0].beta + laps[n - 1].alpha + theta) / step).ceil() as usize;
        let mut worst = 0.0f64;
        let span = 0.5 * (x1 - x0) - pad_x;
        for frac in [-0.8, -0.4, 0.0, 0.4, 0.8] {
            let o = el + w * (frac * span + (0.5 * (x0 + x1) - el.coords.dot(&w)));
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

    // Check 2: binding, every piece against every other, at every step.
    let pairs: Vec<(usize, usize, usize)> = (0..arms.len()).flat_map(|ti| (0..n_p).flat_map(move |i| (0..n_p).filter(move |&j| j != i).map(move |j| (ti, i, j)))).collect();
    let closest: Vec<(f64, usize)> = par_map(&pairs, threads, |&(ti, i, j)| {
        let to_j = rels[ti][j].inverse().after(&rels[ti][i]);
        meshes[i].vertices.iter().step_by(2).map(|v| pieces[j].eval(Point3::from(to_j.apply(v.coords)))).fold((f64::MAX, 0usize), |a, x| (a.0.min(x), a.1 + usize::from(x < -0.3)))
    });

    // Check 3: the fit on the liner. From each skin point under a piece (the upper arm's under the
    // shell, the forearm's under the cap, either under a scale), out along the skin's normal from
    // the liner's outside: how far to the piece, or how far the piece is already past it.
    let sits: Vec<Vec<(Vec<f64>, f64)>> = arms
        .iter()
        .enumerate()
        .map(|(ti, h)| {
            let per: Vec<Option<(usize, f64)>> = par_map(&verts, threads, |&v| {
                let nv = nrms[ti][v];
                if nv.norm() == 0.0 {
                    return None;
                }
                let liner = h.skin[v] + nv * p.foam;
                let owners: Vec<usize> = (0..n_p)
                    .filter(|&i| if i == 0 { upper_share[v] > 0.5 } else if i == n_p - 1 { fore_share[v] > 0.5 } else { true })
                    .collect();
                let mut best: Option<(usize, f64)> = None;
                for i in owners {
                    let back = rels[ti][i].inverse();
                    let (q, nq) = (Point3::from(back.apply(liner.coords)), back.r * nv);
                    let pc = &pieces[i];
                    let val = if pc.eval(q) < 0.0 {
                        let r = (1..=((p.foam / 0.25) as usize)).map(|s| s as f64 * 0.25).find(|&r| pc.eval(q - nq * r) >= 0.0).unwrap_or(p.foam);
                        Some(-r)
                    } else {
                        (1..=40).map(|s| s as f64 * 0.5).find(|&r| pc.eval(q + nq * r) < 0.0).map(|r| r - 0.25)
                    };
                    if let Some(x) = val
                        && best.is_none_or(|(_, y)| x < y)
                    {
                        best = Some((i, x));
                    }
                }
                best
            });
            let mut out = vec![(Vec::new(), 0.0f64); n_p];
            for (i, x) in per.into_iter().flatten() {
                out[i].0.push(x.max(0.0));
                out[i].1 = out[i].1.max(-x);
            }
            out
        })
        .collect();
    println!("  checked ({:.0} s)", t0.elapsed().as_secs_f64());

    // Report.
    let mut lines = Vec::new();
    let mut say = |s: String| {
        println!("{s}");
        lines.push(s);
    };
    let height = verts
        .iter()
        .map(|&i| a0.skin[i] - el)
        .filter(|q| q.dot(&ap).abs() < 3.0 && q.dot(&w).abs() < 8.0)
        .map(|q| q.dot(&d))
        .fold(f64::MIN, f64::max);
    say(format!("the elbow pad, round 1 (track U2): the right arm with an olecranon ({ole_n} skin control vertices move with it), built at the elbow's 90 deg ({})", scan.display()));
    say(format!(
        "  liner foam {:.0} mm, wall {:.0} mm (PA 12), {:.0} mm between sliding pieces (placeholders); the upper arm's shell {:.0} mm up the arm, the cap {:.0} mm down the forearm, round the arm to {:.0} mm past the elbow's axis; {} scales between, each lap spanning {:.0} deg ({:.1} deg of bend each and a {:.0} deg margin), split either side of the elbow's back where its radius comes out least",
        p.foam,
        p.wall,
        p.clear,
        p.up_len,
        p.cap_len,
        p.down,
        p.scales,
        span.to_degrees(),
        ELBOW_FLEXION / n as f64,
        p.margin
    ));
    say(format!("  the sweep, the elbow's bend: {} deg (built at {:.0})", STEPS.iter().map(|s| format!("{s:.0}")).collect::<Vec<_>>().join(" / "), STEPS[BUILD]));
    say(format!("  {} hard pieces, {:.1} g together (PA 12):", n_p, grams.iter().sum::<f64>()));
    for i in 0..n_p {
        say(format!("    {:<16} {:>5.1} g", names[i], grams[i]));
    }
    let top = laps[0].rho + over + p.wall;
    say(format!(
        "  over the elbow, each lap (from the upper arm's shell): its reach back up the arm / forward over the forearm (deg), its under piece's inside radius (clear of the liner; split evenly it would need {}): {}; the skin {:.1} mm behind the joint's centre straight; the stack's top {:.1} mm from the axis, {:.1} mm proud of the pieces either side (the liner and a wall over the skin)",
        even.iter().map(|r| format!("{r:.1}")).collect::<Vec<_>>().join(" / "),
        laps.iter().zip(&raw_rho).map(|(l, r)| format!("{:.0} / {:.0}: {:.1} mm{}", l.beta.to_degrees(), l.alpha.to_degrees(), l.rho, if l.rho > *r + 0.05 { format!(" (raised from {r:.1} to nest)") } else { String::new() })).collect::<Vec<_>>().join("; "),
        height,
        top,
        top - (height + p.foam + p.wall)
    ));
    say("  check 1, laps closed: the widest gap seen over the elbow's back (deg), at each step:".into());
    say(format!("    {}", STEPS.iter().zip(&gaps).map(|(s, g)| format!("{s:.0}: {g:.0}")).collect::<Vec<_>>().join(", ")));
    say("  check 2, binding: at each step, the deepest any piece runs into another (mm), or how close the closest two come:".into());
    for ti in 0..arms.len() {
        let (mut wv, mut wi, mut wj, mut wn) = (f64::MAX, 0, 0, 0usize);
        for (k, &(t, i, j)) in pairs.iter().enumerate() {
            if t == ti && closest[k].0 < wv {
                (wv, wi, wj, wn) = (closest[k].0, i, j, closest[k].1);
            }
        }
        say(format!(
            "    {:>5.1} deg: {}",
            STEPS[ti],
            if wv < 0.0 { format!("{:.1} mm ({wn} surface points over 0.3 mm in), {} into {}", -wv, names[wi], names[wj]) } else { format!("none; closest {:.1} mm ({} by {})", wv, names[wi], names[wj]) }
        ));
    }
    say("  check 3, the fit on the liner: how far each piece stands off it, mean / 90th percentile, and the most it presses in (mm), at each step:".into());
    let stats = |v: &[f64]| {
        if v.is_empty() {
            return (0.0, 0.0);
        }
        let mut s = v.to_vec();
        s.sort_by(|a, b| a.partial_cmp(b).unwrap());
        (s.iter().sum::<f64>() / s.len() as f64, s[(0.9 * (s.len() - 1) as f64) as usize])
    };
    say(format!("    {:<16} {}", "", STEPS.iter().map(|s| format!("{:>15}", format!("{s:.0}"))).collect::<Vec<_>>().join("")));
    for i in 0..n_p {
        say(format!(
            "    {:<16} {}",
            names[i],
            sits.iter()
                .map(|st| {
                    let (mean, p90) = stats(&st[i].0);
                    format!("{:>15}", format!("{mean:.1}/{p90:.1} +{:.1}", st[i].1))
                })
                .collect::<Vec<_>>()
                .join("")
        ));
    }
    std::fs::write(docs.join("summary.txt"), lines.join("\n") + "\n").map_err(|e| e.to_string())?;
    for (i, mm) in posed[BUILD].iter().enumerate() {
        save_mesh(mm, out.join(format!("{i:02}_{}.stl", names[i].replace(' ', "_")))).map_err(|e| e.to_string())?;
    }

    // Pictures: straight, 90 and full, seen along the elbow's axis; the same cut in the plane the
    // arm bends in.
    let skin_c = [0.85, 0.70, 0.60];
    let cols = |i: usize| if i == 0 || i == n_p - 1 { [0.16, 0.20, 0.40] } else { [0.58, 0.70, 0.92] };
    let forward = -d;
    let mut solid: Vec<(IndexedMesh, [f64; 3])> = Vec::new();
    let mut cut: Vec<(IndexedMesh, [f64; 3])> = Vec::new();
    for (slot, ti) in [0usize, BUILD, STEPS.len() - 1].into_iter().enumerate() {
        let shift = Motion { r: nalgebra::Rotation3::identity(), t: forward * (330.0 * slot as f64) - el.coords };
        let arm = quad_mesh(&arms[ti].skin, &quads);
        solid.push((moved(&arm, &shift), skin_c));
        cut.push((moved(&section_lines(&arm, el, w, 1.0), &shift), [0.6, 0.4, 0.3]));
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
    // The pad as built (the elbow at 90): from the side, and from behind either side.
    {
        let arm = quad_mesh(&ab.skin, &quads);
        let mut layers = vec![Layer::solid(&arm, skin_c)];
        layers.extend((0..n_p).map(|i| Layer::solid(&posed[BUILD][i], cols(i))));
        let views = [View::looking(-w, up), View::looking((-d - w).normalize(), up), View::looking((-d + w).normalize(), up)];
        draw(&layers, &views, 1.5, &docs.join("pad_90.png"), Style::pretty())?;
    }
    println!("drew {}/pad.png, pad_section.png, pad_90.png; pieces in {} ({:.0} s)", docs.display(), out.display(), t0.elapsed().as_secs_f64());
    Ok(())
}
