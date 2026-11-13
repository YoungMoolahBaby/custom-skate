//! G3: the gap between the girdle's thigh pad and the shin pad's top, over the hip's and knee's
//! range of motion (Jon: the girdle should "not overlap with the shin pads when moving", "but
//! also yet the gap between the bottom of the girdle and top of the knee pad should be as small
//! as it can be without them interfering with each other").
//!
//! Both pads are carried by the posed body (`body::Body::moves`): each point takes the skin
//! weights of the body under it (`Body::weights_near`), as MakeHuman carries clothes. The shin
//! pad is also carried rigidly with the shin (strapped to it, its knee shield riding the
//! kneecap, which the patellar tendon ties to the shin): the real pad is between the two.
//!
//! The shin pad is the latest fitted one (`pad` run, tongue under, flush: `out/pad/`), built on
//! the stand-in leg (`leg.rs`) leaned like the boot; here it is stood up again (the lean taken
//! out) and set on the stand-in body (`body.rs`), whose knee is checked against the leg's.

use std::path::Path;
use std::time::Instant;

use cortenforge::cf_scan_prep_core::clip_mesh_against_plane;
use cortenforge::mesh::io::load_mesh;
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::body;
use crate::girdle::{self, GPart, GirdleParams};
use crate::hockey_boot::{HockeyParams, landmarks, lean_dx_at};
use crate::leg;
use crate::pad::{NearMesh, Part, close_between, draw};
use crate::pose::{LegPose, Motion};
use crate::render::{Layer, Style, View};

/// A point carried by its weights: the weighted sum of its bones' motions.
fn carry(p: Point3<f64>, w: &[(usize, f64)], moves: &[Motion]) -> Point3<f64> {
    Point3::from(w.iter().fold(Vector3::zeros(), |a, &(b, x)| a + moves[b].apply(p.coords) * x))
}

/// The shin pad's points above `cut` are carried by the skin; below it (all shin), rigidly with
/// the shin.
struct Piece {
    part: Part,
    mesh: IndexedMesh,
    /// Per vertex: skin weights, or none (below the cut: with the shin).
    w: Vec<Option<Vec<(usize, f64)>>>,
}

#[derive(Clone, Copy, Debug)]
enum Carry {
    Skin,
    Shin,
}

#[derive(Clone, Copy, Debug)]
struct Near {
    gap: f64,
    part: Part,
    /// The shin pad's point, standing.
    at: Point3<f64>,
}

fn posed_mesh(m: &IndexedMesh, f: impl Fn(usize, Point3<f64>) -> Point3<f64>) -> IndexedMesh {
    let mut out = m.clone();
    for (i, v) in out.vertices.iter_mut().enumerate() {
        *v = f(i, *v);
    }
    out
}

fn bbox(m: &IndexedMesh) -> (Vector3<f64>, Vector3<f64>) {
    m.vertices.iter().fold((Vector3::repeat(f64::MAX), Vector3::repeat(f64::MIN)), |(lo, hi), v| (lo.inf(&v.coords), hi.sup(&v.coords)))
}

/// The pose's closest approach of the shin pad to the girdle's thigh pad (shell and foam):
/// signed, negative where they overlap. Each shin pad vertex within `reach` of the thigh pad's
/// box is measured; and the per-vertex gaps, for drawing.
fn closest(girdle: &[IndexedMesh], pieces: &[Piece], gw: &[Vec<(usize, f64)>], moves: &[Motion], shin: usize, how: Carry, slide: Vector3<f64>) -> Result<(Near, Vec<IndexedMesh>, Vec<Vec<f64>>, Vec<IndexedMesh>), String> {
    let reach = 40.0;
    let mut off = 0;
    let mut posed_g = Vec::new();
    for m in girdle {
        let pm = posed_mesh(m, |i, v| carry(v, &gw[off + i], moves) + slide);
        off += m.vertices.len();
        posed_g.push(pm);
    }
    let fields: Vec<NearMesh> = posed_g.iter().map(|m| NearMesh::new(m, "girdle thigh pad")).collect::<Result<_, _>>()?;
    let (mut lo, mut hi) = (Vector3::repeat(f64::MAX), Vector3::repeat(f64::MIN));
    for m in &posed_g {
        let (a, b) = bbox(m);
        lo = lo.inf(&a);
        hi = hi.sup(&b);
    }
    lo -= Vector3::repeat(reach);
    hi += Vector3::repeat(reach);
    let mut best = Near { gap: f64::MAX, part: Part::ThighGuard, at: Point3::origin() };
    let mut posed_s = Vec::new();
    let mut gaps = Vec::new();
    for pc in pieces {
        let pm = posed_mesh(&pc.mesh, |i, v| match (how, &pc.w[i]) {
            (Carry::Skin, Some(w)) => carry(v, w, moves),
            _ => Point3::from(moves[shin].apply(v.coords)),
        });
        let mut g = vec![f64::MAX; pm.vertices.len()];
        for (i, q) in pm.vertices.iter().enumerate() {
            if q.x < lo.x || q.y < lo.y || q.z < lo.z || q.x > hi.x || q.y > hi.y || q.z > hi.z {
                continue;
            }
            let d = fields.iter().map(|f| f.eval(*q)).fold(f64::MAX, f64::min);
            g[i] = d;
            if d < best.gap {
                best = Near { gap: d, part: pc.part, at: pc.mesh.vertices[i] };
            }
        }
        gaps.push(g);
        posed_s.push(pm);
    }
    Ok((best, posed_s, gaps, posed_g))
}

/// `cargo run --release -- gap <scan>`: the sweep. `GAP_THIGH_BOTTOM` (mm) moves the girdle's
/// thigh pad bottom from its 600.
pub fn run(scan_path: &Path) -> Result<(), String> {
    let t0 = Instant::now();
    let docs = Path::new("docs/girdle");
    std::fs::create_dir_all(docs).map_err(|e| e.to_string())?;
    let threads = std::env::var("PAD_THREADS").ok().and_then(|v| v.parse::<usize>().ok()).unwrap_or(4).max(1);
    let mut lines: Vec<String> = Vec::new();
    let mut say = |l: String| {
        println!("{l}");
        lines.push(l);
    };

    // The shin pad, stood up again: the lean as `pad::run` put it in, taken out.
    let lg = leg::build(scan_path, 600.0)?;
    let lm = landmarks(&lg.foot, lg.scan_top());
    let zp = 0.5 * (lm.medial.z + lm.lateral.z);
    let lean = HockeyParams::default().lean_deg;
    let kz = lg.knee.z;
    let cut = kz - 150.0;
    let b = body::build(scan_path, 1250.0)?;
    let bk = b.joints["r knee"];
    say(format!(
        "G3, the gap between the girdle's thigh pad and the shin pad over the hip's and knee's range, on the stand-in body ({})",
        scan_path.display()
    ));
    say(format!(
        "knee joint: the leg the shin pad was built on ({:.0}, {:.0}, {:.0}), the body ({:.0}, {:.0}, {:.0}) mm",
        lg.knee.x, lg.knee.y, lg.knee.z, bk.x, bk.y, bk.z
    ));
    let mut pieces = Vec::new();
    let mut pts: Vec<Point3<f64>> = Vec::new();
    for part in Part::all() {
        let name = format!("fitted_tongue_under_flush-{}.stl", part.name().replace([' ', ','], "_").replace("__", "_"));
        let path = Path::new("out/pad").join(&name);
        let mut m = load_mesh(&path).map_err(|e| format!("{}: {e} (run `pad` first)", path.display()))?;
        for v in &mut m.vertices {
            v.x -= lean_dx_at(zp, lean, v.z);
        }
        if m.vertices.iter().all(|v| v.z < cut) {
            continue;
        }
        pts.extend(m.vertices.iter().filter(|v| v.z >= cut));
        pieces.push(Piece { part, mesh: m, w: Vec::new() });
    }

    // The girdle's right thigh pad and hip pad (shell and foam each); for the gap, the thigh
    // pad's lower part (below 720 mm, capped: closed for the inside test).
    let mut gp = GirdleParams::default();
    if let Some(z) = std::env::var("GAP_THIGH_BOTTOM").ok().and_then(|v| v.parse::<f64>().ok()) {
        gp.thigh_bottom = z;
    }
    let g = girdle::on_body(&b, gp)?;
    let mut full: Vec<IndexedMesh> = Vec::new();
    for chunk in [GPart::ThighShell(true), GPart::ThighFoam(true), GPart::HipShell(true), GPart::HipFoam(true)].chunks(threads) {
        let done: Vec<Option<IndexedMesh>> = std::thread::scope(|sc| {
            let hs: Vec<_> = chunk
                .iter()
                .map(|&part| {
                    let (g, v) = (&g, &b.mesh.vertices);
                    sc.spawn(move || girdle::build_part(g, part, v).map(|(m, _)| m))
                })
                .collect();
            hs.into_iter().map(|h| h.join().unwrap()).collect()
        });
        for m in done {
            full.push(m.ok_or("girdle: a part has no skin under it")?);
        }
    }
    let girdle_parts: Vec<IndexedMesh> = full[..2].iter().map(|m| close_between(m, 0.0, gp.thigh_bottom + 120.0)).collect();
    let shell_bottom = girdle_parts[0].vertices.iter().map(|v| v.z).fold(f64::MAX, f64::min);
    let pad_top = pieces.iter().flat_map(|p| p.mesh.vertices.iter().map(|v| v.z)).fold(f64::MIN, f64::max);
    say(format!(
        "standing: girdle's thigh pad bottom {:.0} mm (its outline at {:.0}); the shin pad's top {:.0} (its thigh guard, outline at {:.0}); {:.0} mm apart in height",
        shell_bottom,
        gp.thigh_bottom,
        pad_top,
        kz + 115.0,
        shell_bottom - pad_top
    ));

    // Does the shin pad sit on the body as it did on the leg? The body's skin inside any part.
    let fits: Vec<NearMesh> = pieces.iter().map(|p| NearMesh::new(&p.mesh, "shin pad")).collect::<Result<_, _>>()?;
    let mid = b.shift.1;
    let mut deepest = (0.0f64, Point3::origin(), Part::ThighGuard);
    for v in b.mesh.vertices.iter().filter(|v| v.y < mid && v.z > 150.0 && v.z < kz + 130.0) {
        for (f, p) in fits.iter().zip(&pieces) {
            let d = f.eval(*v);
            if d < deepest.0 {
                deepest = (d, *v, p.part);
            }
        }
    }
    drop(fits);
    let fr = &g.thighs[0];
    say(format!(
        "the body's skin into the shin pad (built on the leg): {:.1} mm at most, into the {} at z {:.0}, {:.0} deg round from the front (+ inside)",
        -deepest.0,
        deepest.2.name(),
        deepest.1.z,
        fr.theta(deepest.1)
    ));

    // Skin weights for everything carried, standing.
    let t1 = Instant::now();
    let gpts: Vec<Point3<f64>> = girdle_parts.iter().flat_map(|m| m.vertices.iter().copied()).collect();
    let gw = b.weights_near(&gpts, 6);
    let sw = b.weights_near(&pts, 6);
    let mut it = sw.into_iter();
    for pc in &mut pieces {
        pc.w = pc.mesh.vertices.iter().map(|v| if v.z >= cut { it.next() } else { None }).collect();
    }
    let shin = b.bone("lowerleg01.R").ok_or("no lowerleg01.R")?;
    say(format!(
        "carried: {} thigh pad points, {} shin pad points above {:.0} mm by the skin (6 nearest control vertices each, {:.0} s); below, with the shin",
        gpts.len(),
        pts.len(),
        cut,
        t1.elapsed().as_secs_f64()
    ));

    // Hip flexion against the torso: the thigh pad is one hard piece, so where the thigh's turn
    // brings its top into the torso (skin the right leg's bones don't move) or the hip pad (on
    // the pelvis), the torso pushes it down the thigh, if the shorts let it slide. The pad
    // carried rigidly with the thigh, the right leg alone posed; the push is how far down the
    // thigh it has to go to clear (within 0.5 mm).
    let leg_bones = b.below("upperleg01.R");
    let femur = b.bone("upperleg01.R").ok_or("no upperleg01.R")?;
    let bw = b.weights_near(&b.mesh.vertices, 6);
    let mut fixed: Vec<Point3<f64>> = b
        .mesh
        .vertices
        .iter()
        .zip(&bw)
        .filter(|(_, w)| w.iter().filter(|e| leg_bones.contains(&e.0)).map(|e| e.1).sum::<f64>() < 0.02)
        .map(|(v, _)| *v)
        .collect();
    drop(bw);
    let n_skin = fixed.len();
    fixed.extend(full[2].vertices.iter().chain(&full[3].vertices));
    let (hip_j, knee_j) = (b.joints["r hip"], b.joints["r knee"]);
    say(format!(
        "hip flexion against the torso ({} still skin points and the hip pad's {}): the thigh pad rigid with the thigh, how deep the torso reaches into it, and how far down the thigh that pushes it:",
        n_skin,
        fixed.len() - n_skin
    ));
    let mut push: Vec<(f64, f64)> = Vec::new();
    for h in [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0, 120.0] {
        let mv = b.moves([LegPose { hip: h, ..Default::default() }, LegPose::default()]);
        let m = mv[femur];
        let pads: Vec<IndexedMesh> = full[..2].iter().map(|x| posed_mesh(x, |_, v| Point3::from(m.apply(v.coords)))).collect();
        let fields: Vec<NearMesh> = pads.iter().map(|x| NearMesh::new(x, "thigh pad")).collect::<Result<_, _>>()?;
        let down = m.r * (knee_j - hip_j).normalize();
        let (mut lo, mut hi) = (Vector3::repeat(f64::MAX), Vector3::repeat(f64::MIN));
        for x in &pads {
            let (a, c) = bbox(x);
            lo = lo.inf(&a);
            hi = hi.sup(&c);
        }
        let reach = 80.0;
        let (lo, hi) = (lo.inf(&(lo + down * reach)), hi.sup(&(hi + down * reach)));
        let near: Vec<(usize, Point3<f64>)> =
            fixed.iter().copied().enumerate().filter(|(_, q)| (0..3).all(|k| q[k] >= lo[k] - 1.0 && q[k] <= hi[k] + 1.0)).collect();
        // Deepest reach into the pad slid `s` down the thigh: the points moved up by s instead.
        let depth = |s: f64| {
            near.iter()
                .map(|&(i, q)| (-fields.iter().map(|f| f.eval(q - down * s)).fold(f64::MAX, f64::min), i))
                .fold((f64::MIN, 0), |a, c| if c.0 > a.0 { c } else { a })
        };
        let (d0, who) = depth(0.0);
        let mut s = 0.0;
        while s < reach && depth(s).0 > 0.5 {
            s += 1.0;
        }
        say(format!(
            "  hip {h:>4.0}: {} {:.1} mm in{}; pushed down {}",
            if who < n_skin { "torso" } else { "hip pad" },
            d0.max(0.0),
            if d0 > 0.5 { format!(" (at z {:.0} standing)", fixed[who].z) } else { String::new() },
            if s >= reach { format!("more than {reach:.0} mm") } else { format!("{s:.0} mm") }
        ));
        push.push((h, s));
    }

    // The sweep: both legs alike.
    let hips = [-20.0, 0.0, 30.0, 60.0, 90.0, 110.0];
    let knees = [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0, 120.0, 135.0];
    let poses: Vec<(f64, f64)> = hips.iter().flat_map(|&h| knees.iter().map(move |&k| (h, k))).collect();
    // The push at a hip angle: the next measured angle up (none for extension).
    let push_at = |h: f64| push.iter().find(|p| p.0 >= h).map(|p| p.1).unwrap_or(0.0);
    let mut results: Vec<(f64, f64, Near, Near, Near)> = Vec::new();
    for chunk in poses.chunks(threads) {
        let done: Vec<Result<(f64, f64, Near, Near, Near), String>> = std::thread::scope(|sc| {
            let hs: Vec<_> = chunk
                .iter()
                .map(|&(h, k)| {
                    let (b, gparts, pieces, gw, push_at) = (&b, &girdle_parts, &pieces, &gw, &push_at);
                    sc.spawn(move || {
                        let p = LegPose { hip: h, knee: k, ankle: 0.0 };
                        let mv = b.moves([p, p]);
                        let none = Vector3::zeros();
                        let (a, ..) = closest(gparts, pieces, gw, &mv, shin, Carry::Skin, none)?;
                        let (c, ..) = closest(gparts, pieces, gw, &mv, shin, Carry::Shin, none)?;
                        let down = mv[femur].r * (knee_j - hip_j).normalize();
                        let (d, ..) = closest(gparts, pieces, gw, &mv, shin, Carry::Shin, down * push_at(h))?;
                        Ok((h, k, a, c, d))
                    })
                })
                .collect();
            hs.into_iter().map(|h| h.join().unwrap()).collect()
        });
        for r in done {
            results.push(r?);
        }
    }
    let mm = |n: &Near| if n.gap > 1e9 { ">40".to_string() } else { format!("{:.0}", n.gap) };
    say(format!(
        "closest approach, mm (negative: overlapping; >40: nothing within 40), by hip flexion (rows) and knee flexion (columns); the shin pad carried by the skin / rigidly with the shin / rigidly, the thigh pad pushed down as above ({:.0} s):",
        t0.elapsed().as_secs_f64()
    ));
    say(format!("  hip\\knee {}", knees.iter().map(|k| format!("{k:>13.0}")).collect::<String>()));
    for &h in &hips {
        let row: String = knees
            .iter()
            .map(|&k| {
                let r = results.iter().find(|r| r.0 == h && r.1 == k).unwrap();
                format!("{:>13}", format!("{}/{}/{}", mm(&r.2), mm(&r.3), mm(&r.4)))
            })
            .collect();
        say(format!("  {h:>8.0} {row}"));
    }
    for (label, pick) in [("by the skin", 0), ("with the shin", 1), ("with the shin, the thigh pad pushed down", 2)] {
        let get = |r: &(f64, f64, Near, Near, Near)| [r.2, r.3, r.4][pick];
        let r = results.iter().min_by(|a, b| get(a).gap.total_cmp(&get(b).gap)).unwrap();
        let n = get(r);
        say(format!(
            "  closest {label}: {:.1} mm at hip {:.0}, knee {:.0}: the shin pad's {} (standing at z {:.0}, {:.0} deg round from the front, + inside)",
            n.gap,
            r.0,
            r.1,
            n.part.name(),
            n.at.z,
            fr.theta(n.at)
        ));
    }
    for r in &results {
        if r.3.gap < 40.0 && r.1 >= 30.0 {
            say(format!(
                "  with the shin, hip {:.0} knee {:.0}: {:.0} mm, the shin pad's {} (standing at z {:.0}, {:.0} deg)",
                r.0,
                r.1,
                r.3.gap,
                r.3.part.name(),
                r.3.at.z,
                fr.theta(r.3.at)
            ));
        }
    }
    let tag = std::env::var("GAP_THIGH_BOTTOM").map(|z| format!("gap_{z}")).unwrap_or_else(|_| "gap".into());
    std::fs::write(docs.join(format!("{tag}.txt")), lines.join("\n") + "\n").map_err(|e| e.to_string())?;

    // Drawn: four poses side by side from the right, the knee's region; the shin pad rigid with
    // the shin, the thigh pad pushed down as above, the shin pad reddening where it comes within
    // 40 mm of the thigh pad.
    let shows = [("standing", 0.0, 0.0), ("skating stance", 60.0, 90.0), ("kneeling", 10.0, 100.0), ("deep crouch", 90.0, 130.0)];
    let mut meshes: Vec<(IndexedMesh, [f64; 3], Option<Vec<[f64; 3]>>)> = Vec::new();
    for (k, (_, h, kn)) in shows.iter().enumerate() {
        let p = LegPose { hip: *h, knee: *kn, ankle: 0.0 };
        let mv = b.moves([p, p]);
        let (body_m, j) = b.posed([p, p], 1250.0);
        let kj = j["r knee"];
        let mut m = body_m;
        // Only round the knee: a box 260 mm round the joint, the right leg's side.
        for (pt, n) in [
            (Point3::new(kj.x - 260.0, 0.0, 0.0), Vector3::x()),
            (Point3::new(kj.x + 260.0, 0.0, 0.0), -Vector3::x()),
            (Point3::new(0.0, 0.0, kj.z - 260.0), Vector3::z()),
            (Point3::new(0.0, 0.0, kj.z + 260.0), -Vector3::z()),
            (Point3::new(0.0, mid, 0.0), -Vector3::y()),
        ] {
            m = clip_mesh_against_plane(&m, pt, n);
        }
        let down = mv[femur].r * (knee_j - hip_j).normalize();
        let (_, posed_s, gaps, posed_g) = closest(&girdle_parts, &pieces, &gw, &mv, shin, Carry::Shin, down * push_at(*h))?;
        let dx = Vector3::new(600.0 * k as f64 - kj.x, 0.0, -kj.z);
        let shift = |mut m: IndexedMesh| {
            m.vertices.iter_mut().for_each(|v| *v += dx);
            m
        };
        meshes.push((shift(m), [0.85, 0.70, 0.60], None));
        meshes.push((shift(posed_g[0].clone()), [0.20, 0.22, 0.26], None));
        for ((pc, pm), gp) in pieces.iter().zip(posed_s).zip(gaps) {
            if matches!(pc.part, Part::WingFoam(_) | Part::KneeLiner | Part::ThighFoam) {
                continue;
            }
            let base = pc.part.color();
            let red = [0.85, 0.10, 0.08];
            let cols = gp
                .iter()
                .map(|&d| {
                    let t = (1.0 - d / 40.0).clamp(0.0, 1.0);
                    [0, 1, 2].map(|c| base[c] + (red[c] - base[c]) * t)
                })
                .collect();
            meshes.push((shift(pm), base, Some(cols)));
        }
    }
    let ls: Vec<Layer> = meshes.iter().map(|(m, c, vc)| Layer { mesh: m, color: *c, vertex_colors: vc.as_deref(), spec: 0.2 }).collect();
    let png = docs.join(format!("{tag}.png"));
    draw(&ls, &[View::looking(Vector3::y(), Vector3::z())], 1.0, &png, Style::pretty())?;
    println!("drew {} ({:.0} s in all)", png.display(), t0.elapsed().as_secs_f64());
    Ok(())
}
