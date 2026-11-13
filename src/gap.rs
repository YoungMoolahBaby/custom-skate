//! G3: the gap between the girdle's thigh pad and the shin pad's top, over the hip's and knee's
//! range of motion (Jon: the girdle should "not overlap with the shin pads when moving", "but
//! also yet the gap between the bottom of the girdle and top of the knee pad should be as small
//! as it can be without them interfering with each other").
//!
//! Round 2 (2026-11-13): the shin pad's knee pieces are the cup (`cup.rs`), built on the knee
//! bent for skating and held on by the sock or pants: at each pose the knee shield (with its
//! liner) and the thigh guard (with its foam) each sit where they best fit the skin under them.
//! The pad's lower parts (front shell, side panels, liners, calf wraps, straps) are strapped to
//! the shin and move with it. Round 1 (the straight shield, carried by the skin or with the shin)
//! is in docs/PLAN.md.
//!
//! The girdle's thigh pad is carried by the skin (`body::Body::moves`): each point takes the skin
//! weights of the body under it (`Body::weights_near`), as MakeHuman carries clothes.
//!
//! The shin pad's lower parts are the latest fitted ones (`pad` run, tongue under, flush:
//! `out/pad/`), built on the stand-in leg (`leg.rs`) leaned like the boot; here they are stood up
//! again (the lean taken out) and set on the stand-in body (`body.rs`), whose knee is checked
//! against the leg's.

use std::path::Path;
use std::time::Instant;

use cortenforge::cf_scan_prep_core::clip_mesh_against_plane;
use cortenforge::mesh::io::load_mesh;
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::body;
use crate::cup;
use crate::girdle::{self, GPart, GirdleParams};
use crate::pad::{self, NearMesh, Outlines, Part, close_between, draw};
use crate::pose::{LegPose, Motion};
use crate::render::{Layer, Style, View};

/// A point carried by its weights: the weighted sum of its bones' motions.
fn carry(p: Point3<f64>, w: &[(usize, f64)], moves: &[Motion]) -> Point3<f64> {
    Point3::from(w.iter().fold(Vector3::zeros(), |a, &(b, x)| a + moves[b].apply(p.coords) * x))
}

/// How a shin pad piece moves.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Carry {
    /// Strapped to the shin; the mesh standing.
    Shin,
    /// A cup piece, fitted to the skin under it; the mesh on the body in the cup's build pose.
    Cup(cup::Piece),
}

struct Piece {
    part: Part,
    mesh: IndexedMesh,
    carry: Carry,
}

#[derive(Clone, Copy, Debug)]
struct Near {
    gap: f64,
    part: Part,
    /// The shin pad's point, where it was built (standing, or the cup's build pose).
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

/// One pose's motions: the bones' (for the girdle), the shin's, and each cup piece's fit.
struct Placed {
    moves: Vec<Motion>,
    shin: Motion,
    cup: [Motion; 2],
}

fn place(pc: &Piece, pl: &Placed, v: Point3<f64>) -> Point3<f64> {
    let m = match pc.carry {
        Carry::Shin => &pl.shin,
        Carry::Cup(k) => &pl.cup[cup::Piece::ALL.iter().position(|x| *x == k).unwrap()],
    };
    Point3::from(m.apply(v.coords))
}

/// The pose's closest approach of the shin pad to the girdle's thigh pad (shell and foam):
/// signed, negative where they overlap. Each shin pad vertex within `reach` of the thigh pad's
/// box is measured; and the per-vertex gaps, for drawing.
fn closest(girdle: &[IndexedMesh], pieces: &[Piece], gw: &[Vec<(usize, f64)>], pl: &Placed, slide: Vector3<f64>) -> Result<(Near, Vec<IndexedMesh>, Vec<Vec<f64>>, Vec<IndexedMesh>), String> {
    let reach = 40.0;
    let mut off = 0;
    let mut posed_g = Vec::new();
    for m in girdle {
        let pm = posed_mesh(m, |i, v| carry(v, &gw[off + i], &pl.moves) + slide);
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
        let pm = posed_mesh(&pc.mesh, |_, v| place(pc, pl, v));
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
/// thigh pad bottom from its 600. Run `pad` and `cup` first.
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

    let ll = pad::leaned_leg(scan_path)?;
    let kz = ll.knee.z;
    let b = body::build(scan_path, 1250.0)?;
    let bk = b.joints["r knee"];
    let shin = b.bone("lowerleg01.R").ok_or("no lowerleg01.R")?;
    say(format!(
        "G3, round 2: the gap between the girdle's thigh pad and the shin pad over the hip's and knee's range, on the stand-in body ({}); the shin pad's knee pieces the cup built at knee {} deg",
        scan_path.display(),
        cup::KNEE_BUILD
    ));
    say(format!(
        "knee joint: the leg the shin pad was built on ({:.0}, {:.0}, {:.0}), the body ({:.0}, {:.0}, {:.0}) mm",
        ll.lg.knee.x, ll.lg.knee.y, ll.lg.knee.z, bk.x, bk.y, bk.z
    ));

    // The cup's build pose, and the skin under each of its pieces.
    let (build_m, _, build_v) = cup::posed(&b, cup::build_pose())?;
    let anc = cup::anchors(&b, &ll, &Outlines::new(kz));
    let to_body = build_v[shin];
    let fs = cup::front_of_shin(&b);

    // The shin pad: lower parts stood up again (the lean taken out); the cup's parts from the
    // leaned shin frame to the body in the build pose.
    let mut pieces = Vec::new();
    for part in Part::all() {
        let (name, carry) = match cup::Piece::of(part) {
            Some(k) => (cup::stl_name(part), Carry::Cup(k)),
            None => (format!("fitted_tongue_under_flush-{}.stl", part.name().replace([' ', ','], "_").replace("__", "_")), Carry::Shin),
        };
        let path = Path::new("out/pad").join(&name);
        let mut m = load_mesh(&path).map_err(|e| format!("{}: {e} (run `pad` and `cup` first)", path.display()))?;
        for v in &mut m.vertices {
            *v = ll.unlean(*v);
            if carry != Carry::Shin {
                *v = Point3::from(to_body.apply(v.coords));
            }
        }
        if carry == Carry::Shin && m.vertices.iter().all(|v| v.z < kz - 150.0) {
            continue;
        }
        pieces.push(Piece { part, mesh: m, carry });
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
    say(format!("standing: the girdle's thigh pad bottom {:.0} mm (its outline at {:.0})", shell_bottom, gp.thigh_bottom));

    // Does the shin pad's lower part sit on the body as it did on the leg? The body's skin
    // inside any of them.
    let lower: Vec<&Piece> = pieces.iter().filter(|p| p.carry == Carry::Shin).collect();
    let fits: Vec<NearMesh> = lower.iter().map(|p| NearMesh::new(&p.mesh, "shin pad")).collect::<Result<_, _>>()?;
    let mid = b.shift.1;
    let mut deepest = (0.0f64, Point3::origin(), Part::ShinShell);
    for v in b.mesh.vertices.iter().filter(|v| v.y < mid && v.z > 150.0 && v.z < kz + 130.0) {
        for (f, p) in fits.iter().zip(&lower) {
            let d = f.eval(*v);
            if d < deepest.0 {
                deepest = (d, *v, p.part);
            }
        }
    }
    drop(fits);
    let fr = &g.thighs[0];
    say(format!(
        "the body's skin into the shin pad's lower parts (built on the leg): {:.1} mm at most, into the {} at z {:.0}, {:.0} deg round from the front (+ inside)",
        -deepest.0,
        deepest.2.name(),
        deepest.1.z,
        fr.theta(deepest.1)
    ));

    // Skin weights for the girdle, standing.
    let t1 = Instant::now();
    let gpts: Vec<Point3<f64>> = girdle_parts.iter().flat_map(|m| m.vertices.iter().copied()).collect();
    let gw = b.weights_near(&gpts, 6);
    say(format!(
        "carried: {} thigh pad points by the skin (6 nearest control vertices each, {:.0} s); the shin pad's lower parts with the shin; the cup's knee shield and thigh guard each fitted to the skin under them ({} and {} skin points)",
        gpts.len(),
        t1.elapsed().as_secs_f64(),
        anc[0].len(),
        anc[1].len()
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

    // One pose's placement: the bones' motions, the shin's, the cup pieces' fits.
    let placed = |p: LegPose| -> Result<Placed, String> {
        let (qm, _, mv) = cup::posed(&b, p)?;
        let n = cup::normals(&qm, fs);
        let fits = [0, 1].map(|k| cup::fit(&anc[k], &build_m, &qm, &n));
        Ok(Placed { shin: mv[shin], moves: mv, cup: fits })
    };

    // The sweep: both legs alike.
    let hips = [-20.0, 0.0, 30.0, 60.0, 90.0, 110.0];
    let knees = [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0, 120.0, 135.0];
    let poses: Vec<(f64, f64)> = hips.iter().flat_map(|&h| knees.iter().map(move |&k| (h, k))).collect();
    // The push at a hip angle: the next measured angle up (none for extension).
    let push_at = |h: f64| push.iter().find(|p| p.0 >= h).map(|p| p.1).unwrap_or(0.0);
    let mut results: Vec<(f64, f64, Near, Near)> = Vec::new();
    for chunk in poses.chunks(threads) {
        let done: Vec<Result<(f64, f64, Near, Near), String>> = std::thread::scope(|sc| {
            let hs: Vec<_> = chunk
                .iter()
                .map(|&(h, k)| {
                    let (gparts, pieces, gw, push_at, placed) = (&girdle_parts, &pieces, &gw, &push_at, &placed);
                    sc.spawn(move || {
                        let pl = placed(LegPose { hip: h, knee: k, ankle: 0.0 })?;
                        let (a, ..) = closest(gparts, pieces, gw, &pl, Vector3::zeros())?;
                        let down = pl.moves[femur].r * (knee_j - hip_j).normalize();
                        let (c, ..) = closest(gparts, pieces, gw, &pl, down * push_at(h))?;
                        Ok((h, k, a, c))
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
        "closest approach, mm (negative: overlapping; >40: nothing within 40), by hip flexion (rows) and knee flexion (columns); the thigh pad where the skin carries it / pushed down by the belly as above ({:.0} s):",
        t0.elapsed().as_secs_f64()
    ));
    say(format!("  hip\\knee {}", knees.iter().map(|k| format!("{k:>9.0}")).collect::<String>()));
    for &h in &hips {
        let row: String = knees
            .iter()
            .map(|&k| {
                let r = results.iter().find(|r| r.0 == h && r.1 == k).unwrap();
                format!("{:>9}", format!("{}/{}", mm(&r.2), mm(&r.3)))
            })
            .collect();
        say(format!("  {h:>8.0} {row}"));
    }
    for (label, pick) in [("where the skin carries the thigh pad", 0), ("the thigh pad pushed down", 1)] {
        let get = |r: &(f64, f64, Near, Near)| [r.2, r.3][pick];
        let r = results.iter().min_by(|a, b| get(a).gap.total_cmp(&get(b).gap)).unwrap();
        let n = get(r);
        say(format!("  closest, {label}: {:.1} mm at hip {:.0}, knee {:.0}: the shin pad's {} (as built at z {:.0})", n.gap, r.0, r.1, n.part.name(), n.at.z));
    }
    for r in &results {
        if r.2.gap < 30.0 {
            say(format!("  hip {:.0} knee {:.0}: {:.0} mm, the shin pad's {} (as built at z {:.0})", r.0, r.1, r.2.gap, r.2.part.name(), r.2.at.z));
        }
    }
    let tag = std::env::var("GAP_THIGH_BOTTOM").map(|z| format!("gap2_{z}")).unwrap_or_else(|_| "gap2".into());
    std::fs::write(docs.join(format!("{tag}.txt")), lines.join("\n") + "\n").map_err(|e| e.to_string())?;

    // Drawn: four poses side by side from the right, the knee's region; the thigh pad pushed down
    // as above, the shin pad reddening where it comes within 40 mm of the thigh pad.
    let shows = [("standing", 0.0, 0.0), ("skating stance", 60.0, 70.0), ("kneeling", 10.0, 100.0), ("deep crouch", 90.0, 130.0)];
    let mut meshes: Vec<(IndexedMesh, [f64; 3], Option<Vec<[f64; 3]>>)> = Vec::new();
    for (k, (_, h, kn)) in shows.iter().enumerate() {
        let p = LegPose { hip: *h, knee: *kn, ankle: 0.0 };
        let pl = placed(p)?;
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
        let down = pl.moves[femur].r * (knee_j - hip_j).normalize();
        let (_, posed_s, gaps, posed_g) = closest(&girdle_parts, &pieces, &gw, &pl, down * push_at(*h))?;
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
