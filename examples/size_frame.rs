//! One like-for-like sizing run (src/sizing.rs): weigh a frame and FE it under the stride load
//! with the sideways push, then print one summary line and append it to out/size/runs.csv.
//!
//! `cargo run --release --example size_frame -- onepiece rail_t=3.5 [cell=2] [lam=0.45]`
//! `cargo run --release --example size_frame -- rocker rail_t=3 wall_t=2.5 [cell=2] [lam=0.45]`
//! `cargo run --release --example size_frame -- rocker vee=1 web_truss=6 deck_panels=6 [clamp_hw=18]`
//!
//! Any `key=value` from `sizing::set_param` sets that frame parameter.
use std::io::Write as _;

use custom_skate::frame::FrameParams;
use custom_skate::round2::STAND_IN_LENGTH;
use custom_skate::sizing::{self, Fe};
use custom_skate::strength::LOADS;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let part = args.first().cloned().unwrap_or_else(|| "onepiece".into());
    let mut p = FrameParams::default();
    let (mut cell, mut lam, mut sole, mut clamp_hw, mut cell_up) = (2.0, LOADS.lateral, false, 0.0, 0.0);
    let mut set = Vec::new();
    for a in &args[1..] {
        let (k, v) = a.split_once('=').unwrap_or_else(|| panic!("expected key=value, got {a}"));
        let v: f64 = v.parse().unwrap_or_else(|_| panic!("not a number: {a}"));
        match k {
            "cell" => cell = v,
            // cell_up=1.25: a finer mesh for the upper only (leaning walls cross the lattice).
            "cell_up" => {
                cell_up = v;
                set.push(format!("cell_up={v}"));
            }
            "lam" => lam = v,
            // sole=1: the upper bonded to a custom boot's sole along its length, not bolted at the mounts.
            "sole" => {
                sole = v > 0.0;
                set.push(format!("sole={v}"));
            }
            // clamp_hw=18: hold only the web top within |y| < 18 mm at the mounts (0 = the whole web).
            "clamp_hw" => {
                clamp_hw = v;
                set.push(format!("clamp_hw={v}"));
            }
            _ => {
                sizing::set_param(&mut p, k, v).unwrap();
                set.push(format!("{k}={v}"));
            }
        }
    }
    // The wall must sit on its ledge.
    if p.ledge_w < p.side_gap + p.wall_t + 0.5 {
        p.ledge_w = p.side_gap + p.wall_t + 0.5;
        set.push(format!("ledge_w={}", p.ledge_w));
    }
    let length = STAND_IN_LENGTH;
    println!("{part} {} (cell {cell} mm, sideways ratio {lam})", set.join(" "));
    let fmt_top = |fe: &Fe, name: &str| {
        for (s, q) in fe.top.iter().take(4) {
            println!("    {name} {s:6.1} MPa at x {:+6.1}, y {:+5.1}, z {:+5.1}", q.x, q.y, q.z);
        }
    };
    if part == "section" {
        // A front view cut at x = centre + `lam` (reused as the cut's offset, mm): U upper,
        // L lower, T ties, a axle, o wheel; 1 mm per column, 1.5 mm per row.
        let f = custom_skate::frame::build(0.0, length, p);
        let l = f.layout;
        let x = l.xc + lam;
        let mut z = l.z_s + 1.0;
        while z > l.ground_z - 1.0 {
            let row: String = (-40..=40)
                .map(|y| {
                    let q = nalgebra::Point3::new(x, y as f64, z);
                    let inside = |s: &cortenforge::cf_design::Solid| s.evaluate(&q) < 0.0;
                    if inside(&f.upper) {
                        'U'
                    } else if inside(&f.lower) {
                        'L'
                    } else if f.ties.as_ref().is_some_and(inside) {
                        'T'
                    } else if inside(&f.axles) {
                        'a'
                    } else if inside(&f.wheels) {
                        'o'
                    } else {
                        '.'
                    }
                })
                .collect();
            println!("{:6.1} {row}", z - l.ground_z);
            z -= 1.5;
        }
        println!("lean {:.1} deg; web {:.1} mm wide; {:.1} g", l.v_sin.atan2(l.v_cos).to_degrees(), 2.0 * l.wall_out(l.z_s), sizing::rocker_grams(&p, length));
        return;
    }
    if part == "mesh_upper" {
        let (e, n, err) = sizing::mesh_upper(&p, length, cell).expect("mesh");
        println!("MESH upper {} cell {cell}: {e} Tet10, {n} nodes, volume {:+.1}%", set.join(" "), 100.0 * err);
        return;
    }
    if part == "weigh" {
        let parts = sizing::rocker_parts(&p, length);
        let total: f64 = parts.iter().map(|(_, g)| g).sum();
        let split: Vec<String> = parts.iter().map(|(n, g)| format!("{n} {g:.1}")).collect();
        println!("WEIGH {}: one-piece {:.1} g, rocker {total:.1} g ({})", set.join(" "), sizing::onepiece_grams(&p, length), split.join(", "));
        return;
    }
    let (grams, peak, peak_where, dy, extra, dof, vol, secs) = match part.as_str() {
        "onepiece" => {
            let g = sizing::onepiece_grams(&p, length);
            let fe = sizing::onepiece(&p, length, cell, lam).expect("one-piece FE");
            fmt_top(&fe, "frame");
            let w = fe.top.first().map_or(String::new(), |t| format!("x{:+.0} y{:+.0} z{:+.0}", t.1.x, t.1.y, t.1.z));
            (g, fe.peak(), format!("frame {w}"), fe.axle_dy, String::new(), fe.dof, format!("{:+.1}%", 100.0 * fe.vol_err), fe.seconds)
        }
        "rocker" => {
            let g = sizing::rocker_grams(&p, length);
            let r = sizing::rocker(&p, length, cell, cell_up, lam, sole, clamp_hw).expect("rocker FE");
            fmt_top(&r.lower, "lower");
            fmt_top(&r.upper, "upper");
            let (pl, pu) = (r.lower.peak(), r.upper.peak());
            let (peak, which, fe) = if pl >= pu { (pl, "lower", &r.lower) } else { (pu, "upper", &r.upper) };
            let w = fe.top.first().map_or(String::new(), |t| format!("x{:+.0} y{:+.0} z{:+.0}", t.1.x, t.1.y, t.1.z));
            let c = r.carry;
            println!(
                "    axles' give: lower {:?} + carried by the upper {:?} (shift {:.3} mm, yaw {:.2e}/mm, roll {:.2e} rad)",
                r.lower.axle_dy.map(|v| (v * 1000.0).round() / 1000.0),
                r.upper.axle_dy.map(|v| (v * 1000.0).round() / 1000.0),
                c.dy,
                c.yaw,
                c.roll
            );
            let extra = format!("lower {pl:.0} upper {pu:.0} MPa; give lower {:.2} upper {:.2} mm", mean(&r.lower.axle_dy), mean(&r.upper.axle_dy));
            let vol = format!("{:+.1}%/{:+.1}%", 100.0 * r.lower.vol_err, 100.0 * r.upper.vol_err);
            (g, peak, format!("{which} {w}"), r.axle_dy, extra, r.lower.dof + r.upper.dof, vol, r.lower.seconds + r.upper.seconds)
        }
        other => panic!("part {other}: onepiece or rocker"),
    };
    let dmax = dy.iter().fold(0.0f64, |a, b| a.max(b.abs()));
    let line = format!(
        "{part},{},{cell},{lam},{grams:.1},{peak:.0},{peak_where},{:.3},{dmax:.3},{extra},{vol},{dof},{secs:.0}",
        set.join(" "),
        mean(&dy)
    );
    println!(
        "RESULT {part} {}: {grams:.1} g, peak {peak:.0} MPa ({peak_where}), axles sideways mean {:.3} max {dmax:.3} mm {extra} (mesh {vol}, {dof} dof, {secs:.0} s)",
        set.join(" "),
        mean(&dy)
    );
    let _ = std::fs::create_dir_all("out/size");
    let path = std::path::Path::new("out/size/runs.csv");
    let new = !path.exists();
    let mut file = std::fs::OpenOptions::new().create(true).append(true).open(path).expect("runs.csv");
    if new {
        let _ = writeln!(file, "part,params,cell,lam,grams,peak_mpa,peak_where,dy_mean_mm,dy_max_mm,split,mesh_vol_err,dof,seconds");
    }
    let _ = writeln!(file, "{line}");
}

fn mean(a: &[f64; 4]) -> f64 {
    a.iter().sum::<f64>() / 4.0
}
