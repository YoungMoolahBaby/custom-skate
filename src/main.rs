//! `cargo run --release -- <mode>`
//!
//! - `check` (default): the tools this project builds on, against known results.
//! - `pitch`: how a rocker turns weight shift into foot pitch, ice vs roller.
//! - `scan <file>`: load a foot scan (OBJ, STL, PLY) and measure it.
//! - `round1 <scan> [out]`: the whole skate on one foot, round-1 quality.
//! - `chassis [foot length mm] [out]`: the round-2 roller chassis on its own, weighed and checked.

use custom_skate::check;
use custom_skate::pitch::{FT, Rocker, fixed_alpha, passive_rocking_stiffness, roller_alpha};
use custom_skate::scan;

fn main() {
    let mode = std::env::args().nth(1).unwrap_or_else(|| "check".into());
    match mode.as_str() {
        "check" => {
            let t = check::run();
            println!("check: {}/{} passed", t.pass, t.pass + t.fail);
            if t.fail > 0 {
                std::process::exit(1);
            }
        }
        "pitch" => pitch(),
        "scan" => {
            let Some(path) = std::env::args().nth(2) else {
                eprintln!("usage: scan <file.ply|stl|obj>");
                std::process::exit(2);
            };
            if let Err(e) = scan::run(std::path::Path::new(&path)) {
                eprintln!("{e}");
                std::process::exit(1);
            }
        }
        "round1" => {
            let Some(path) = std::env::args().nth(2) else {
                eprintln!("usage: round1 <scan> [out dir]");
                std::process::exit(2);
            };
            let out = std::env::args().nth(3).unwrap_or_else(|| "out/round1".into());
            if let Err(e) = custom_skate::round1::run(std::path::Path::new(&path), std::path::Path::new(&out)) {
                eprintln!("{e}");
                std::process::exit(1);
            }
        }
        "chassis" => {
            let length = match std::env::args().nth(2) {
                Some(s) => s.parse::<f64>().unwrap_or_else(|_| {
                    eprintln!("usage: chassis [foot length mm] [out dir]");
                    std::process::exit(2);
                }),
                None => custom_skate::round2::STAND_IN_LENGTH,
            };
            let out = std::env::args().nth(3).unwrap_or_else(|| "out/chassis".into());
            if let Err(e) = custom_skate::round2::run(length, std::path::Path::new(&out)) {
                eprintln!("{e}");
                std::process::exit(1);
            }
        }
        other => {
            eprintln!("unknown mode {other:?}; modes: check, pitch, scan, round1, chassis");
            std::process::exit(2);
        }
    }
}

/// Degrees of foot pitch per centimetre of contact shift, ice and roller, from the paper's
/// model. The skater numbers (75 kg, centre of mass 0.7-1.0 m above the contact) are
/// assumptions for the passive stiffness column only.
fn pitch() {
    let per_cm = |rad_per_m: f64| (rad_per_m * 0.01).to_degrees();

    println!("Ice, fixed holder (eq. 1): the blade radius sets pitch per contact shift");
    println!("  {:<22} {:>12} {:>16}", "blade radius", "deg per cm", "deg over 30 cm");
    for ft in [9.0, 11.0, 13.0] {
        let s = ft * FT;
        println!(
            "  {:<22} {:>12.3} {:>16.2}",
            format!("{ft} ft ({s:.2} m)"),
            per_cm(1.0 / s),
            fixed_alpha(0.30, s).to_degrees()
        );
    }

    println!();
    println!("Ice, dynamic holder (eq. 9-11): blade 2.7 m, height 3 cm, rocker radius varied");
    println!("  {:<14} {:>12} {:>20} {:>14}", "rocker radius", "deg per cm", "acts like a blade of", "amplification");
    for r in [0.5, 1.0, 2.0, 4.0] {
        let p = Rocker { r, s: 2.7, h: 0.03, theta: 0.0 };
        let slope = (p.phi_of_y(1e-6) - p.phi_of_y(-1e-6)) / 2e-6;
        println!(
            "  {:<14} {:>12.3} {:>20} {:>14.2}",
            format!("{r} m"),
            per_cm(slope),
            format!("{:.2} m ({:.1} ft)", p.effective_radius(), p.effective_radius() / FT),
            p.amplification()
        );
    }

    println!();
    println!("Roller, rocker on a flat lower chassis, all wheels down (eq. 7, s -> inf)");
    println!("Marsblade R1 inserts span 5-15 ft (Hockey World Blog, 2020).");
    println!(
        "  {:<18} {:>11} {:>15} {:>34}",
        "rocker radius", "deg per cm", "deg at +-5 cm", "passive N m/rad, CoM 0.7 / 0.85 / 1.0 m"
    );
    for ft in [5.0, 7.5, 10.0, 15.0] {
        let r = ft * FT;
        let k: Vec<String> =
            [0.7, 0.85, 1.0].iter().map(|&h| format!("{:.0}", passive_rocking_stiffness(75.0, r, h))).collect();
        println!(
            "  {:<18} {:>11.3} {:>15.2} {:>34}",
            format!("{ft} ft ({r:.2} m)"),
            per_cm(1.0 / r),
            roller_alpha(0.05, r).to_degrees(),
            k.join(" / ")
        );
    }
    println!("  one-piece chassis: 0 deg per cm; it pitches only by tipping onto its end wheels");
}
