//! Measure every scan in a folder and compare the frame `scan::measure` finds with the
//! dataset's own: the stand-in scans (Schuster et al. 2024) come aligned with x heel to toe, y
//! across, z up and the floor at z = 0 (seen in `view_scan`). Prints one line per scan, then a
//! summary.
//!
//! `cargo run --release --example batch_scan -- <folder> [out.csv]`

use std::io::Write;

use custom_skate::scan;
use nalgebra::Vector3;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let dir = std::path::Path::new(&args[1]);
    let mut files: Vec<_> = std::fs::read_dir(dir)
        .expect("read dir")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e.eq_ignore_ascii_case("ply")))
        .collect();
    files.sort();
    let mut csv = args.get(2).map(|p| std::fs::File::create(p).expect("csv"));
    if let Some(f) = csv.as_mut() {
        writeln!(f, "file,length_mm,width_mm,height_mm,up_from,along_dot_x,up_dot_z,x_extent_mm,ball_frac,ball_width_mm,ball_girth_mm").unwrap();
    }
    let (mut ups, mut alongs, mut flips, mut errs) = (Vec::new(), Vec::new(), 0, Vec::new());
    let mut from_sole = 0;
    for path in &files {
        let (mesh, _, _) = match scan::load(path) {
            Ok(m) => m,
            Err(e) => {
                println!("{e}");
                continue;
            }
        };
        let m = scan::measure(&mesh, 20);
        let (xmin, xmax) = mesh.vertices.iter().fold((f64::MAX, f64::MIN), |a, v| (a.0.min(v.x), a.1.max(v.x)));
        let ax = m.along.dot(&Vector3::x());
        let uz = m.up.dot(&Vector3::z());
        if ax < 0.0 {
            flips += 1;
        }
        if m.up_from == "the flat sole" {
            from_sole += 1;
        }
        ups.push(uz);
        alongs.push(ax.abs());
        errs.push((m.length - (xmax - xmin)) / (xmax - xmin));
        let ball = m
            .stations
            .iter()
            .filter(|s| (0.55..=0.85).contains(&s.frac))
            .max_by(|a, b| a.width.total_cmp(&b.width))
            .cloned();
        let name = path.file_name().unwrap().to_string_lossy();
        println!(
            "{name:<16} L {:6.1} W {:5.1} H {:5.1}  up from {:<26} along.x {:+.3} up.z {:+.3}",
            m.length, m.width, m.height, m.up_from, ax, uz
        );
        if let Some(f) = csv.as_mut() {
            let (bf, bw, bg) = ball.map_or((0.0, 0.0, 0.0), |b| (b.frac, b.width, b.perimeter));
            writeln!(
                f,
                "{name},{:.2},{:.2},{:.2},{},{:.4},{:.4},{:.2},{bf:.2},{bw:.2},{bg:.2}",
                m.length,
                m.width,
                m.height,
                m.up_from,
                ax,
                uz,
                xmax - xmin
            )
            .unwrap();
        }
    }
    let q = |v: &mut Vec<f64>, f: f64| {
        v.sort_by(|a, b| a.total_cmp(b));
        v[((v.len() - 1) as f64 * f).round() as usize]
    };
    let n = ups.len();
    println!();
    println!("{n} scans; up from the flat sole in {from_sole}");
    println!(
        "up . z:      min {:.4}, median {:.4}",
        q(&mut ups, 0.0),
        q(&mut ups, 0.5)
    );
    println!(
        "|along . x|: min {:.4}, p10 {:.4}, median {:.4}  (heel at the wrong end in {flips})",
        q(&mut alongs, 0.0),
        q(&mut alongs, 0.1),
        q(&mut alongs, 0.5)
    );
    println!(
        "length vs the file's x extent: min {:+.2}%, median {:+.2}%, max {:+.2}%",
        q(&mut errs, 0.0) * 100.0,
        q(&mut errs, 0.5) * 100.0,
        q(&mut errs, 1.0) * 100.0
    );
}
