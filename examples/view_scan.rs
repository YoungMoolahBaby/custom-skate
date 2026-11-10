//! Look at a scan: raw axis-aligned extents, then three orthographic views (looking down -z,
//! along -y, along -x of the file's own frame) splatted to a PNG, nearer points brighter.
//!
//! `cargo run --release --example view_scan -- <scan> <out.png>`

use std::fs::File;
use std::io::BufWriter;

use custom_skate::scan;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let (input, out) = (&args[1], &args[2]);
    let (mesh, scale, merged) = scan::load(std::path::Path::new(input)).expect("load");
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for v in &mesh.vertices {
        for k in 0..3 {
            lo[k] = lo[k].min(v[k]);
            hi[k] = hi[k].max(v[k]);
        }
    }
    println!("{input}: units x{scale}, merged {merged}");
    for (k, name) in ["x", "y", "z"].iter().enumerate() {
        println!("  {name}: {:8.1} .. {:8.1} mm  (extent {:6.1})", lo[k], hi[k], hi[k] - lo[k]);
    }

    // Views: (horizontal axis, vertical axis, depth axis toward the viewer).
    let views = [(0usize, 1usize, 2usize), (0, 2, 1), (1, 2, 0)];
    let px_per_mm = 1.5;
    let pad = 10usize;
    let size = |k: usize| ((hi[k] - lo[k]) * px_per_mm).ceil() as usize + 2 * pad;
    let widths: Vec<usize> = views.iter().map(|v| size(v.0)).collect();
    let height = views.iter().map(|v| size(v.1)).max().unwrap();
    let width: usize = widths.iter().sum();
    let mut img = vec![20u8; width * height];
    let mut x0 = 0;
    for (vi, &(h, v, d)) in views.iter().enumerate() {
        let mut depth = vec![f64::MIN; widths[vi] * height];
        for p in &mesh.vertices {
            let px = pad + ((p[h] - lo[h]) * px_per_mm) as usize;
            let py = height - 1 - (pad + ((p[v] - lo[v]) * px_per_mm) as usize);
            let i = py * widths[vi] + px;
            if p[d] > depth[i] {
                depth[i] = p[d];
                let shade = 60.0 + 195.0 * (p[d] - lo[d]) / (hi[d] - lo[d]).max(1e-9);
                img[py * width + x0 + px] = shade as u8;
            }
        }
        x0 += widths[vi];
    }
    let file = File::create(out).expect("create png");
    let mut enc = png::Encoder::new(BufWriter::new(file), width as u32, height as u32);
    enc.set_color(png::ColorType::Grayscale);
    enc.set_depth(png::BitDepth::Eight);
    enc.write_header().expect("header").write_image_data(&img).expect("write");
    println!("  views (x-y from +z, x-z from +y, y-z from +x) -> {out}, {width} x {height} px");
}
