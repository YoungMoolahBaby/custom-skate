//! A small software renderer for looking at parts: orthographic views, depth buffer, two-sided
//! Lambert shading, several views side by side in one PNG. Display only.

use std::fs::File;
use std::io::BufWriter;
use std::path::Path;

use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Rotation3, Vector3};

#[derive(Clone, Copy)]
pub struct Layer<'a> {
    pub mesh: &'a IndexedMesh,
    pub color: [f64; 3],
    /// Per-vertex colours (overrides `color`), e.g. a fit map.
    pub vertex_colors: Option<&'a [[f64; 3]]>,
}

impl<'a> Layer<'a> {
    pub fn solid(mesh: &'a IndexedMesh, color: [f64; 3]) -> Self {
        Layer { mesh, color, vertex_colors: None }
    }
}

/// Blue (low) -> white -> red (high) for `t` in [0, 1].
pub fn diverging(t: f64) -> [f64; 3] {
    let t = t.clamp(0.0, 1.0);
    if t < 0.5 {
        let s = t / 0.5;
        [0.15 + 0.85 * s, 0.35 + 0.65 * s, 0.85 + 0.15 * s]
    } else {
        let s = (t - 0.5) / 0.5;
        [1.0, 1.0 - 0.75 * s, 1.0 - 0.8 * s]
    }
}

/// A view: world-to-camera rotation (camera x right, y up, z toward the viewer) and a label.
pub struct View {
    pub rot: Rotation3<f64>,
}

impl View {
    /// Looking along `dir` (from the viewer into the scene) with `up` roughly up on screen.
    pub fn looking(dir: Vector3<f64>, up: Vector3<f64>) -> Self {
        let back = -dir.normalize(); // camera z points toward the viewer
        let right = up.cross(&back).normalize();
        let up = back.cross(&right);
        View { rot: Rotation3::from_basis_unchecked(&[right, up, back]).transpose() }
    }
}

/// The four standard views of a part in the foot frame (x heel to toe, z up): from the side
/// (-y), from above, from the front (toe end), and a three-quarter view from front-side-above.
pub fn standard_views() -> Vec<View> {
    vec![
        View::looking(Vector3::y(), Vector3::z()),
        View::looking(-Vector3::z(), Vector3::y()),
        View::looking(-Vector3::x(), Vector3::z()),
        View::looking(Vector3::new(-0.6, 0.65, -0.45), Vector3::z()),
    ]
}

pub fn render(layers: &[Layer], views: &[View], px_per_mm: f64, path: &Path) -> std::io::Result<(u32, u32)> {
    let pad = 12usize;
    let light = Vector3::new(-0.3, 0.5, 0.8).normalize();
    let mut panels: Vec<(usize, usize, Vec<[u8; 3]>)> = Vec::new();
    for view in views {
        let cam: Vec<Vec<Point3<f64>>> =
            layers.iter().map(|l| l.mesh.vertices.iter().map(|v| view.rot * v).collect()).collect();
        let (mut lo, mut hi) = (Vector3::repeat(f64::MAX), Vector3::repeat(f64::MIN));
        for c in &cam {
            for p in c {
                lo = lo.inf(&p.coords);
                hi = hi.sup(&p.coords);
            }
        }
        let w = ((hi.x - lo.x) * px_per_mm).ceil() as usize + 2 * pad;
        let h = ((hi.y - lo.y) * px_per_mm).ceil() as usize + 2 * pad;
        let mut img = vec![[245u8, 245, 242]; w * h];
        let mut depth = vec![f64::MIN; w * h];
        let to_px = |p: &Point3<f64>| {
            (
                pad as f64 + (p.x - lo.x) * px_per_mm,
                (h - pad) as f64 - (p.y - lo.y) * px_per_mm,
                p.z,
            )
        };
        for (layer, c) in layers.iter().zip(&cam) {
            for f in &layer.mesh.faces {
                let [a, b, d] = f.map(|i| c[i as usize]);
                let n = (b - a).cross(&(d - a));
                if n.norm() == 0.0 {
                    continue;
                }
                let shade = 0.28 + 0.72 * n.normalize().dot(&light).abs();
                let base = match layer.vertex_colors {
                    Some(vc) => {
                        let [i, j, k] = f.map(|i| i as usize);
                        [0, 1, 2].map(|c| (vc[i][c] + vc[j][c] + vc[k][c]) / 3.0)
                    }
                    None => layer.color,
                };
                let col = base.map(|k| (k * shade * 255.0).clamp(0.0, 255.0) as u8);
                let (p0, p1, p2) = (to_px(&a), to_px(&b), to_px(&d));
                let xmin = p0.0.min(p1.0).min(p2.0).floor().max(0.0) as usize;
                let xmax = (p0.0.max(p1.0).max(p2.0).ceil() as usize).min(w - 1);
                let ymin = p0.1.min(p1.1).min(p2.1).floor().max(0.0) as usize;
                let ymax = (p0.1.max(p1.1).max(p2.1).ceil() as usize).min(h - 1);
                let area = (p1.0 - p0.0) * (p2.1 - p0.1) - (p2.0 - p0.0) * (p1.1 - p0.1);
                if area.abs() < 1e-12 {
                    continue;
                }
                for y in ymin..=ymax {
                    for x in xmin..=xmax {
                        let (px, py) = (x as f64 + 0.5, y as f64 + 0.5);
                        let w0 = ((p1.0 - px) * (p2.1 - py) - (p2.0 - px) * (p1.1 - py)) / area;
                        let w1 = ((p2.0 - px) * (p0.1 - py) - (p0.0 - px) * (p2.1 - py)) / area;
                        let w2 = 1.0 - w0 - w1;
                        if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                            continue;
                        }
                        let z = w0 * p0.2 + w1 * p1.2 + w2 * p2.2;
                        let i = y * w + x;
                        if z > depth[i] {
                            depth[i] = z;
                            img[i] = col;
                        }
                    }
                }
            }
        }
        panels.push((w, h, img));
    }
    let width: usize = panels.iter().map(|p| p.0).sum();
    let height = panels.iter().map(|p| p.1).max().unwrap_or(1);
    let mut out = vec![245u8; width * height * 3];
    let mut x0 = 0;
    for (w, h, img) in &panels {
        for y in 0..*h {
            for x in 0..*w {
                let o = (y * width + x0 + x) * 3;
                out[o..o + 3].copy_from_slice(&img[y * w + x]);
            }
        }
        x0 += w;
    }
    let file = File::create(path)?;
    let mut enc = png::Encoder::new(BufWriter::new(file), width as u32, height as u32);
    enc.set_color(png::ColorType::Rgb);
    enc.set_depth(png::BitDepth::Eight);
    let mut writer = enc.write_header().map_err(std::io::Error::other)?;
    writer.write_image_data(&out).map_err(std::io::Error::other)?;
    Ok((width as u32, height as u32))
}
