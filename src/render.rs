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
    /// Specular highlight, 0 (matte) to 1 (gloss); smooth style only. Black-on-black parts read
    /// apart by their sheen.
    pub spec: f64,
}

impl<'a> Layer<'a> {
    pub fn solid(mesh: &'a IndexedMesh, color: [f64; 3]) -> Self {
        Layer { mesh, color, vertex_colors: None, spec: 0.0 }
    }

    pub fn with_spec(mut self, spec: f64) -> Self {
        self.spec = spec;
        self
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

/// How a render looks. `Style::default()` is the original flat look.
#[derive(Clone, Copy)]
pub struct Style {
    /// Interpolate vertex normals across each triangle (Phong-style) instead of flat facets.
    pub smooth: bool,
    /// Dark lines where the part changes or the depth jumps (silhouettes, part edges).
    pub outline: bool,
    /// Supersampling factor (1 = none): rendered k times larger, then averaged down.
    pub ssaa: usize,
}

impl Default for Style {
    fn default() -> Self {
        Style { smooth: false, outline: false, ssaa: 1 }
    }
}

impl Style {
    /// Smooth shading, outlines, 2x2 supersampling: for showing a design.
    pub fn pretty() -> Self {
        Style { smooth: true, outline: true, ssaa: 2 }
    }
}

pub fn render(layers: &[Layer], views: &[View], px_per_mm: f64, path: &Path) -> std::io::Result<(u32, u32)> {
    render_styled(layers, views, px_per_mm, path, Style::default())
}

/// Area-weighted vertex normals (zero where a vertex has no area around it).
fn vertex_normals(mesh: &IndexedMesh) -> Vec<Vector3<f64>> {
    let mut n = vec![Vector3::zeros(); mesh.vertices.len()];
    for f in &mesh.faces {
        let [a, b, c] = f.map(|i| mesh.vertices[i as usize]);
        let fnorm = (b - a).cross(&(c - a));
        for i in f {
            n[*i as usize] += fnorm;
        }
    }
    for v in &mut n {
        let l = v.norm();
        if l > 0.0 {
            *v /= l;
        }
    }
    n
}

pub fn render_styled(
    layers: &[Layer],
    views: &[View],
    px_per_mm: f64,
    path: &Path,
    style: Style,
) -> std::io::Result<(u32, u32)> {
    let k = style.ssaa.max(1);
    let px_per_mm = px_per_mm * k as f64;
    let pad = 12 * k;
    let light = Vector3::new(-0.3, 0.5, 0.8).normalize();
    let fill = Vector3::new(0.6, -0.2, 0.4).normalize();
    // Blinn half vector between the key light and the viewer (camera z points at the viewer).
    let half = (light + Vector3::z()).normalize();
    let normals: Vec<Vec<Vector3<f64>>> =
        if style.smooth { layers.iter().map(|l| vertex_normals(l.mesh)).collect() } else { Vec::new() };
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
        let mut id = vec![usize::MAX; w * h];
        let to_px = |p: &Point3<f64>| {
            (
                pad as f64 + (p.x - lo.x) * px_per_mm,
                (h - pad) as f64 - (p.y - lo.y) * px_per_mm,
                p.z,
            )
        };
        for (li, (layer, c)) in layers.iter().zip(&cam).enumerate() {
            let vn: Option<Vec<Vector3<f64>>> =
                normals.get(li).map(|ns| ns.iter().map(|n| view.rot * n).collect());
            for f in &layer.mesh.faces {
                let [a, b, d] = f.map(|i| c[i as usize]);
                let n = (b - a).cross(&(d - a));
                if n.norm() == 0.0 {
                    continue;
                }
                let shade_of = |n: &Vector3<f64>| -> (f64, f64) {
                    let n = n.normalize();
                    if style.smooth {
                        let diffuse = 0.22 + 0.62 * n.dot(&light).abs() + 0.16 * n.dot(&fill).abs();
                        (diffuse, layer.spec * n.dot(&half).abs().powi(40))
                    } else {
                        (0.28 + 0.72 * n.dot(&light).abs(), 0.0)
                    }
                };
                let flat_shade = shade_of(&n);
                let corner_n = vn.as_ref().map(|vn| f.map(|i| vn[i as usize]));
                let base = match layer.vertex_colors {
                    Some(vc) => {
                        let [i, j, k] = f.map(|i| i as usize);
                        [0, 1, 2].map(|c| (vc[i][c] + vc[j][c] + vc[k][c]) / 3.0)
                    }
                    None => layer.color,
                };
                let col_of = |(shade, hi): (f64, f64)| base.map(|k| ((k * shade + 0.7 * hi) * 255.0).clamp(0.0, 255.0) as u8);
                let flat_col = col_of(flat_shade);
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
                            id[i] = li;
                            img[i] = match &corner_n {
                                Some([na, nb, nc]) => {
                                    let ni = na * w0 + nb * w1 + nc * w2;
                                    if ni.norm() > 1e-9 { col_of(shade_of(&ni)) } else { flat_col }
                                }
                                None => flat_col,
                            };
                        }
                    }
                }
            }
        }
        if style.outline {
            // A pixel on a part edge or a depth step (> 2.5 mm) is drawn dark.
            let step = 2.5;
            let mut edge = vec![false; w * h];
            for y in 0..h {
                for x in 0..w {
                    let i = y * w + x;
                    let mut e = false;
                    for (dx, dy) in [(1usize, 0usize), (0, 1)] {
                        let (x2, y2) = (x + dx, y + dy);
                        if x2 >= w || y2 >= h {
                            continue;
                        }
                        let j = y2 * w + x2;
                        if id[i] != id[j] || (id[i] != usize::MAX && (depth[i] - depth[j]).abs() > step) {
                            e = true;
                        }
                    }
                    if e {
                        edge[i] = true;
                    }
                }
            }
            for (i, e) in edge.iter().enumerate() {
                if *e {
                    img[i] = [40, 42, 48];
                }
            }
        }
        if k > 1 {
            let (w2, h2) = (w / k, h / k);
            let mut small = vec![[0u8; 3]; w2 * h2];
            for y in 0..h2 {
                for x in 0..w2 {
                    let mut acc = [0u32; 3];
                    for dy in 0..k {
                        for dx in 0..k {
                            let p = img[(y * k + dy) * w + x * k + dx];
                            for c in 0..3 {
                                acc[c] += p[c] as u32;
                            }
                        }
                    }
                    small[y * w2 + x] = acc.map(|a| (a / (k * k) as u32) as u8);
                }
            }
            panels.push((w2, h2, small));
        } else {
            panels.push((w, h, img));
        }
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
