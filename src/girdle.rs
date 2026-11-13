//! Track G2: the girdle, round 1 (docs/PLAN.md, track D: the girdle).
//!
//! Jon (2026-11-12): "i like girdles, and i like them to no overlap with the shin pads when
//! moving, and be tight to my thihgs ... a modern girdle. pants are too baggy/get in the way".
//! So: compression shorts with no gap to the body, and pads over them, each an offset from the
//! stand-in body (`body.rs`, standing):
//! - **thigh pads** across the front of each thigh, from the groin crease down to above the knee
//!   (where pucks hit; foam-heavy, as D3 found foam thickness the lever);
//! - **hip pads** over the bony points of the hips (the trochanter, ANSUR 896 mm up) up to the top
//!   of the pelvis (the iliac crest, 1058);
//! - a **kidney pad** across the lower back;
//! - a **tailbone pad** down the middle of the back below it.
//!
//! Outlines are drawn on two maps of the body laid flat (as the shin pad's, `pad::Outline`):
//! round the torso (about a vertical axis between the hip joints, 0 at the back, + toward +y,
//! mm of arc at 150 mm per radian) for the hip, kidney and tailbone pads; round each thigh (about
//! its hip-knee axis, 0 at the front, + toward the inside, 70 mm per radian) for the thigh pads.
//! The shorts are drawn as fabric on the body and weighed from their area.
//!
//! Thicknesses are placeholders (D3's runs say foam where pucks land); the cup is a separate
//! item. The bottom of the thigh pads is G3's to set against the shin pad's top through the
//! knee's bend; here it sits 600 mm up, ~20 mm above the shin pad's thigh guard standing.

use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

use cortenforge::cf_design::{Aabb, Solid};
use cortenforge::mesh::io::save_mesh;
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::body;
use crate::hockey_boot::mesh_volume;
use crate::materials::{FABRIC, LINER_FOAM, Material, PA12_MJF};
use crate::pad::{Field, Frame, Outline, close_between, draw, mirrored, smax};
use crate::render::{Layer, Style, View};
use crate::round1::mesh_of;

const R_TORSO: f64 = 150.0;
const R_THIGH: f64 = 70.0;

#[derive(Clone, Copy, Debug)]
pub struct GirdleParams {
    /// The shorts: fabric thickness, top (at the waist), bottom (on the thighs), mm.
    pub fabric: f64,
    pub shorts_top: f64,
    pub shorts_bottom: f64,
    /// Foam and shell (PA 12) of each pad, mm.
    pub thigh_foam: f64,
    pub thigh_wall: f64,
    pub hip_foam: f64,
    pub hip_wall: f64,
    pub kidney_foam: f64,
    pub kidney_wall: f64,
    pub tail_foam: f64,
    pub tail_wall: f64,
    /// The thigh pads' bottom edge, mm up (G3's to set).
    pub thigh_bottom: f64,
}

impl Default for GirdleParams {
    fn default() -> Self {
        GirdleParams {
            fabric: 1.0,
            shorts_top: 1100.0,
            shorts_bottom: 585.0,
            thigh_foam: 12.0,
            thigh_wall: 2.0,
            hip_foam: 10.0,
            hip_wall: 2.0,
            kidney_foam: 12.0,
            kidney_wall: 1.5,
            tail_foam: 10.0,
            tail_wall: 2.0,
            thigh_bottom: 600.0,
        }
    }
}

/// The girdle's parts; `true` is the right side.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GPart {
    ThighShell(bool),
    ThighFoam(bool),
    HipShell(bool),
    HipFoam(bool),
    KidneyShell,
    KidneyFoam,
    TailShell,
    TailFoam,
}

impl GPart {
    pub fn all() -> Vec<GPart> {
        let mut v = Vec::new();
        for r in [true, false] {
            v.extend([GPart::ThighShell(r), GPart::ThighFoam(r), GPart::HipShell(r), GPart::HipFoam(r)]);
        }
        v.extend([GPart::KidneyShell, GPart::KidneyFoam, GPart::TailShell, GPart::TailFoam]);
        v
    }

    pub fn shell(&self) -> bool {
        matches!(self, GPart::ThighShell(_) | GPart::HipShell(_) | GPart::KidneyShell | GPart::TailShell)
    }

    pub fn name(&self) -> String {
        let side = |r: &bool| if *r { "right" } else { "left" };
        match self {
            GPart::ThighShell(r) => format!("thigh shell, {}", side(r)),
            GPart::ThighFoam(r) => format!("thigh foam, {}", side(r)),
            GPart::HipShell(r) => format!("hip shell, {}", side(r)),
            GPart::HipFoam(r) => format!("hip foam, {}", side(r)),
            GPart::KidneyShell => "kidney shell".into(),
            GPart::KidneyFoam => "kidney foam".into(),
            GPart::TailShell => "tailbone shell".into(),
            GPart::TailFoam => "tailbone foam".into(),
        }
    }

    pub fn material(&self) -> Material {
        if self.shell() { PA12_MJF } else { LINER_FOAM }
    }

    pub fn cell(&self) -> f64 {
        if self.shell() { 0.6 } else { 1.0 }
    }

    pub fn color(&self) -> [f64; 3] {
        if self.shell() { [0.62, 0.72, 0.84] } else { [0.11, 0.12, 0.14] }
    }
}

/// The outlines on the two maps.
pub struct GOutlines {
    /// Round a thigh: s toward the inside (+), z up.
    pub thigh: Outline,
    /// Round the torso: |s| (both sides mirrored), z up; centred on the side (90 deg from the
    /// back is s = 236 mm).
    pub hip: Outline,
    /// Round the torso, centred on the back.
    pub kidney: Outline,
    pub tail: Outline,
}

impl GOutlines {
    pub fn new(p: &GirdleParams) -> Self {
        let b = p.thigh_bottom;
        // The thigh pad: from just above the knee up to the groin crease, higher on the outside
        // (~880) than the inside (~800, clear of the crotch); 65 deg round to the outside, 50 to
        // the inside.
        let thigh = [(-80.0, b), (55.0, b), (60.0, 760.0), (45.0, 815.0), (10.0, 850.0), (-40.0, 878.0), (-82.0, 880.0)];
        let hip = [(200.0, 890.0), (282.0, 890.0), (298.0, 960.0), (285.0, 1075.0), (218.0, 1085.0), (194.0, 985.0)];
        let kidney = mirrored(&[(0.0, 1095.0), (150.0, 1090.0), (176.0, 1040.0), (166.0, 990.0), (60.0, 966.0), (0.0, 962.0)]);
        let tail = mirrored(&[(0.0, 955.0), (28.0, 950.0), (30.0, 915.0), (16.0, 880.0), (0.0, 872.0)]);
        GOutlines {
            thigh: Outline::new(&thigh, 4),
            hip: Outline::new(&hip, 4),
            kidney: Outline::new(&kidney, 4),
            tail: Outline::new(&tail, 4),
        }
    }
}

/// The girdle on the body.
#[derive(Clone)]
pub struct Girdle {
    pub body: Field,
    pub p: GirdleParams,
    pub o: Arc<GOutlines>,
    /// The torso map's axis (x, y), and each thigh's frame (right, left).
    pub torso: (f64, f64),
    pub thighs: [Frame; 2],
}

impl Girdle {
    /// (s, z) on the torso map: s mm of arc from the back, + toward +y.
    fn torso_sz(&self, q: Point3<f64>) -> (f64, f64) {
        let th = (q.y - self.torso.1).atan2(-(q.x - self.torso.0));
        (th * R_TORSO, q.z)
    }

    /// (s, z, r) on a thigh's map: s mm of arc from the front, + toward the inside; r the
    /// distance from the thigh's axis.
    fn thigh_sz(&self, right: bool, q: Point3<f64>) -> (f64, f64, f64) {
        let f = &self.thighs[if right { 0 } else { 1 }];
        let c = f.axis_at(q.z);
        let r = q - c;
        let th = r.dot(&f.side).atan2(r.dot(&f.fwd));
        // `side` points to +y: the inside for the right leg, the outside for the left.
        let inward = if right { 1.0 } else { -1.0 };
        (inward * th * R_THIGH, q.z, (r.x * r.x + r.y * r.y).sqrt())
    }

    /// The part's region (negative inside).
    pub fn region(&self, part: GPart, q: Point3<f64>) -> f64 {
        let o = &self.o;
        match part {
            GPart::ThighShell(r) | GPart::ThighFoam(r) => {
                let (s, z, rad) = self.thigh_sz(r, q);
                // Only near this thigh. The femur (the axis) sits well back in the thigh: the
                // front of the upper thigh is ~115 mm from it, so the first try's 120 mm cut the
                // pad away there (2026-11-12). The outline keeps it off the other leg and the
                // buttock; this only bounds it.
                smax(o.thigh.eval(s, z), rad - 180.0, 2.0)
            }
            GPart::HipShell(r) | GPart::HipFoam(r) => {
                let (s, z) = self.torso_sz(q);
                // The right hip is -y: s < 0.
                let on_side = if r { s } else { -s };
                o.hip.eval(s.abs(), z).max(on_side)
            }
            GPart::KidneyShell | GPart::KidneyFoam => {
                let (s, z) = self.torso_sz(q);
                o.kidney.eval(s, z)
            }
            GPart::TailShell | GPart::TailFoam => {
                let (s, z) = self.torso_sz(q);
                o.tail.eval(s, z)
            }
        }
    }

    fn layers(&self, part: GPart) -> (f64, f64) {
        let p = &self.p;
        let f = p.fabric;
        match part {
            GPart::ThighFoam(_) => (f, f + p.thigh_foam),
            GPart::ThighShell(_) => (f + p.thigh_foam, f + p.thigh_foam + p.thigh_wall),
            GPart::HipFoam(_) => (f, f + p.hip_foam),
            GPart::HipShell(_) => (f + p.hip_foam, f + p.hip_foam + p.hip_wall),
            GPart::KidneyFoam => (f, f + p.kidney_foam),
            GPart::KidneyShell => (f + p.kidney_foam, f + p.kidney_foam + p.kidney_wall),
            GPart::TailFoam => (f, f + p.tail_foam),
            GPart::TailShell => (f + p.tail_foam, f + p.tail_foam + p.tail_wall),
        }
    }

    pub fn field(&self, part: GPart, q: Point3<f64>) -> f64 {
        let r = self.region(part, q);
        if r > 3.0 {
            return r;
        }
        let (a, b) = self.layers(part);
        let d = self.body.eval(q);
        (a - d).max(d - b).max(r)
    }
}

/// The box a part can be in: the body's vertices inside its region (with a margin), grown by
/// its thickness.
fn part_bounds(g: &Girdle, part: GPart, verts: &[Point3<f64>]) -> Option<Aabb> {
    let (_, outer) = g.layers(part);
    let mut lo = Vector3::repeat(f64::MAX);
    let mut hi = Vector3::repeat(f64::MIN);
    let mut any = false;
    for v in verts.iter().filter(|v| g.region(part, **v) < 15.0) {
        lo = lo.inf(&v.coords);
        hi = hi.sup(&v.coords);
        any = true;
    }
    let m = outer + 8.0;
    any.then(|| Aabb::new(Point3::from(lo - Vector3::repeat(m)), Point3::from(hi + Vector3::repeat(m))))
}

pub fn run(scan_path: &Path, out: &Path) -> Result<(), String> {
    let t0 = Instant::now();
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let docs = Path::new("docs/girdle");
    std::fs::create_dir_all(docs).map_err(|e| e.to_string())?;
    let b = body::build(scan_path, 1250.0)?;
    let p = GirdleParams::default();
    // The body's field over the girdle's heights: exact near the skin, a 3 mm grid beyond.
    let closed = close_between(&b.mesh, 500.0, 1180.0);
    let field = Field::new(&closed, 25.0, 3.0, Some(8.0), "body")?;
    let (rh, lh, rk, lk) = (b.joints["r hip"], b.joints["l hip"], b.joints["r knee"], b.joints["l knee"]);
    let torso = (0.5 * (rh.x + lh.x), 0.5 * (rh.y + lh.y));
    let g = Girdle {
        body: field,
        p,
        o: Arc::new(GOutlines::new(&p)),
        torso,
        thighs: [Frame::new(rk, rh), Frame::new(lk, lh)],
    };
    println!("girdle: torso axis ({:.0}, {:.0}); right thigh axis from the knee ({:.0}, {:.0}, {:.0}) to the hip ({:.0}, {:.0}, {:.0}) ({:.0} s)", torso.0, torso.1, rk.x, rk.y, rk.z, rh.x, rh.y, rh.z, t0.elapsed().as_secs_f64());
    let threads = std::env::var("PAD_THREADS").ok().and_then(|v| v.parse::<usize>().ok()).unwrap_or(4).max(1);
    let verts = b.mesh.vertices.clone();
    let mut parts: Vec<(GPart, IndexedMesh, f64)> = Vec::new();
    for chunk in GPart::all().chunks(threads) {
        let done: Vec<(GPart, IndexedMesh, f64)> = std::thread::scope(|sc| {
            let hs: Vec<_> = chunk
                .iter()
                .filter_map(|&part| {
                    let bounds = part_bounds(&g, part, &verts)?;
                    let gg = g.clone();
                    Some(sc.spawn(move || {
                        let s = Solid::user_fn(move |q| gg.field(part, q), bounds);
                        let m = mesh_of(&s, part.cell());
                        let grams = mesh_volume(&m).abs() / 1000.0 * part.material().density / 1000.0;
                        (part, m, grams)
                    }))
                })
                .collect();
            hs.into_iter().map(|h| h.join().unwrap()).collect()
        });
        parts.extend(done);
    }
    // The shorts: the body's skin between their top and bottom (and, on the thighs, above
    // their bottom), weighed from its area.
    let in_shorts = |v: &Point3<f64>| v.z > p.shorts_bottom && v.z < p.shorts_top;
    let mut area = 0.0;
    for f in &b.mesh.faces {
        let [a, c, d] = f.map(|i| b.mesh.vertices[i as usize]);
        if in_shorts(&Point3::from((a.coords + c.coords + d.coords) / 3.0)) {
            area += 0.5 * (c - a).cross(&(d - a)).norm();
        }
    }
    let shorts_g = area * p.fabric / 1000.0 * FABRIC.density / 1000.0;
    let pads_g: f64 = parts.iter().map(|x| x.2).sum();
    let mut lines = vec![
        format!("girdle, round 1 (track G2), on the stand-in body ({})", scan_path.display()),
        format!(
            "layers (placeholders): shorts {} mm fabric from {:.0} to {:.0} mm up; thigh pads {} mm foam + {} mm PA 12, bottom at {:.0}; hip pads {} + {}; kidney {} + {}; tailbone {} + {}",
            p.fabric, p.shorts_bottom, p.shorts_top, p.thigh_foam, p.thigh_wall, p.thigh_bottom, p.hip_foam, p.hip_wall, p.kidney_foam, p.kidney_wall, p.tail_foam, p.tail_wall
        ),
        format!("girdle {:.0} g: pads {:.0} g, shorts {:.0} g ({:.0} cm2 of fabric)", pads_g + shorts_g, pads_g, shorts_g, area / 100.0),
    ];
    for (part, _, gr) in &parts {
        lines.push(format!("  {:<18} {:>6.1} g  ({})", part.name(), gr, part.material().name));
    }
    for l in &lines {
        println!("{l}");
    }
    std::fs::write(docs.join("summary.txt"), lines.join("\n") + "\n").map_err(|e| e.to_string())?;
    for (part, m, _) in &parts {
        save_mesh(m, out.join(format!("{}.stl", part.name().replace([' ', ','], "_").replace("__", "_")))).map_err(|e| e.to_string())?;
    }
    // Drawn: the body with the shorts as dark fabric, the pads' shells (foam hidden).
    let skin = [0.85, 0.70, 0.60];
    let fabric = [0.13, 0.14, 0.16];
    let colors: Vec<[f64; 3]> = b.mesh.vertices.iter().map(|v| if in_shorts(v) { fabric } else { skin }).collect();
    let mut ls = vec![Layer { mesh: &b.mesh, color: skin, vertex_colors: Some(&colors), spec: 0.0 }];
    for (part, m, _) in &parts {
        if part.shell() {
            ls.push(Layer::solid(m, part.color()).with_spec(0.3));
        }
    }
    let views = [
        View::looking(-Vector3::x(), Vector3::z()),
        View::looking(Vector3::y(), Vector3::z()),
        View::looking(Vector3::x(), Vector3::z()),
        View::looking(Vector3::new(-0.6, 0.65, -0.25), Vector3::z()),
    ];
    draw(&ls, &views, 0.9, &docs.join("girdle.png"), Style::pretty())?;
    println!("drew docs/girdle/girdle.png ({:.0} s in all)", t0.elapsed().as_secs_f64());
    Ok(())
}
