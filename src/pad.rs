//! Track D2: the shin pad (docs/PLAN.md, track D).
//!
//! Built on the stand-in leg (`leg.rs`) as it stands in the skate: leaned forward like the boot
//! (`hockey_boot::lean_dx_at`), with the boot's own meshes (`out/boot`) for the parts the pad
//! meets. Every surface is an offset from one body:
//! - **tongue under** (Jon's way, the default): the leg, the tongue and the laces' bow blended,
//!   so the pad's bottom comes down over the tongue and its liner sits on it;
//! - **tongue over**: the leg alone, the pad's bottom tucked behind the tongue; the run reports
//!   how far the tongue is in its way.
//!
//! The parts: a hard shin shell (PA 12) over a foam liner, from just above the boot's cuff (its
//! top edge followed all round, `clear_boot` above it) to below the knee; a knee cap over its
//! own liner, overlapping the shell's top; soft calf wings (TPU over foam) round the sides; two
//! straps round the back of the calf. Thicknesses are placeholders until D3 sizes them for a
//! puck.
//!
//! **Comparison pads, on the same leg** (Jon, 2026-11-12: not his own pads, a different body):
//! a stock-style 14" pad and an undersized 13" one, sized by the retail rule (the knee's centre
//! to the top of the boot). A stock shell isn't fitted: it has to clear the biggest leg its size
//! takes, so its inside here is the convex hull of the leg with ANSUR's 95th-percentile girths,
//! pulled back onto the shin as its straps would. The undersized one is a size shorter and
//! clears the median leg (the smaller size's biggest legs, assumed about our median). Same
//! layers, materials and coverage angles as the fitted pad, so the comparison is fit and length.
//!
//! Frame: the foot's standard frame, leaned (x forward, y toward the medial side on this right
//! leg, z up, mm). Angles round the leg (`theta`, degrees) are about an axis through the knee
//! joint and the shin's front line, 0 at the front, + medial.

use std::f64::consts::FRAC_PI_2;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

use cortenforge::cf_design::{Aabb, Solid};
use cortenforge::cf_scan_prep_core::{auto_cap_open_boundaries, clip_mesh_against_plane};
use cortenforge::mesh::io::{load_mesh, save_mesh};
use cortenforge::mesh::repair::weld_vertices;
use cortenforge::mesh::sdf::{CachedGridSdf, ParitySign, PseudoNormalSign, Signed, TriMeshDistance, WALL_THRESHOLD_FACTOR_DEFAULT};
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::hockey_boot::{HockeyParams, landmarks, lean_dx_at, mesh_volume};
use crate::leg::{self, ansur};
use crate::materials::{LINER_FOAM, Material, PA12_MJF, STRAP, TPU};
use crate::render::{Layer, Style, View, render_styled};
use crate::round1::mesh_of;

/// mm of arc per radian round the leg: turns an angle into a distance for the parts' edges.
const R_REF: f64 = 45.0;

/// Round 2 (2026-11-12), the modern pad (Jon: "having them look like modern shinpads ... but
/// maybe a bit lighter because of the custom fit instead of fully relying on squish"; his
/// reference: a stock pad's 3D model). Hard parts thinner, stiffened by shape instead.
#[derive(Clone, Copy, Debug)]
pub struct PadParams {
    /// Sock, mm.
    pub ease: f64,
    /// Liner at the front of the shin and at the side panels' back edges (placeholders until
    /// D3).
    pub liner_front: f64,
    pub liner_side: f64,
    /// Hard parts' wall, PA 12 (round 1: 2.5 mm, plain).
    pub wall: f64,
    /// The liner tapers from the front to `side_to` degrees round; the side panels reach
    /// `side_overlap` mm under the front shell's edge (round 3: the outlines are curves,
    /// `Outlines`).
    pub side_to: f64,
    pub side_overlap: f64,
    /// The liner shows this far past the shells' edges, mm.
    pub liner_border: f64,
    /// The front shell's ridge down the shin: height and width (mm of arc).
    pub crest_h: f64,
    pub crest_w: f64,
    /// Moulded segments: grooves across the front shell, depth and width, at these fractions
    /// of its height.
    pub groove_d: f64,
    pub groove_w: f64,
    pub grooves_at: [f64; 2],
    /// Vent slots in each side panel: count, width (mm of arc) and height.
    pub vents: usize,
    pub vent_w: f64,
    pub vent_h: f64,
    /// The shin shell's top below the knee joint, mm.
    pub shell_below_knee: f64,
    /// Clearance between the pad's bottom edge and the boot's top edge, all round.
    pub clear_boot: f64,
    /// How far above the knee joint the pad reaches (the thigh guard's top), for boxes and
    /// counts. The knee shield: its liner, its dome (extra stand-off over the kneecap, filled
    /// with liner), the V ridge across it.
    pub knee_above: f64,
    pub knee_liner: f64,
    pub dome_h: f64,
    pub chevron_h: f64,
    /// The thigh guard's foam.
    pub thigh_foam: f64,
    /// Calf wraps: from the side panels to this angle; foam and a thin skin; their top below the
    /// knee joint.
    pub wing_to: f64,
    pub wing_foam: f64,
    pub wing_wall: f64,
    pub wing_below_knee: f64,
    /// Straps across the back from this angle; width, thickness; heights as fractions from the
    /// pad's front bottom to the knee joint.
    pub strap_from: f64,
    pub strap_w: f64,
    pub strap_t: f64,
    pub strap_at: [f64; 2],
    /// Tongue over: how far the pad's bottom reaches down behind the tongue's top.
    pub over_overlap: f64,
}

impl Default for PadParams {
    fn default() -> Self {
        PadParams {
            ease: 1.0,
            liner_front: 14.0,
            liner_side: 5.0,
            wall: 2.0,
            side_to: 105.0,
            side_overlap: 6.0,
            liner_border: 3.0,
            crest_h: 2.5,
            crest_w: 16.0,
            groove_d: 0.8,
            groove_w: 5.0,
            grooves_at: [0.36, 0.68],
            vents: 3,
            vent_w: 7.0,
            vent_h: 34.0,
            shell_below_knee: 20.0,
            clear_boot: 10.0,
            knee_above: 115.0,
            knee_liner: 9.0,
            dome_h: 5.0,
            chevron_h: 1.8,
            thigh_foam: 6.0,
            wing_to: 150.0,
            wing_foam: 6.0,
            wing_wall: 1.0,
            wing_below_knee: 60.0,
            strap_from: 150.0,
            strap_w: 38.0,
            strap_t: 2.0,
            strap_at: [0.3, 0.72],
            over_overlap: 30.0,
        }
    }
}

/// A closed mesh's signed distance: a grid (mesh-sdf's flood-filled sign, so enclosed voids
/// count as inside), plus the distance to the grid's box outside it (the grid clamps there,
/// CORTENFORGE.md), and exact distances within `near` mm of the surface if asked.
#[derive(Clone)]
pub struct Field {
    grid: Arc<CachedGridSdf>,
    exact: Option<Arc<Signed<TriMeshDistance, PseudoNormalSign>>>,
    near: f64,
    lo: Point3<f64>,
    hi: Point3<f64>,
    /// Evaluate at p + shift: moves the surface by -shift.
    pub shift: Vector3<f64>,
}

impl Field {
    pub fn new(mesh: &IndexedMesh, margin: f64, cell: f64, near: Option<f64>, what: &str) -> Result<Self, String> {
        let t = Instant::now();
        let lo = mesh.vertices.iter().fold(Vector3::repeat(f64::MAX), |a, v| a.inf(&v.coords)) - Vector3::repeat(margin);
        let hi = mesh.vertices.iter().fold(Vector3::repeat(f64::MIN), |a, v| a.sup(&v.coords)) + Vector3::repeat(margin);
        let bounds = Aabb::new(Point3::from(lo), Point3::from(hi));
        let dist = TriMeshDistance::new(mesh.clone()).map_err(|e| format!("{what}: distance: {e}"))?;
        let (grid, report) =
            CachedGridSdf::build(&dist, bounds, cell, WALL_THRESHOLD_FACTOR_DEFAULT).map_err(|e| format!("{what}: grid: {e:?}"))?;
        println!("  {what}: grid {cell} mm, {:?} nodes, {:.1} s", report.dims, t.elapsed().as_secs_f64());
        let exact = near.map(|_| {
            let sign = PseudoNormalSign::from_distance(&dist);
            Arc::new(Signed { distance: dist, sign })
        });
        Ok(Field { grid: Arc::new(grid), exact, near: near.unwrap_or(0.0), lo: bounds.min, hi: bounds.max, shift: Vector3::zeros() })
    }

    pub fn eval(&self, p: Point3<f64>) -> f64 {
        let p = p + self.shift;
        let out = Vector3::new(
            (self.lo.x - p.x).max(p.x - self.hi.x).max(0.0),
            (self.lo.y - p.y).max(p.y - self.hi.y).max(0.0),
            (self.lo.z - p.z).max(p.z - self.hi.z).max(0.0),
        )
        .norm();
        let g = self.grid.signed_distance(p) + out;
        match &self.exact {
            Some(e) if out == 0.0 && g.abs() <= self.near => e.evaluate(p),
            _ => g,
        }
    }
}

/// Exact distance to a closed mesh, only near a box (far from it, the distance to the box).
/// Inside by ray parity: the pseudo-normal test misread points beside the boot shell's thin
/// edges as inside it (a pad "30 mm into" a 2.5 mm shell, 2026-11-12).
#[derive(Clone)]
pub struct NearMesh {
    exact: Arc<Signed<TriMeshDistance, ParitySign>>,
    lo: Point3<f64>,
    hi: Point3<f64>,
}

impl NearMesh {
    pub fn new(mesh: &IndexedMesh, what: &str) -> Result<Self, String> {
        let lo = mesh.vertices.iter().fold(Vector3::repeat(f64::MAX), |a, v| a.inf(&v.coords));
        let hi = mesh.vertices.iter().fold(Vector3::repeat(f64::MIN), |a, v| a.sup(&v.coords));
        let dist = TriMeshDistance::new(mesh.clone()).map_err(|e| format!("{what}: distance: {e}"))?;
        let sign = ParitySign::new(mesh).map_err(|e| format!("{what}: parity: {e}"))?;
        Ok(NearMesh { exact: Arc::new(Signed { distance: dist, sign }), lo: Point3::from(lo), hi: Point3::from(hi) })
    }

    pub fn eval(&self, p: Point3<f64>) -> f64 {
        let out = Vector3::new(
            (self.lo.x - p.x).max(p.x - self.hi.x).max(0.0),
            (self.lo.y - p.y).max(p.y - self.hi.y).max(0.0),
            (self.lo.z - p.z).max(p.z - self.hi.z).max(0.0),
        )
        .norm();
        if out > 20.0 { out } else { self.exact.evaluate(p) }
    }
}

/// What the pad wraps.
#[derive(Clone)]
pub enum Body {
    /// The leg, and for tongue-under the tongue and the laces' top, blended by `k` mm.
    Fitted { leg: Field, tongue: Option<Field>, laces: Option<NearMesh> },
    /// A stock pad's inside: a convex hull (moved by its `shift`).
    Hull(Field),
}

pub(crate) fn smin(a: f64, b: f64, k: f64) -> f64 {
    let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
    b + (a - b) * h - k * h * (1.0 - h)
}

pub(crate) fn smax(a: f64, b: f64, k: f64) -> f64 {
    -smin(-a, -b, k)
}

pub(crate) fn smoothstep(e0: f64, e1: f64, x: f64) -> f64 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

impl Body {
    pub fn eval(&self, p: Point3<f64>) -> f64 {
        match self {
            Body::Fitted { leg, tongue, laces } => {
                let mut d = leg.eval(p);
                if let Some(t) = tongue {
                    d = smin(d, t.eval(p), 3.0);
                }
                if let Some(l) = laces {
                    // Only the bow and the top crossing: the pad never reaches lower.
                    if p.z > 115.0 {
                        d = smin(d, l.eval(p), 2.0);
                    }
                }
                d
            }
            Body::Hull(h) => h.eval(p),
        }
    }
}

/// The axis round which angles are measured.
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub a: Point3<f64>,
    pub u: Vector3<f64>,
    pub fwd: Vector3<f64>,
    pub side: Vector3<f64>,
}

impl Frame {
    pub fn new(a: Point3<f64>, b: Point3<f64>) -> Self {
        let u = (b - a).normalize();
        let fwd = (Vector3::x() - u * u.x).normalize();
        let side = u.cross(&fwd);
        Frame { a, u, fwd, side }
    }

    pub fn axis_at(&self, z: f64) -> Point3<f64> {
        self.a + self.u * ((z - self.a.z) / self.u.z)
    }

    /// Angle round the leg at p, degrees: 0 at the front, + toward `side` (medial here).
    pub fn theta(&self, p: Point3<f64>) -> f64 {
        let r = p - self.axis_at(p.z);
        r.dot(&self.side).atan2(r.dot(&self.fwd)).to_degrees()
    }

    pub fn dir(&self, theta_deg: f64) -> Vector3<f64> {
        let t = theta_deg.to_radians();
        self.fwd * t.cos() + self.side * t.sin()
    }
}

/// A height round the leg, every 5 degrees from -180, linearly interpolated.
#[derive(Clone, Debug)]
pub struct Ring(pub Vec<f64>);

impl Ring {
    pub fn at(&self, theta: f64) -> f64 {
        let n = self.0.len();
        let x = (theta + 180.0).rem_euclid(360.0) / (360.0 / n as f64);
        let i = x.floor() as usize % n;
        let f = x - x.floor();
        self.0[i] * (1.0 - f) + self.0[(i + 1) % n] * f
    }

    pub fn flat(z: f64) -> Self {
        Ring(vec![z; 72])
    }
}

/// The boot's top edge round the leg (shell top, 5-degree bins), with the open throat's gap
/// bridged from its sides (the tongue fills it), smoothed over +-10 degrees.
pub fn boot_top(shell: &IndexedMesh, f: &Frame) -> Ring {
    let mut top = vec![f64::NAN; 72];
    for v in shell.vertices.iter().filter(|v| v.z > 90.0) {
        let c = f.axis_at(v.z);
        if ((v.x - c.x).powi(2) + (v.y - c.y).powi(2)).sqrt() > 130.0 {
            continue;
        }
        let b = (((f.theta(*v) + 180.0) / 5.0) as usize).min(71);
        if top[b].is_nan() || v.z > top[b] {
            top[b] = v.z;
        }
    }
    // Empty bins: their neighbours.
    for _ in 0..72 {
        for b in 0..72 {
            if top[b].is_nan() {
                let (l, r) = (top[(b + 71) % 72], top[(b + 1) % 72]);
                top[b] = if l.is_nan() { r } else if r.is_nan() { l } else { 0.5 * (l + r) };
            }
        }
    }
    // The throat: between -35 and +35 degrees the shell dips (no shell over the tongue); bridge
    // it straight from the values at +-35.
    let at = |t: f64| top[(((t + 180.0) / 5.0) as usize).min(71)];
    let (l, r) = (at(-35.0), at(35.0));
    for b in 0..72 {
        let t = -180.0 + 5.0 * b as f64 + 2.5;
        if t.abs() < 35.0 {
            let s = (t + 35.0) / 70.0;
            top[b] = top[b].max(l * (1.0 - s) + r * s);
        }
    }
    let sm: Vec<f64> = (0..72).map(|b| (0..5).map(|k| top[(b + 72 + k - 2) % 72]).sum::<f64>() / 5.0).collect();
    Ring(sm)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Part {
    /// The front shell: a tapered shield down the shin, a ridge down it, moulded segments.
    ShinShell,
    /// Side panels following the front shell's edge and stepped over it, vented: inside (true),
    /// outside.
    SidePanel(bool),
    /// The liner under the front shell and side panels, showing as a border round them.
    ShinLiner,
    /// The knee shield: one piece over the kneecap and the sides of the knee (its ears), domed,
    /// a V ridge across it, its bottom a V lapped over the front shell's top.
    KneeCap,
    KneeLiner,
    /// The thigh guard, tucked under the knee shield's top, and its foam.
    ThighGuard,
    ThighFoam,
    /// Calf wraps: foam under a thin skin.
    WingFoam(bool),
    Wing(bool),
    Strap(usize),
}

impl Part {
    pub fn all() -> Vec<Part> {
        let mut v = vec![Part::ShinShell, Part::ShinLiner, Part::KneeCap, Part::KneeLiner, Part::ThighGuard, Part::ThighFoam];
        for m in [true, false] {
            v.extend([Part::SidePanel(m), Part::WingFoam(m), Part::Wing(m)]);
        }
        v.extend([Part::Strap(0), Part::Strap(1)]);
        v
    }

    /// The hard parts (the shell pieces), the ones a puck meets.
    pub fn hard(&self) -> bool {
        matches!(self, Part::ShinShell | Part::SidePanel(_) | Part::KneeCap | Part::ThighGuard)
    }

    pub fn foam(&self) -> bool {
        matches!(self, Part::ShinLiner | Part::KneeLiner | Part::ThighFoam | Part::WingFoam(_))
    }

    pub fn name(&self) -> String {
        let side = |m: &bool| if *m { "inside" } else { "outside" };
        match self {
            Part::ShinShell => "front shell".into(),
            Part::SidePanel(m) => format!("side panel, {}", side(m)),
            Part::ShinLiner => "shin liner".into(),
            Part::KneeCap => "knee shield".into(),
            Part::KneeLiner => "knee liner".into(),
            Part::ThighGuard => "thigh guard".into(),
            Part::ThighFoam => "thigh foam".into(),
            Part::WingFoam(m) => format!("calf foam, {}", side(m)),
            Part::Wing(m) => format!("calf skin, {}", side(m)),
            Part::Strap(i) => format!("strap {}", i + 1),
        }
    }

    pub fn material(&self) -> Material {
        match self {
            p if p.hard() => PA12_MJF,
            p if p.foam() => LINER_FOAM,
            Part::Wing(_) => TPU,
            _ => STRAP,
        }
    }

    pub fn cell(&self) -> f64 {
        match self {
            p if p.hard() => 0.6,
            Part::Wing(_) => 0.5,
            Part::Strap(_) => 0.6,
            _ => 1.0,
        }
    }

    /// Two-tone hard parts like a modern pad (light front pieces, dark side pieces); the liners
    /// as dark fabric, since they show round the shells' edges.
    pub fn color(&self) -> [f64; 3] {
        match self {
            Part::ShinShell | Part::KneeCap | Part::ThighGuard => [0.62, 0.72, 0.84],
            Part::SidePanel(_) => [0.17, 0.18, 0.20],
            p if p.foam() => [0.11, 0.12, 0.14],
            Part::Wing(_) => [0.21, 0.22, 0.24],
            _ => [0.06, 0.06, 0.07],
        }
    }
}

/// A smooth bump, 1 at 0, falling to 0 (with zero slope) at |x| = 1.
pub(crate) fn bump(x: f64) -> f64 {
    let t = 1.0 - x * x;
    if t > 0.0 { t * t } else { 0.0 }
}

/// A rounded rectangle's signed distance in 2D: centred, half-extents `hx`, `hy`, corner `r`.
pub(crate) fn rounded_rect(x: f64, y: f64, hx: f64, hy: f64, r: f64) -> f64 {
    let qx = x.abs() - (hx - r);
    let qy = y.abs() - (hy - r);
    (qx.max(0.0).powi(2) + qy.max(0.0).powi(2)).sqrt() + qx.max(qy).min(0.0) - r
}

/// Chaikin's corner cutting on a closed polygon: each round replaces every edge by its points
/// at 1/4 and 3/4, so the outline converges to a smooth (quadratic B-spline) curve.
pub(crate) fn chaikin(p: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = p.len();
    let mut out = Vec::with_capacity(2 * n);
    for i in 0..n {
        let (a, b) = (p[i], p[(i + 1) % n]);
        out.push((0.75 * a.0 + 0.25 * b.0, 0.75 * a.1 + 0.25 * b.1));
        out.push((0.25 * a.0 + 0.75 * b.0, 0.25 * a.1 + 0.75 * b.1));
    }
    out
}

/// A symmetric outline from its half on the medial side (s >= 0), listed from the top of the
/// centre line round to the bottom of it.
pub(crate) fn mirrored(half: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut v = half.to_vec();
    for i in (1..half.len() - 1).rev() {
        v.push((-half[i].0, half[i].1));
    }
    v
}

/// A part's outline on the leg laid flat: `s` round the leg (mm of arc at `R_REF` per radian,
/// 0 at the front, + medial), `z` up. A closed polygon smoothed by Chaikin's corner cutting,
/// with its signed distance (negative inside) on a 0.5 mm grid, bilinear between nodes.
#[derive(Clone)]
pub struct Outline {
    s0: f64,
    z0: f64,
    cell: f64,
    ns: usize,
    nz: usize,
    d: Arc<Vec<f32>>,
    pub z_min: f64,
    pub z_max: f64,
    /// The smoothed outline, for drawing.
    pub points: Arc<Vec<(f64, f64)>>,
}

impl Outline {
    pub fn new(poly: &[(f64, f64)], rounds: usize) -> Self {
        let mut p = poly.to_vec();
        for _ in 0..rounds {
            p = chaikin(&p);
        }
        let cell = 0.5;
        let (mut s_lo, mut s_hi, mut z_lo, mut z_hi) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
        for &(s, z) in &p {
            s_lo = s_lo.min(s);
            s_hi = s_hi.max(s);
            z_lo = z_lo.min(z);
            z_hi = z_hi.max(z);
        }
        let m = 25.0;
        let (s0, z0) = (s_lo - m, z_lo - m);
        let ns = ((s_hi - s_lo + 2.0 * m) / cell).ceil() as usize + 1;
        let nz = ((z_hi - z_lo + 2.0 * m) / cell).ceil() as usize + 1;
        let segs: Vec<((f64, f64), (f64, f64))> = (0..p.len()).map(|i| (p[i], p[(i + 1) % p.len()])).collect();
        let d: Vec<f32> = (0..ns * nz)
            .map(|k| {
                let (s, z) = (s0 + (k % ns) as f64 * cell, z0 + (k / ns) as f64 * cell);
                let mut best = f64::MAX;
                let mut inside = false;
                for &((ax, ay), (bx, by)) in &segs {
                    let (ex, ey) = (bx - ax, by - ay);
                    let t = (((s - ax) * ex + (z - ay) * ey) / (ex * ex + ey * ey).max(1e-12)).clamp(0.0, 1.0);
                    let (dx, dy) = (s - ax - t * ex, z - ay - t * ey);
                    best = best.min(dx * dx + dy * dy);
                    if (ay > z) != (by > z) && s < ax + (z - ay) / (by - ay) * ex {
                        inside = !inside;
                    }
                }
                (if inside { -best.sqrt() } else { best.sqrt() }) as f32
            })
            .collect();
        Outline { s0, z0, cell, ns, nz, d: Arc::new(d), z_min: z_lo, z_max: z_hi, points: Arc::new(p) }
    }

    pub fn eval(&self, s: f64, z: f64) -> f64 {
        let fx = ((s - self.s0) / self.cell).clamp(0.0, (self.ns - 1) as f64);
        let fz = ((z - self.z0) / self.cell).clamp(0.0, (self.nz - 1) as f64);
        let (i, j) = ((fx as usize).min(self.ns - 2), (fz as usize).min(self.nz - 2));
        let (u, v) = (fx - i as f64, fz - j as f64);
        let at = |a: usize, b: usize| self.d[b * self.ns + a] as f64;
        let inner = (1.0 - v) * ((1.0 - u) * at(i, j) + u * at(i + 1, j)) + v * ((1.0 - u) * at(i, j + 1) + u * at(i + 1, j + 1));
        // Outside the grid: add the distance to it (the grid's edge is > 25 mm from the outline).
        let out_s = (self.s0 - s).max(s - (self.s0 + (self.ns - 1) as f64 * self.cell)).max(0.0);
        let out_z = (self.z0 - z).max(z - (self.z0 + (self.nz - 1) as f64 * self.cell)).max(0.0);
        inner + (out_s * out_s + out_z * out_z).sqrt()
    }
}

/// The pad's outlines on the leg laid flat (round 3, 2026-11-12: curves after Jon's reference,
/// in place of round 2's rectangles). Heights from the knee joint `kz`; `s` in mm of arc at
/// `R_REF` (1 deg = 0.785 mm: 45 deg = 35, 90 deg = 71, 150 deg = 118). The real width is the
/// angle times the leg's own radius, so a constant `s` narrows toward the ankle by itself. The
/// pad's bottom edge follows the boot separately (`Pad::z_bot`), so the outlines run past it.
pub struct Outlines {
    /// The front shell: a shield, widest below the knee, tapering to the ankle.
    pub front: Outline,
    /// The side panels' outline on the medial half (s >= 0, used mirrored); the front shell's
    /// inside is cut from it, less the overlap.
    pub side: Outline,
    /// The knee shield, ears at the sides of the knee, a V at its bottom.
    pub knee: Outline,
    /// The thigh guard, under the knee shield's top.
    pub thigh: Outline,
    /// The calf wraps' outline on the medial half (s >= 0, used mirrored).
    pub calf: Outline,
}

impl Outlines {
    pub fn new(kz: f64) -> Self {
        let front = mirrored(&[
            (0.0, kz - 12.0),
            (22.0, kz - 14.0),
            (40.0, kz - 22.0),
            (50.0, kz - 40.0),
            (52.0, kz - 90.0),
            (47.0, 300.0),
            (40.0, 230.0),
            (36.0, 170.0),
            (35.0, 90.0),
            (0.0, 90.0),
        ]);
        // The side panel's top tucks ~10 mm under the knee shield's bottom edge all the way round
        // (checked with `examples/probe_pad_outlines.rs`; round 3's first two tries stopped 1-33 mm
        // short of it and left skin showing).
        let side = [
            (28.0, kz - 12.0),
            (50.0, kz - 16.0),
            (66.0, kz - 12.0),
            (80.0, kz - 4.0),
            (90.0, kz + 8.0),
            (93.0, kz - 40.0),
            (86.0, kz - 100.0),
            (84.0, 300.0),
            (78.0, 220.0),
            (72.0, 150.0),
            (70.0, 90.0),
            (28.0, 90.0),
        ];
        let knee = mirrored(&[
            (0.0, kz + 92.0),
            (28.0, kz + 88.0),
            (48.0, kz + 72.0),
            (62.0, kz + 50.0),
            (80.0, kz + 38.0),
            (90.0, kz + 15.0),
            (86.0, kz - 12.0),
            (66.0, kz - 26.0),
            (44.0, kz - 30.0),
            (22.0, kz - 36.0),
            (0.0, kz - 46.0),
        ]);
        let thigh = mirrored(&[(0.0, kz + 115.0), (28.0, kz + 112.0), (44.0, kz + 98.0), (42.0, kz + 80.0), (24.0, kz + 70.0), (0.0, kz + 66.0)]);
        let calf = [(74.0, kz - 70.0), (108.0, kz - 76.0), (121.0, kz - 110.0), (123.0, 250.0), (114.0, 175.0), (88.0, 150.0), (72.0, 150.0)];
        Outlines {
            front: Outline::new(&front, 4),
            side: Outline::new(&side, 4),
            knee: Outline::new(&knee, 4),
            thigh: Outline::new(&thigh, 4),
            calf: Outline::new(&calf, 4),
        }
    }
}

/// One pad on the leg.
#[derive(Clone)]
pub struct Pad {
    pub name: String,
    pub body: Arc<Body>,
    /// The leg itself (straps, bulk).
    pub leg: Field,
    pub f: Frame,
    pub p: PadParams,
    /// The pad's bottom edge round the leg.
    pub z_bot: Ring,
    pub knee_z: f64,
    /// The leg's box over the pad's heights, for part bounds.
    pub lo: Vector3<f64>,
    pub hi: Vector3<f64>,
    /// Tongue under, flush: the shell's offset taken from the leg alone (a lap joint over the
    /// tongue), not from the leg and tongue blended.
    pub flush: bool,
    pub o: Arc<Outlines>,
}

impl Pad {
    fn liner_at(&self, th: f64) -> f64 {
        let s = (th.abs() / self.p.side_to).min(1.0);
        self.p.liner_side + (self.p.liner_front - self.p.liner_side) * (FRAC_PI_2 * s).cos().powi(2)
    }

    /// A reference height below the knee (bulk is measured below it).
    pub fn shell_top(&self) -> f64 {
        self.knee_z - self.p.shell_below_knee
    }

    pub fn z_front_bottom(&self) -> f64 {
        self.z_bot.at(0.0)
    }

    fn band(d: f64, a: f64, b: f64) -> f64 {
        (a - d).max(d - b)
    }

    /// (theta degrees, s mm of arc, z) at a point.
    fn tsz(&self, q: Point3<f64>) -> (f64, f64, f64) {
        let th = self.f.theta(q);
        (th, th.to_radians() * R_REF, q.z)
    }

    /// A shin shell piece between `inner` and `outer` mm off what it is offset from. Flush (tongue
    /// under, a lap joint): off the leg, and where the tongue would come through it, lifted to
    /// clear the tongue by 1 mm, a small local bulge (2026-11-12: with 14 mm of front foam the
    /// tongue's curled top corners still stood 2.8 mm proud of a shell offset from the leg alone).
    fn shell_band(&self, q: Point3<f64>, inner: f64, outer: f64) -> f64 {
        if !self.flush {
            return Self::band(self.body.eval(q), inner, outer);
        }
        let mut base = self.leg.eval(q) - inner;
        if let Body::Fitted { tongue: Some(t), .. } = &*self.body {
            base = smin(base, t.eval(q) - 1.0, 3.0);
        }
        (-base).max(base - (outer - inner))
    }

    /// The knee shield's dome: extra stand-off over the kneecap (its centre ~15 mm above the
    /// knee joint), falling smoothly to 0 ~55 mm away.
    fn dome(&self, s: f64, z: f64) -> f64 {
        let dz = (z - (self.knee_z + 15.0)) / 55.0;
        self.p.dome_h * bump((dz * dz + (s / 55.0).powi(2)).sqrt())
    }

    /// The side panel's outline at (s, z): its own outline (mirrored), less the front shell's
    /// inside beyond the overlap.
    fn side_panel(&self, s: f64, z: f64) -> f64 {
        self.o.side.eval(s.abs(), z).max(-(self.o.front.eval(s, z) + self.p.side_overlap))
    }

    /// How far the side panel lifts to stand on the front shell's edge (0..1).
    fn side_step(&self, s: f64, z: f64) -> f64 {
        1.0 - smoothstep(-1.0, 5.0, self.o.front.eval(s, z))
    }

    fn front_outer(&self, s: f64, z: f64, lt: f64) -> f64 {
        let p = &self.p;
        let crest = p.crest_h * bump(s / (0.5 * p.crest_w));
        let (z0, z1) = (self.z_front_bottom(), self.o.front.z_max);
        let groove: f64 = p.grooves_at.iter().map(|f| p.groove_d * bump((z - (z0 + f * (z1 - z0))) / (0.5 * p.groove_w))).sum();
        p.ease + lt + p.wall + crest - groove
    }

    fn side_inner(&self, s: f64, z: f64, lt: f64) -> f64 {
        self.p.ease + lt + self.side_step(s, z) * (self.p.wall + 0.4)
    }

    /// The vent slots' centres in the medial side panel (s, z); mirrored for the lateral one.
    fn vents(&self) -> Vec<(f64, f64)> {
        let p = &self.p;
        let s = 0.5 * (52.0 + 84.0);
        let th = s / R_REF * 180.0 / std::f64::consts::PI;
        let (z0, z1) = (self.z_bot.at(th) + 35.0, self.knee_z - 95.0);
        (0..p.vents).map(|i| (s, z0 + (i as f64 + 0.5) * (z1 - z0) / p.vents as f64)).collect()
    }

    /// The part's own region on the leg laid flat (a pseudo-distance, negative inside).
    fn region(&self, part: Part, q: Point3<f64>) -> f64 {
        let (th, s, z) = self.tsz(q);
        let p = &self.p;
        let o = &self.o;
        let bot = self.z_bot.at(th) - z;
        let on_side = |medial: bool| if medial { -s } else { s };
        match part {
            Part::ShinShell => smax(o.front.eval(s, z), bot, 3.0),
            Part::SidePanel(medial) => {
                let mut e = smax(smax(self.side_panel(s, z), bot, 3.0), on_side(medial), 1.0);
                for (sc, zc) in self.vents() {
                    let slot = rounded_rect(s.abs() - sc, z - zc, 0.5 * p.vent_w, 0.5 * p.vent_h, 0.5 * p.vent_w);
                    e = e.max(-slot);
                }
                e
            }
            Part::ShinLiner => smax(o.front.eval(s, z).min(o.side.eval(s.abs(), z)) - p.liner_border, bot, 3.0),
            Part::KneeCap => o.knee.eval(s, z),
            Part::KneeLiner => {
                let shin = o.front.eval(s, z).min(o.side.eval(s.abs(), z)) - p.liner_border;
                (o.knee.eval(s, z) + 2.0).max(0.5 - shin).max(0.5 - o.thigh.eval(s, z))
            }
            Part::ThighGuard => o.thigh.eval(s, z),
            Part::ThighFoam => o.thigh.eval(s, z) + 2.0,
            Part::WingFoam(medial) | Part::Wing(medial) => {
                let e = smax(o.calf.eval(s.abs(), z), 1.0 - o.side.eval(s.abs(), z), 2.0);
                smax(smax(e, bot + 15.0, 4.0), on_side(medial), 1.0)
            }
            Part::Strap(i) => {
                let zf = self.z_front_bottom();
                let zs = zf + p.strap_at[i] * (self.knee_z - zf);
                ((z - zs).abs() - 0.5 * p.strap_w).max((p.strap_from - th.abs()).to_radians() * R_REF)
            }
        }
    }

    /// The part's field (negative inside).
    pub fn field(&self, part: Part, q: Point3<f64>) -> f64 {
        let r = self.region(part, q);
        if r > 3.0 {
            return r;
        }
        let p = &self.p;
        let (th, s, z) = self.tsz(q);
        let lt = self.liner_at(th);
        let band = match part {
            // Flush: the shell keeps its offset from the leg itself; the liner fills from what
            // the pad wraps (leg, tongue, laces) out to the shell, so over the tongue it is
            // thinner and the outside stays flush.
            Part::ShinLiner if self.flush => (p.ease - self.body.eval(q)).max(self.leg.eval(q) - (p.ease + lt)),
            Part::ShinLiner => Self::band(self.body.eval(q), p.ease, p.ease + lt),
            Part::ShinShell => self.shell_band(q, p.ease + lt, self.front_outer(s, z, lt)),
            Part::SidePanel(_) => {
                let inner = self.side_inner(s, z, lt);
                self.shell_band(q, inner, inner + p.wall)
            }
            Part::KneeLiner => Self::band(self.body.eval(q), p.ease, p.ease + p.knee_liner + self.dome(s, z)),
            // The shield lifts over whatever it laps: the front shell's top, the side panels'
            // tops, the thigh guard's bottom.
            Part::KneeCap => {
                let base = p.ease + p.knee_liner + self.dome(s, z);
                let w = |d: f64| 1.0 - smoothstep(-1.0, 5.0, d);
                let lift_f = w(o_eval(&self.o.front, s, z)) * (self.front_outer(s, z, lt) + 0.4 - base).max(0.0);
                let lift_s = w(self.side_panel(s, z)) * (self.side_inner(s, z, lt) + p.wall + 0.4 - base).max(0.0);
                let lift_t = w(o_eval(&self.o.thigh, s, z)) * (p.ease + p.thigh_foam + p.wall + 0.4 - base).max(0.0);
                let inner = base + lift_f.max(lift_s).max(lift_t);
                let vz = self.knee_z - 8.0 + 0.55 * s.abs();
                let chevron = p.chevron_h * bump((z - vz) / 3.5) * (1.0 - smoothstep(40.0, 55.0, s.abs()));
                Self::band(self.body.eval(q), inner, inner + p.wall + chevron)
            }
            Part::ThighGuard => Self::band(self.body.eval(q), p.ease + p.thigh_foam, p.ease + p.thigh_foam + p.wall),
            Part::ThighFoam => Self::band(self.body.eval(q), p.ease, p.ease + p.thigh_foam),
            Part::WingFoam(_) => Self::band(self.body.eval(q), p.ease, p.ease + p.wing_foam),
            Part::Wing(_) => Self::band(self.body.eval(q), p.ease + p.wing_foam, p.ease + p.wing_foam + p.wing_wall),
            Part::Strap(_) => Self::band(self.leg.eval(q), p.ease, p.ease + p.strap_t),
        };
        band.max(r)
    }

    pub fn bounds(&self, part: Part) -> Aabb {
        let m = 25.0;
        let p = &self.p;
        let o = &self.o;
        let zb = self.z_bot.0.iter().cloned().fold(f64::MAX, f64::min);
        let (z0, z1) = match part {
            Part::ShinShell => (zb - 2.0, o.front.z_max + 2.0),
            Part::SidePanel(_) => (zb - 2.0, o.side.z_max + 2.0),
            Part::ShinLiner => (zb - 2.0, o.front.z_max.max(o.side.z_max) + p.liner_border + 2.0),
            Part::KneeCap | Part::KneeLiner => (o.knee.z_min - 2.0, o.knee.z_max + 2.0),
            Part::ThighGuard | Part::ThighFoam => (o.thigh.z_min - 2.0, o.thigh.z_max + 2.0),
            Part::WingFoam(_) | Part::Wing(_) => (zb + 13.0, o.calf.z_max + 2.0),
            Part::Strap(i) => {
                let zf = self.z_front_bottom();
                let zs = zf + p.strap_at[i] * (self.knee_z - zf);
                (zs - 0.5 * p.strap_w - 2.0, zs + 0.5 * p.strap_w + 2.0)
            }
        };
        let (mut lo, mut hi) = (self.lo - Vector3::repeat(m), self.hi + Vector3::repeat(m));
        match part {
            Part::WingFoam(true) | Part::Wing(true) | Part::SidePanel(true) => lo.y = self.f.a.y - 10.0,
            Part::WingFoam(false) | Part::Wing(false) | Part::SidePanel(false) => hi.y = self.f.a.y + 10.0,
            _ => {}
        }
        // The calf wraps and straps are behind the side panels: nothing of them is more than
        // ~25 mm in front of the axis. (A thin skin at fine cells in the whole leg's box asked for
        // 8.9 GB on the stock pad, 2026-11-12.)
        if matches!(part, Part::WingFoam(_) | Part::Wing(_) | Part::Strap(_)) {
            hi.x = hi.x.min(self.f.axis_at(z1).x.max(self.f.axis_at(z0).x) + 25.0);
        }
        Aabb::new(Point3::new(lo.x, lo.y, z0), Point3::new(hi.x, hi.y, z1))
    }

    pub fn solid(&self, part: Part) -> Solid {
        let me = self.clone();
        Solid::user_fn(move |q| me.field(part, q), self.bounds(part))
    }
}

fn o_eval(o: &Outline, s: f64, z: f64) -> f64 {
    o.eval(s, z)
}

/// One pad's numbers.
pub struct Report {
    pub name: String,
    pub parts: Vec<(Part, IndexedMesh, f64)>,
    pub grams: f64,
    /// Standoff of the shell's outside from the leg at mid-shin, (theta, mm).
    pub standoff: Vec<(f64, f64)>,
    pub standoff_max: (f64, f64, f64),
    pub hard_cm2: f64,
    pub soft_cm2: f64,
    pub exposed_cm2: f64,
    pub into_boot: (f64, f64),
}

/// The pad's outside at (theta, z): march out from the axis (0.2 mm steps).
fn standoff(pad: &Pad, th: f64, z: f64) -> Option<f64> {
    let parts = [Part::ShinShell, Part::SidePanel(true), Part::SidePanel(false)];
    let c = pad.f.axis_at(z);
    let d = pad.f.dir(th);
    let (mut r_leg, mut r_out) = (None, None);
    let mut r = 0.0;
    while r < 160.0 {
        let q = c + d * r;
        if pad.leg.eval(q) < 0.0 {
            r_leg = Some(r);
        }
        if parts.iter().any(|&pt| pad.field(pt, q) < 0.0) {
            r_out = Some(r);
        }
        r += 0.2;
    }
    Some(r_out? - r_leg?)
}

pub fn measure(pad: &Pad, parts: Vec<(Part, IndexedMesh, f64)>, leg_mesh: &IndexedMesh, boot_top: &Ring, shell: &NearMesh, tongue: &Field) -> Report {
    let grams = parts.iter().map(|(_, _, g)| g).sum();
    let z_mid = 0.5 * (pad.z_front_bottom() + pad.knee_z);
    let standoff_v: Vec<(f64, f64)> = [0.0, 45.0, -45.0, 90.0, -90.0]
        .iter()
        .filter_map(|&th| standoff(pad, th, z_mid).map(|s| (th, s)))
        .collect();
    let mut smax_ = (0.0, 0.0, 0.0);
    let mut z = pad.z_front_bottom() + 20.0;
    while z < pad.shell_top() - 20.0 {
        let mut th = -90.0;
        while th <= 90.0 {
            if let Some(s) = standoff(pad, th, z) {
                if s > smax_.0 {
                    smax_ = (s, th, z);
                }
            }
            th += 10.0;
        }
        z += 20.0;
    }
    // Skin covered: leg triangles whose centres fall in the hard parts' or the wings' regions,
    // and skin left bare between the boot's top and the pad (front and sides, |theta| <= 100;
    // the tongue covers the front up to its top).
    let (mut hard, mut soft, mut exposed) = (0.0, 0.0, 0.0);
    let tongue_top = 186.0;
    for f in &leg_mesh.faces {
        let [a, b, c] = f.map(|i| leg_mesh.vertices[i as usize]);
        let area = 0.5 * (b - a).cross(&(c - a)).norm();
        let m = Point3::from((a.coords + b.coords + c.coords) / 3.0);
        if m.z < 100.0 || m.z > pad.knee_z + pad.p.knee_above + 10.0 {
            continue;
        }
        let th = pad.f.theta(m);
        let inside = |part: Part| pad.region(part, m) < 0.0;
        if Part::all().into_iter().any(|pt| pt.hard() && inside(pt)) {
            hard += area;
        } else if inside(Part::Wing(true)) || inside(Part::Wing(false)) {
            soft += area;
        } else if th.abs() <= 100.0 && m.z > boot_top.at(th) && m.z < pad.z_bot.at(th) + 1.0 && !(th.abs() < 45.0 && m.z < tongue_top) {
            exposed += area;
        }
    }
    // How far the pad goes into the boot's shell or its tongue (fitting badly with the skate).
    let mut into = (0.0f64, 0.0f64);
    for (part, mesh, _) in &parts {
        if matches!(part, Part::Strap(_)) {
            continue;
        }
        for v in mesh.vertices.iter().filter(|v| v.z < 200.0) {
            into.0 = into.0.max(-shell.eval(*v));
            into.1 = into.1.max(-tongue.eval(*v));
        }
    }
    Report {
        name: pad.name.clone(),
        parts,
        grams,
        standoff: standoff_v,
        standoff_max: smax_,
        hard_cm2: hard / 100.0,
        soft_cm2: soft / 100.0,
        exposed_cm2: exposed / 100.0,
        into_boot: (into.0.max(0.0), into.1.max(0.0)),
    }
}

/// Mesh and weigh every part (threads).
/// Mesh and weigh every part, `PAD_THREADS` (default 4) at a time: each uniform mesh holds its
/// whole grid while it runs, and all 16 parts at once used too much memory (2026-11-12).
pub fn build_parts(pad: &Pad) -> Vec<(Part, IndexedMesh, f64)> {
    let n = std::env::var("PAD_THREADS").ok().and_then(|v| v.parse::<usize>().ok()).unwrap_or(4).max(1);
    let all = Part::all();
    let mut out = Vec::with_capacity(all.len());
    for chunk in all.chunks(n) {
        let done: Vec<(Part, IndexedMesh, f64)> = std::thread::scope(|sc| {
            let hs: Vec<_> = chunk
                .iter()
                .map(|&part| {
                    let pad = pad.clone();
                    sc.spawn(move || {
                        let m = mesh_of(&pad.solid(part), part.cell());
                        let g = mesh_volume(&m).abs() / 1000.0 * part.material().density / 1000.0;
                        (part, m, g)
                    })
                })
                .collect();
            hs.into_iter().map(|h| h.join().unwrap()).collect()
        });
        out.extend(done);
    }
    out
}

/// Draw to `path`; if that file can't be written (Windows holds a picture open in a viewer:
/// "a file with a user-mapped section open", 2026-11-12), draw to `<name>-new.png` instead and
/// say so, rather than stopping the run.
pub(crate) fn draw(layers: &[Layer], views: &[View], px_per_mm: f64, path: &Path, style: Style) -> Result<(), String> {
    match render_styled(layers, views, px_per_mm, path, style) {
        Ok(_) => Ok(()),
        Err(err) => {
            let alt = path.with_file_name(format!("{}-new.png", path.file_stem().and_then(|s| s.to_str()).unwrap_or("render")));
            println!("  {} couldn't be written ({err}; open in a viewer?): drawing {} instead", path.display(), alt.display());
            render_styled(layers, views, px_per_mm, &alt, style).map(|_| ()).map_err(|e| format!("{}: {e}", alt.display()))
        }
    }
}

fn lean_mesh(m: &IndexedMesh, zp: f64, deg: f64) -> IndexedMesh {
    let mut out = m.clone();
    for v in &mut out.vertices {
        v.x += lean_dx_at(zp, deg, v.z);
    }
    out
}

pub(crate) fn close_between(m: &IndexedMesh, z0: f64, z1: f64) -> IndexedMesh {
    let mut c = clip_mesh_against_plane(m, Point3::new(0.0, 0.0, z1), -Vector3::z());
    c = clip_mesh_against_plane(&c, Point3::new(0.0, 0.0, z0), Vector3::z());
    weld_vertices(&mut c, 1e-6);
    auto_cap_open_boundaries(&mut c);
    c
}

/// The leg scaled round the frame's axis by `f(z)`, for a pad that has to clear a bigger leg.
fn scaled_leg(m: &IndexedMesh, fr: &Frame, f: &dyn Fn(f64) -> f64) -> Vec<Point3<f64>> {
    m.vertices
        .iter()
        .map(|v| {
            let c = fr.axis_at(v.z);
            let r = *v - c;
            let k = f(v.z);
            Point3::new(c.x + r.x * k, c.y + r.y * k, v.z)
        })
        .collect()
}

/// A stock-style pad: its inside the convex hull of `pts` (the leg it must clear) between the
/// pad's heights, moved back along the leg's forward direction until it meets the shin (straps
/// pull it on), a flat bottom edge at `z_bottom`.
#[allow(clippy::too_many_arguments)]
fn stock_pad(
    name: &str,
    pts: &[Point3<f64>],
    leg_mesh: &IndexedMesh,
    base: &Pad,
    z_bottom: f64,
) -> Result<Pad, String> {
    let (z0, z1) = (z_bottom - 15.0, base.knee_z + base.p.knee_above + 20.0);
    let sel: Vec<Point3<f64>> = pts.iter().filter(|v| v.z >= z0 && v.z <= z1).copied().collect();
    let hull = cortenforge::cf_geometry::convex_hull(&sel, None).ok_or("stock hull: degenerate")?;
    let mut mesh = IndexedMesh::default();
    mesh.vertices = hull.vertices.clone();
    mesh.faces = hull.faces.clone();
    let mut field = Field::new(&mesh, 25.0, 1.5, None, &format!("{name} hull"))?;
    // Pull it back onto the shin: the least gap between the front of the leg and the hull.
    let mut gap = f64::MAX;
    for v in leg_mesh.vertices.iter().step_by(3) {
        if v.z < z_bottom + 20.0 || v.z > base.knee_z - 40.0 || base.f.theta(*v).abs() > 30.0 {
            continue;
        }
        gap = gap.min(-field.eval(*v));
    }
    field.shift = base.f.fwd * gap;
    println!("  {name}: hull {} faces, moved back {gap:.1} mm onto the shin", mesh.faces.len());
    Ok(Pad { name: name.into(), body: Arc::new(Body::Hull(field)), z_bot: Ring::flat(z_bottom), ..base.clone() })
}

pub fn run(scan_path: &Path, out: &Path) -> Result<(), String> {
    let t0 = Instant::now();
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let docs = Path::new("docs/pad");
    std::fs::create_dir_all(docs).map_err(|e| e.to_string())?;
    let boot_dir = std::env::var_os("PAD_BOOT_DIR").map(PathBuf::from).unwrap_or_else(|| PathBuf::from("out/boot"));
    let load = |n: &str| load_mesh(boot_dir.join(n)).map_err(|e| format!("{}: {e} (run `boot` first)", boot_dir.join(n).display()));
    let (shell_m, plate, laces_m, eyelets, guard) =
        (load("shell.stl")?, load("tongue_plate.stl")?, load("laces.stl")?, load("eyelets.stl")?, load("tendon_guard.stl")?);

    // The leg, leaned like the boot.
    let lg = leg::build(scan_path, 600.0)?;
    let lm = landmarks(&lg.foot, lg.scan_top());
    let zp = 0.5 * (lm.medial.z + lm.lateral.z);
    let lean = HockeyParams::default().lean_deg;
    let leaned = lean_mesh(&lg.mesh, zp, lean);
    let lean_pt = |p: Point3<f64>| Point3::new(p.x + lean_dx_at(zp, lean, p.z), p.y, p.z);
    let knee = lean_pt(lg.knee);
    // The shin's front line: the most forward point of each section between 170 and 430 mm.
    let crest: Vec<Point3<f64>> = (0..14)
        .filter_map(|k| {
            let z = 170.0 + 20.0 * k as f64;
            let cs = crate::scan::section(&leaned, Point3::new(0.0, 0.0, z), Vector3::z());
            cs.points.iter().copied().max_by(|a, b| a.x.total_cmp(&b.x))
        })
        .collect();
    let y_crest = crest.iter().map(|p| p.y).sum::<f64>() / crest.len().max(1) as f64;
    let a150 = lean_pt(Point3::new(lg.ankle.x, y_crest, 150.0));
    let f = Frame::new(a150, Point3::new(knee.x, y_crest, knee.z));
    println!(
        "leg leaned {lean} deg like the boot: knee joint ({:.0}, {:.0}, {:.0}); shin's front line at y {y_crest:.1}; axis tilt {:.1} deg forward",
        knee.x,
        knee.y,
        knee.z,
        f.u.x.atan2(f.u.z).to_degrees()
    );

    // Fields: the leg (exact near the skin), the tongue (a grid: its lattice cavity counts as
    // inside), the laces (exact, near their top only), the boot's shell (exact, for checks).
    let leg_closed = close_between(&leaned, 80.0, 600.0);
    let leg_f = Field::new(&leg_closed, 30.0, 2.0, Some(6.0), "leg")?;
    let tongue_f = Field::new(&plate, 20.0, 1.25, None, "tongue plate")?;
    let laces_f = NearMesh::new(&laces_m, "laces")?;
    let shell_f = NearMesh::new(&shell_m, "boot shell")?;
    let top = boot_top(&shell_m, &f);
    println!(
        "boot's top edge round the leg: front {:.0} (throat bridged), medial side {:.0}, lateral side {:.0}, back {:.0} mm",
        top.at(0.0),
        top.at(90.0),
        top.at(-90.0),
        top.at(180.0)
    );

    let p = PadParams::default();
    let in_pad = leaned.vertices.iter().filter(|v| v.z > 110.0 && v.z < knee.z + p.knee_above + 20.0);
    let (lo, hi) = in_pad.fold((Vector3::repeat(f64::MAX), Vector3::repeat(f64::MIN)), |(lo, hi), v| (lo.inf(&v.coords), hi.sup(&v.coords)));
    let z_bot = Ring(top.0.iter().map(|z| z + p.clear_boot).collect());
    let under = Pad {
        name: "fitted, tongue under".into(),
        body: Arc::new(Body::Fitted { leg: leg_f.clone(), tongue: Some(tongue_f.clone()), laces: Some(laces_f.clone()) }),
        leg: leg_f.clone(),
        f,
        p,
        z_bot: z_bot.clone(),
        knee_z: knee.z,
        lo,
        hi,
        flush: false,
        o: Arc::new(Outlines::new(knee.z)),
    };
    // Tongue under, flush: the same, but the shell's offset from the leg alone, so the outside
    // stays slim over the tongue; where the tongue reaches past the shell's inside, the run
    // says how far (how much the tongue's top would have to change).
    let flush = Pad { name: "fitted, tongue under, flush".into(), flush: true, ..under.clone() };
    // Tongue over: the leg alone; in front (within the tongue's +-50 degrees) the bottom reaches
    // `over_overlap` below the tongue's top, behind it.
    let tongue_top = plate.vertices.iter().map(|v| v.z).fold(f64::MIN, f64::max);
    let z_over = Ring(
        (0..72)
            .map(|b| {
                let th = -180.0 + 5.0 * b as f64;
                let w = 1.0 - smoothstep(35.0, 55.0, th.abs());
                (1.0 - w) * z_bot.0[b] + w * (tongue_top - p.over_overlap)
            })
            .collect(),
    );
    let over = Pad {
        name: "fitted, tongue over".into(),
        body: Arc::new(Body::Fitted { leg: leg_f.clone(), tongue: None, laces: None }),
        z_bot: z_over,
        ..under.clone()
    };

    // The comparison pads, sized by the retail rule: the knee's centre (mid-patella, ANSUR: 20 mm
    // above the tibial height) to the boot's top at the front.
    let mid_patella = knee.z + (ansur::KNEE_HEIGHT_MID_PATELLA - ansur::TIBIAL_HEIGHT);
    let boot_front_top = top.at(0.0);
    let length_in = (mid_patella - boot_front_top) / 25.4;
    let stock_in = length_in.round();
    println!("retail sizing: knee centre {mid_patella:.0} mm up, boot top {boot_front_top:.0}: {length_in:.1}\" -> a {stock_in:.0}\" pad, undersized {:.0}\"", stock_in - 1.0);
    // ANSUR II, men 170-180 cm: 95th-percentile girths over the medians (RESEARCH.md 18).
    let p95 = |z: f64| {
        let (ankle, calf, thigh) = (252.0 / 228.0, 440.0 / 391.0, 460.0 / 408.0);
        if z <= 130.0 {
            ankle
        } else if z <= 360.0 {
            ankle + (calf - ankle) * (z - 130.0) / 230.0
        } else if z <= 500.0 {
            calf + (thigh - calf) * (z - 360.0) / 140.0
        } else {
            thigh
        }
    };
    // `PAD_QUICK=1`: the fitted pad alone (no comparison pads, cuts or tongue-over), for
    // iterating on its look (~10 min).
    let quick = std::env::var_os("PAD_QUICK").is_some();
    let cmp = if quick {
        None
    } else {
        let big = scaled_leg(&leaned, &f, &p95);
        let stock = stock_pad(&format!("stock {stock_in:.0}\""), &big, &leaned, &under, mid_patella - stock_in * 25.4)?;
        let small = stock_pad(&format!("undersized {:.0}\"", stock_in - 1.0), &leaned.vertices, &leaned, &under, mid_patella - (stock_in - 1.0) * 25.4)?;
        Some((stock, small))
    };

    // Each pad in turn: build, measure, draw, save, then let its meshes go (one pad's parts in
    // memory at a time).
    let pretty = Style::pretty();
    let skin = [0.85, 0.70, 0.60];
    let leg_up = clip_mesh_against_plane(&leaned, Point3::new(0.0, 0.0, 100.0), Vector3::z());
    let leg_up = clip_mesh_against_plane(&leg_up, Point3::new(0.0, 0.0, 595.0), -Vector3::z());
    let boot_layers = [
        Layer::solid(&shell_m, [0.12, 0.12, 0.13]).with_spec(0.35),
        Layer::solid(&plate, [0.90, 0.90, 0.87]),
        Layer::solid(&laces_m, [0.96, 0.96, 0.94]),
        Layer::solid(&eyelets, [0.80, 0.81, 0.84]).with_spec(0.5),
        Layer::solid(&guard, [0.10, 0.10, 0.11]),
    ];
    let views = [
        View::looking(Vector3::new(-0.70, 0.55, -0.25), Vector3::z()),
        View::looking(Vector3::new(-0.70, -0.55, -0.25), Vector3::z()),
        View::looking(Vector3::new(0.0, -1.0, 0.0), Vector3::z()),
    ];
    let mut reports = Vec::new();
    let mut runs: Vec<(&Pad, &str)> = vec![(&flush, "pad_flush.png")];
    if let Some((stock, small)) = &cmp {
        runs.extend([(&over, "pad_over.png"), (stock, "pad_stock.png"), (small, "pad_undersized.png")]);
    }
    for (pad, file) in runs {
        let t = Instant::now();
        let parts = build_parts(pad);
        let mut r = measure(pad, parts, &leaned, &top, &shell_f, &tongue_f);
        println!("{}: {:.0} g, {:.1} s", r.name, r.grams, t.elapsed().as_secs_f64());
        if std::ptr::eq(pad, &flush) {
            // Views to mark up (Jon draws on screenshots): front, outside, back, inside, with a
            // faint grid on the leg and the pad: lines every 50 mm up (darker every 100) and
            // every 30 degrees round (the front line red, the sides at +-90 blue).
            let grid = |m: &IndexedMesh, c: [f64; 3]| -> Vec<[f64; 3]> {
                m.vertices
                    .iter()
                    .map(|v| {
                        let th = f.theta(*v);
                        let zr = v.z.rem_euclid(50.0).min(50.0 - v.z.rem_euclid(50.0));
                        let tr = th.rem_euclid(30.0).min(30.0 - th.rem_euclid(30.0)).to_radians() * R_REF;
                        let hundred = v.z.rem_euclid(100.0).min(100.0 - v.z.rem_euclid(100.0)) < 0.8;
                        if tr < 0.7 && th.abs() < 1.0 {
                            [0.80, 0.15, 0.12]
                        } else if tr < 0.7 && (th.abs() - 90.0).abs() < 1.0 {
                            [0.15, 0.30, 0.80]
                        } else if zr < 0.6 || tr < 0.6 {
                            let k = if hundred { 0.45 } else { 0.70 };
                            [c[0] * k, c[1] * k, c[2] * k]
                        } else {
                            c
                        }
                    })
                    .collect()
            };
            let leg_c = grid(&leg_up, skin);
            let part_c: Vec<Vec<[f64; 3]>> = r.parts.iter().map(|(pt, m, _)| grid(m, pt.color())).collect();
            let mut ls = vec![Layer { mesh: &leg_up, color: skin, vertex_colors: Some(&leg_c), spec: 0.0 }];
            ls.extend(boot_layers.iter().copied());
            for ((pt, m, _), c) in r.parts.iter().zip(&part_c) {
                if !matches!(pt, Part::WingFoam(_) | Part::KneeLiner | Part::ThighFoam) {
                    ls.push(Layer { mesh: m, color: pt.color(), vertex_colors: Some(c), spec: if pt.hard() { 0.25 } else { 0.0 } });
                }
            }
            let mv = [
                View::looking(-Vector3::x(), Vector3::z()),
                View::looking(Vector3::y(), Vector3::z()),
                View::looking(Vector3::x(), Vector3::z()),
                View::looking(-Vector3::y(), Vector3::z()),
            ];
            draw(&ls, &mv, 1.2, &docs.join("pad_views.png"), pretty)?;
        }
        let mut ls = vec![Layer::solid(&leg_up, skin)];
        ls.extend(boot_layers.iter().copied());
        for (part, m, _) in &r.parts {
            // The shin liner shows as a border round the shells; the other foams stay hidden.
            if !matches!(part, Part::WingFoam(_) | Part::KneeLiner | Part::ThighFoam) {
                ls.push(Layer::solid(m, part.color()).with_spec(if part.hard() { 0.3 } else { 0.0 }));
            }
        }
        draw(&ls, &views, 1.2, &docs.join(file), pretty)?;
        let tag = r.name.replace([' ', ',', '"'], "_").replace("__", "_");
        for (part, m, _) in &r.parts {
            let n = format!("{tag}-{}.stl", part.name().replace([' ', ','], "_").replace("__", "_"));
            save_mesh(m, out.join(n)).map_err(|e| e.to_string())?;
        }
        r.parts = r.parts.into_iter().map(|(p, _, g)| (p, IndexedMesh::default(), g)).collect();
        reports.push(r);
    }

    // The summary.
    let mut lines = vec![
        format!("shin pads, round 3 (track D2: curved outlines, one knee shield), on {} joined to the stand-in leg, leaned {lean} deg like the boot", scan_path.display()),
        format!(
            "layers (placeholders until D3): sock {} mm; liner {} mm at the front to {} at {} deg round, showing {} mm past the shells; hard parts {} mm PA 12: a tapered front shell with a {} mm ridge down the shin and {} mm grooves across, side panels following its edge and stepped over it ({} mm under), {} vent slots each; one knee shield (ears over the sides of the knee, a V bottom over the front shell) over {} mm of liner, domed {} mm more over the kneecap, a {} mm V ridge across it; thigh guard under its top on {} mm foam; calf wraps {} mm foam + {} mm TPU skin; straps {} x {} mm",
            p.ease,
            p.liner_front,
            p.liner_side,
            p.side_to,
            p.liner_border,
            p.wall,
            p.crest_h,
            p.groove_d,
            p.side_overlap,
            p.vents,
            p.knee_liner,
            p.dome_h,
            p.chevron_h,
            p.thigh_foam,
            p.wing_foam,
            p.wing_wall,
            p.strap_w,
            p.strap_t
        ),
        format!(
            "boot's top edge: front {:.0} (throat bridged; the tongue reaches {tongue_top:.0}), sides {:.0} / {:.0}, back {:.0} mm; fitted pad's bottom {} mm above it all round",
            top.at(0.0),
            top.at(90.0),
            top.at(-90.0),
            top.at(180.0),
            p.clear_boot
        ),
        format!("retail sizing: knee centre {mid_patella:.0} mm up to the boot's top {boot_front_top:.0}: {length_in:.1}\", so a {stock_in:.0}\" pad; undersized one size down"),
        String::new(),
    ];
    for r in &reports {
        lines.push(format!("{}: {:.0} g", r.name, r.grams));
        for (part, _, g) in &r.parts {
            lines.push(format!("  {:<22} {:>6.1} g  ({})", part.name(), g, part.material().name));
        }
        lines.push(format!(
            "  bulk at mid-shin (shell's outside to skin): {}; most {:.1} mm at {:+.0} deg, z {:.0}",
            r.standoff.iter().map(|(t, s)| format!("{t:+.0} deg {s:.1} mm")).collect::<Vec<_>>().join(", "),
            r.standoff_max.0,
            r.standoff_max.1,
            r.standoff_max.2
        ));
        lines.push(format!(
            "  skin covered: hard {:.0} cm2, soft (wings) {:.0} cm2; bare between the boot and the pad {:.0} cm2",
            r.hard_cm2, r.soft_cm2, r.exposed_cm2
        ));
        lines.push(format!("  into the boot: shell {:.1} mm, tongue {:.1} mm", r.into_boot.0, r.into_boot.1));
    }
    for l in &lines {
        println!("{l}");
    }
    std::fs::write(docs.join("summary.txt"), lines.join("\n") + "\n").map_err(|e| e.to_string())?;
    let Some((stock, small)) = &cmp else {
        println!("quick run (PAD_QUICK): the fitted pad only; drew docs/pad/pad_flush.png and pad_views.png ({:.0} s in all)", t0.elapsed().as_secs_f64());
        return Ok(());
    };

    // Sections across the shin at mid-height, the three pads side by side (fitted, stock,
    // undersized), looking down: the bulk.
    let z_cut = 0.5 * (flush.z_front_bottom() + flush.knee_z);
    let below = Solid::plane(Vector3::z(), z_cut);
    let mut sec_meshes: Vec<(IndexedMesh, [f64; 3])> = Vec::new();
    for (k, pad) in [&flush, stock, small].iter().enumerate() {
        let dy = -170.0 * k as f64;
        let mut parts: Vec<(Solid, Aabb, [f64; 3])> = Vec::new();
        let lf = pad.leg.clone();
        let mut lb = pad.bounds(Part::ShinShell);
        lb.min.z = z_cut - 25.0;
        lb.max.z = z_cut + 1.0;
        parts.push((Solid::user_fn(move |q| lf.eval(q), lb).intersect(below.clone()), lb, skin));
        for part in [Part::ShinShell, Part::SidePanel(true), Part::SidePanel(false), Part::ShinLiner, Part::WingFoam(true), Part::WingFoam(false), Part::Wing(true), Part::Wing(false), Part::Strap(0), Part::Strap(1)] {
            let mut b = pad.bounds(part);
            if b.min.z > z_cut || b.max.z < z_cut - 25.0 {
                continue;
            }
            b.min.z = z_cut - 25.0;
            b.max.z = b.max.z.min(z_cut + 1.0);
            let pd = (*pad).clone();
            parts.push((Solid::user_fn(move |q| pd.field(part, q), b).intersect(below.clone()), b, part.color()));
        }
        let ms: Vec<(IndexedMesh, [f64; 3])> = std::thread::scope(|sc| {
            let hs: Vec<_> = parts.into_iter().map(|(s, _, c)| sc.spawn(move || (mesh_of(&s, 0.5), c))).collect();
            hs.into_iter().map(|h| h.join().unwrap()).collect()
        });
        for (mut m, c) in ms {
            for v in &mut m.vertices {
                v.y += dy;
            }
            sec_meshes.push((m, c));
        }
    }
    let sl: Vec<Layer> = sec_meshes.iter().filter(|(m, _)| !m.faces.is_empty()).map(|(m, c)| Layer::solid(m, *c)).collect();
    draw(&sl, &[View::looking(-Vector3::z(), Vector3::x())], 3.0, &docs.join("pad_sections.png"), pretty)?;

    // Lengthwise, through the shin's front line, seen from the outside: how the pad's bottom
    // meets the tongue: tongue under, under flush, and over, left to right (200 mm apart).
    let keep = Solid::plane(-Vector3::y(), -y_crest); // the medial half, y >= y_crest
    let mut len_meshes: Vec<(IndexedMesh, [f64; 3])> = Vec::new();
    for (k, pad) in [&under, &flush, &over].iter().enumerate() {
        let dx = 200.0 * k as f64;
        let mut items: Vec<(Solid, [f64; 3])> = Vec::new();
        let lf = pad.leg.clone();
        let mut lb = pad.bounds(Part::ShinShell);
        lb.min.z = 90.0;
        lb.max.z = pad.knee_z + pad.p.knee_above + 10.0;
        lb.min.y = y_crest - 1.0;
        items.push((Solid::user_fn(move |q| lf.eval(q), lb).intersect(keep.clone()), skin));
        let tf = tongue_f.clone();
        let tb = Aabb::new(Point3::new(60.0, y_crest - 1.0, 90.0), Point3::new(150.0, 60.0, 200.0));
        items.push((Solid::user_fn(move |q| tf.eval(q), tb).intersect(keep.clone()), [0.90, 0.90, 0.87]));
        let sf = shell_f.clone();
        let sb = Aabb::new(Point3::new(-20.0, y_crest - 1.0, 90.0), Point3::new(150.0, 70.0, 146.0));
        items.push((Solid::user_fn(move |q| sf.eval(q), sb).intersect(keep.clone()), [0.12, 0.12, 0.13]));
        for part in [Part::ShinShell, Part::ShinLiner, Part::KneeCap, Part::KneeLiner, Part::ThighGuard, Part::ThighFoam] {
            let mut b = pad.bounds(part);
            b.min.y = b.min.y.max(y_crest - 1.0);
            let pd = (*pad).clone();
            items.push((Solid::user_fn(move |q| pd.field(part, q), b).intersect(keep.clone()), part.color()));
        }
        let ms: Vec<(IndexedMesh, [f64; 3])> = std::thread::scope(|sc| {
            let hs: Vec<_> = items.into_iter().map(|(s, c)| sc.spawn(move || (mesh_of(&s, 0.6), c))).collect();
            hs.into_iter().map(|h| h.join().unwrap()).collect()
        });
        for (mut m, c) in ms {
            for v in &mut m.vertices {
                v.x += dx;
            }
            len_meshes.push((m, c));
        }
    }
    let ll: Vec<Layer> = len_meshes.iter().filter(|(m, _)| !m.faces.is_empty()).map(|(m, c)| Layer::solid(m, *c)).collect();
    draw(&ll, &[View::looking(Vector3::y(), Vector3::z())], 1.6, &docs.join("pad_lengthwise.png"), pretty)?;
    println!("drew docs/pad/*.png; wrote STLs to {} ({:.0} s in all)", out.display(), t0.elapsed().as_secs_f64());
    Ok(())
}
