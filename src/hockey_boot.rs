//! The hockey boot (track B, the boot's round 2): a player's skate boot grown from the last.
//!
//! Jon (2026-11-11): "more like a traditional hockey skate, with the top cuff around the ankle
//! flared a bit for ankle mobility", the throat running "from the toes to the heel, direction
//! wise", a tendon guard, the tongue, eyelets, layers. Built on round 1's last base
//! (`foot::last_base`), with the leg carried up above the scan as a prism of its top section.
//! Every line is placed from landmarks found on the scan, so it follows the foot it is built on:
//! the ankle bones (malleoli), and the dorsal line, the ridge of the foot from the toes up the
//! instep and the front of the shin.
//!
//! The parts are signed-distance functions over one shared geometry (`BootGeom`), in layers
//! measured out from the last's base surface (`d`, mm; negative inside):
//!
//! - `ease` (fit allowance) .. `inner_off`: the liner; thicker where the cuff flares (the collar)
//!   and round the ankle bones (the ankle pads, in pockets the shell bulges out for). Under the
//!   foot it is the footbed. Under the throat it is the tongue's padding instead.
//! - `inner_off` .. `outer_off`: the shell, `wall` thick, plus an eyelet facing (a thicker strip
//!   along each side of the throat that carries the eyelets). Trimmed at the cuff line, opened
//!   over the dorsal line for the throat, holed for the eyelets.
//! - Outside the shell: the toe cap and the tendon guard (both overlap the shell, riveted or
//!   bonded in a real boot), and the outsole under it, which the chassis or holder mounts to.
//! - The tongue: padding under the throat and its edges, a raised core filling the throat, and
//!   above the cuff its full thickness.
//! - Eyelets (grommets through the facing) and the laces, criss-crossed over the tongue.
//!
//! Sizes not measured yet are hand estimates, marked in `HockeyParams`; `docs/MEASURE.md` lists
//! what to measure on Jon's TF9 to replace them.

use std::fmt::Write as _;
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

use cortenforge::cf_design::mechanism::mass::mass_properties;
use cortenforge::cf_design::{Aabb, Solid};
use cortenforge::mesh::io::save_mesh;
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::foot::{self, Foot};
use crate::materials::{AL_6061_T6, CFRP, LACE, LINER_FOAM, Material, PA12_MJF, TPU};
use crate::render::{Layer, Style, View, render_styled, standard_views};
use crate::round1::mesh_of;
use crate::scan;

/// The boot's sizes, mm. "Estimate" marks a hand estimate until measured (MEASURE.md, the TF9).
#[derive(Debug, Clone, Copy)]
pub struct HockeyParams {
    /// Fit allowance between the last's base and the liner (round 1).
    pub ease: f64,
    /// Liner foam thickness (round 1).
    pub liner: f64,
    /// Shell wall (round 1).
    pub wall: f64,
    /// Outsole under the shell.
    pub sole: f64,
    /// Cuff top above the higher ankle bone, at the leg's centre line. Estimate.
    pub cuff_above_ankle: f64,
    /// The cuff's top edge rises toward the back by this angle. Estimate.
    pub trim_tilt_deg: f64,
    /// How far the cuff's top edge stands off the leg beyond the normal layers, at the sides
    /// (30% of it at the front and back). Estimate.
    pub flare: f64,
    /// The flare starts this far above the higher ankle bone and grows (quadratically) to the top.
    pub flare_from: f64,
    /// Ankle pockets: the shell bulges out (and the liner thickens) round each ankle bone by up
    /// to this much, Gaussian with `ankle_sigma`. Estimate.
    pub ankle_pocket: f64,
    pub ankle_sigma: f64,
    /// Ankle pads: the liner within this radius of an ankle bone.
    pub ankle_pad_r: f64,
    /// The throat starts here, as a fraction of the foot's length (round 1's throat ended at 0.70).
    pub throat_front: f64,
    /// Throat gap between the quarters' edges: at the front, and at the cuff top. Estimate.
    pub throat_w0: f64,
    pub throat_w1: f64,
    /// How far from the dorsal line (in the side view) the throat and tongue reach.
    pub reach: f64,
    /// Eyelet facing: width beyond the throat's edge, and the extra thickness. Estimate.
    pub facing_w: f64,
    pub facing_t: f64,
    /// Eyelets: spacing along the throat, and centre distance from the throat's edge. Estimate.
    pub eyelet_spacing: f64,
    pub eyelet_inset: f64,
    /// Tongue: overlap under each quarter, height above the cuff, how far it stands proud of the
    /// shell in the throat. Estimate.
    pub tongue_overlap: f64,
    pub tongue_above_cuff: f64,
    pub tongue_proud: f64,
    /// Tendon guard: height above the cuff at the back, thickness, how far its top kicks back
    /// off the leg. Estimate.
    pub guard_above_cuff: f64,
    pub guard_t: f64,
    pub guard_kick: f64,
    /// Toe cap thickness over the shell.
    pub toe_cap_t: f64,
    /// Lace radius (drawn round).
    pub lace_r: f64,
}

impl Default for HockeyParams {
    fn default() -> Self {
        Self {
            ease: 1.0,
            liner: 4.0,
            wall: 2.5,
            sole: 3.0,
            cuff_above_ankle: 75.0,
            trim_tilt_deg: 8.0,
            flare: 8.0,
            flare_from: 15.0,
            ankle_pocket: 3.0,
            ankle_sigma: 12.0,
            ankle_pad_r: 24.0,
            throat_front: 0.70,
            throat_w0: 26.0,
            throat_w1: 32.0,
            reach: 32.0,
            facing_w: 18.0,
            facing_t: 1.5,
            eyelet_spacing: 19.0,
            eyelet_inset: 9.0,
            tongue_overlap: 10.0,
            tongue_above_cuff: 40.0,
            tongue_proud: 1.5,
            guard_above_cuff: 55.0,
            guard_t: 2.5,
            guard_kick: 12.0,
            toe_cap_t: 1.5,
            lace_r: 1.6,
        }
    }
}

/// Landmarks found on the scan (standard frame: heel at x = 0, toes +x, floor z = 0).
#[derive(Debug, Clone, Copy)]
pub struct Landmarks {
    /// The ankle bones' outermost points on the skin: medial (inside) and lateral.
    pub medial: Point3<f64>,
    pub lateral: Point3<f64>,
    /// +1 if the medial side is +y (a right foot in the standard frame), -1 if -y.
    pub medial_side: f64,
    /// The back of the heel at the lower ankle bone's height.
    pub heel_back_x: f64,
    /// The leg's centre (x, y) just below the top of the scan, and the scan's top.
    pub leg_c: (f64, f64),
    pub leg_top: f64,
    pub length: f64,
}

/// The ankle bones are the outermost skin points each side in the ankle region (10-33% of the
/// length, 50 mm up to just below the scan's top); the medial one sits higher and further
/// forward than the lateral one (on stand-in 022: 4 mm higher, 18 mm forward).
pub fn landmarks(foot: &Foot) -> Landmarks {
    let (l, h) = (foot.length, foot.height);
    let v = &foot.mesh.vertices;
    let in_ankle = |q: &&Point3<f64>| q.x > 0.10 * l && q.x < 0.33 * l && q.z > 50.0 && q.z < h - 12.0;
    let plus = v.iter().filter(in_ankle).max_by(|a, b| a.y.total_cmp(&b.y)).copied().unwrap_or(Point3::origin());
    let minus = v.iter().filter(in_ankle).min_by(|a, b| a.y.total_cmp(&b.y)).copied().unwrap_or(Point3::origin());
    let (medial, lateral, medial_side) =
        if plus.z + plus.x >= minus.z + minus.x { (plus, minus, 1.0) } else { (minus, plus, -1.0) };
    let zl = medial.z.min(lateral.z);
    let heel_back_x = v.iter().filter(|q| (q.z - zl).abs() < 3.0).map(|q| q.x).fold(f64::MAX, f64::min);
    let (mut x0, mut x1, mut y0, mut y1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for q in v.iter().filter(|q| q.z > h - 20.0 && q.z < h - 10.0) {
        x0 = x0.min(q.x);
        x1 = x1.max(q.x);
        y0 = y0.min(q.y);
        y1 = y1.max(q.y);
    }
    Landmarks { medial, lateral, medial_side, heel_back_x, leg_c: (0.5 * (x0 + x1), 0.5 * (y0 + y1)), leg_top: h, length: l }
}

/// The dorsal line: the ridge of the last from the toes, up the instep and the front of the shin,
/// as points (x, ridge y, z), with arc length measured in the side view (x-z).
pub struct Profile {
    pub pts: Vec<Point3<f64>>,
    pub s: Vec<f64>,
    /// Per segment: ax, az, dx, dz, 1/len^2.
    seg: Vec<[f64; 5]>,
}

/// The nearest point of the profile in the side view.
#[derive(Debug, Clone, Copy)]
pub struct Near {
    /// Arc length there.
    pub s: f64,
    /// Distance in the side view (ignores y).
    pub d: f64,
    /// The ridge's y there.
    pub y: f64,
}

impl Profile {
    pub fn new(pts: Vec<Point3<f64>>) -> Self {
        let mut s = vec![0.0];
        let mut seg = Vec::new();
        for w in pts.windows(2) {
            let (dx, dz) = (w[1].x - w[0].x, w[1].z - w[0].z);
            let l2 = (dx * dx + dz * dz).max(1e-12);
            s.push(s.last().unwrap() + l2.sqrt());
            seg.push([w[0].x, w[0].z, dx, dz, 1.0 / l2]);
        }
        Profile { pts, s, seg }
    }

    pub fn len(&self) -> f64 {
        *self.s.last().unwrap_or(&0.0)
    }

    pub fn closest(&self, x: f64, z: f64) -> Near {
        let mut best = (f64::MAX, 0usize, 0.0);
        for (i, g) in self.seg.iter().enumerate() {
            let (px, pz) = (x - g[0], z - g[1]);
            let t = ((px * g[2] + pz * g[3]) * g[4]).clamp(0.0, 1.0);
            let (ex, ez) = (px - t * g[2], pz - t * g[3]);
            let d2 = ex * ex + ez * ez;
            if d2 < best.0 {
                best = (d2, i, t);
            }
        }
        let (d2, i, t) = best;
        Near {
            s: self.s[i] + t * (self.s[i + 1] - self.s[i]),
            d: d2.sqrt(),
            y: self.pts[i].y + t * (self.pts[i + 1].y - self.pts[i].y),
        }
    }

    /// The point at arc length `s` and the unit tangent there (side view, toward the top).
    pub fn at(&self, s: f64) -> (Point3<f64>, Vector3<f64>) {
        let n = self.s.len();
        let i = self.s.partition_point(|&v| v <= s).saturating_sub(1).min(n - 2);
        let t = ((s - self.s[i]) / (self.s[i + 1] - self.s[i]).max(1e-12)).clamp(0.0, 1.0);
        let p = self.pts[i] + (self.pts[i + 1] - self.pts[i]) * t;
        let tan = Vector3::new(self.pts[i + 1].x - self.pts[i].x, 0.0, self.pts[i + 1].z - self.pts[i].z);
        (p, tan.normalize())
    }

    /// The outward normal at `s` in the side view (away from the foot: up over the instep,
    /// forward on the shin).
    pub fn normal(&self, s: f64) -> Vector3<f64> {
        let (_, t) = self.at(s);
        Vector3::new(t.z, 0.0, -t.x)
    }

    /// The first `s` from the toe end where `pred` holds (0.25 mm steps).
    pub fn find(&self, pred: impl Fn(&Point3<f64>) -> bool) -> Option<f64> {
        let mut s = 0.0;
        while s <= self.len() {
            if pred(&self.at(s).0) {
                return Some(s);
            }
            s += 0.25;
        }
        None
    }
}

/// Where a ray from `c` (side view) leaves the solid, in the plane at `y`: the outermost exit,
/// found by stepping in from far out. None if the ray misses it.
fn ray_exit(base: &dyn Fn(Point3<f64>) -> f64, c: (f64, f64), dir: (f64, f64), y: f64) -> Option<f64> {
    let at = |t: f64| base(Point3::new(c.0 + t * dir.0, y, c.1 + t * dir.1));
    let mut t_out = 420.0;
    if at(t_out) <= 0.0 {
        return None;
    }
    loop {
        let v = at(t_out);
        let t_in = t_out - (0.8 * v).max(0.5);
        if t_in <= 0.0 {
            return None;
        }
        if at(t_in) <= 0.0 {
            let (mut a, mut b) = (t_in, t_out);
            for _ in 0..30 {
                let m = 0.5 * (a + b);
                if at(m) <= 0.0 {
                    a = m;
                } else {
                    b = m;
                }
            }
            return Some(0.5 * (a + b));
        }
        t_out = t_in;
    }
}

/// The dorsal line, by rays fanned from `c` (behind the ankle, side view) from toward the toes up
/// to the shin. On each ray, the exit is taken in every plane y = const (2 mm apart), and the
/// ridge is where it reaches furthest: the centre of the y values within 1.5 mm of the furthest.
fn dorsal_profile(base: &dyn Fn(Point3<f64>) -> f64, c: (f64, f64), z_max: f64, y_lo: f64, y_hi: f64) -> Vec<Point3<f64>> {
    let mut raw = Vec::new();
    let mut th = -14.0f64;
    while th <= 75.0 {
        let dir = (th.to_radians().cos(), th.to_radians().sin());
        let mut hits = Vec::new();
        let mut y = y_lo;
        while y <= y_hi {
            if let Some(t) = ray_exit(base, c, dir, y) {
                hits.push((y, t));
            }
            y += 2.0;
        }
        if let Some(tmax) = hits.iter().map(|h| h.1).reduce(f64::max) {
            let ys: Vec<f64> = hits.iter().filter(|h| h.1 > tmax - 1.5).map(|h| h.0).collect();
            let yc = ys.iter().sum::<f64>() / ys.len() as f64;
            let p = Point3::new(c.0 + tmax * dir.0, yc, c.1 + tmax * dir.1);
            if p.z < z_max && p.x > c.0 + 5.0 {
                raw.push(p);
            }
        }
        th += 0.25;
    }
    // Resample every 2 mm of side-view arc length, then smooth over +-4 samples.
    let mut s = vec![0.0];
    for w in raw.windows(2) {
        s.push(s.last().unwrap() + ((w[1].x - w[0].x).powi(2) + (w[1].z - w[0].z).powi(2)).sqrt());
    }
    let total = *s.last().unwrap_or(&0.0);
    let step = 2.0;
    let mut even = Vec::new();
    let mut i = 0;
    let mut sk = 0.0;
    while sk <= total && raw.len() > 1 {
        while i + 2 < s.len() && s[i + 1] < sk {
            i += 1;
        }
        let span = s[i + 1] - s[i];
        let t = if span > 1e-9 { ((sk - s[i]) / span).clamp(0.0, 1.0) } else { 0.0 };
        even.push(raw[i] + (raw[i + 1] - raw[i]) * t);
        sk += step;
    }
    let half = 4usize;
    (0..even.len())
        .map(|k| {
            let h = half.min(k).min(even.len() - 1 - k);
            let w = &even[k - h..=k + h];
            Point3::from(w.iter().map(|p| p.coords).sum::<Vector3<f64>>() / w.len() as f64)
        })
        .collect()
}

fn smoothstep(e0: f64, e1: f64, x: f64) -> f64 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// A rounded rectangle's signed distance in 2D: centred, half-extents `hx`, `hy`, corner `r`.
fn rounded_rect(x: f64, y: f64, hx: f64, hy: f64, r: f64) -> f64 {
    let qx = x.abs() - (hx - r);
    let qy = y.abs() - (hy - r);
    (qx.max(0.0).powi(2) + qy.max(0.0).powi(2)).sqrt() + qx.max(qy).min(0.0) - r
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PartKind {
    Shell,
    ToeCap,
    Outsole,
    Liner,
    AnklePads,
    Footbed,
    Tongue,
    TendonGuard,
}

/// A part: what it is, what it's made of, how it's drawn, the mesh cell it needs.
pub struct PartSpec {
    pub kind: PartKind,
    pub name: &'static str,
    pub material: Material,
    pub color: [f64; 3],
    pub cell: f64,
}

pub fn part_specs() -> Vec<PartSpec> {
    use PartKind::*;
    vec![
        PartSpec { kind: Shell, name: "shell", material: PA12_MJF, color: [0.16, 0.25, 0.42], cell: 0.75 },
        PartSpec { kind: ToeCap, name: "toe cap", material: PA12_MJF, color: [0.13, 0.13, 0.15], cell: 0.3 },
        PartSpec { kind: Outsole, name: "outsole", material: PA12_MJF, color: [0.09, 0.09, 0.10], cell: 0.75 },
        PartSpec { kind: Liner, name: "liner", material: LINER_FOAM, color: [0.42, 0.44, 0.48], cell: 1.0 },
        PartSpec { kind: AnklePads, name: "ankle pads", material: LINER_FOAM, color: [0.78, 0.36, 0.22], cell: 1.0 },
        PartSpec { kind: Footbed, name: "footbed", material: LINER_FOAM, color: [0.22, 0.55, 0.45], cell: 1.0 },
        PartSpec { kind: Tongue, name: "tongue", material: LINER_FOAM, color: [0.90, 0.90, 0.87], cell: 1.0 },
        PartSpec { kind: TendonGuard, name: "tendon guard", material: TPU, color: [0.22, 0.22, 0.25], cell: 0.6 },
    ]
}

/// The boot's shared geometry: the last's base with the leg extended, the landmarks, the dorsal
/// line and the stations along it, the eyelets and the lace path.
pub struct BootGeom {
    pub p: HockeyParams,
    pub lm: Landmarks,
    base: Solid,
    /// Above this height the leg is the prism of its section here.
    pub z_ext: f64,
    pub prof: Profile,
    /// Along the dorsal line: the throat's front end, where it meets the cuff's top, the
    /// tongue's top.
    pub s_front: f64,
    pub s_cuff: f64,
    pub s_tongue_top: f64,
    /// Cuff top at the leg's centre line, and at the back of the leg.
    pub z_cuff: f64,
    pub z_trim_back: f64,
    /// The toe cap's back edge at the dorsal line: x, and the ridge y there.
    pub toe_cap_x: f64,
    pub toe_cap_y: f64,
    /// Eyelet centres on the facing's outer surface and their outward axes, in pairs from the
    /// toe end: [2k] on the -y side, [2k + 1] on the +y side.
    pub eyelets: Vec<(Point3<f64>, Vector3<f64>)>,
    /// The lace: the criss-cross with its knot, then the bow's loops and tails.
    pub lace_paths: Vec<Vec<Point3<f64>>>,
    pub bounds: Aabb,
    /// The outsole's bottom (the chassis or holder mounts here).
    pub sole_bottom: f64,
}

impl BootGeom {
    pub fn new(base: Solid, base_bounds: Aabb, lm: Landmarks, p: HockeyParams) -> Self {
        let z_ext = lm.leg_top - 15.0;
        let z_cuff = lm.medial.z.max(lm.lateral.z) + p.cuff_above_ankle;
        let tilt = p.trim_tilt_deg.to_radians().tan();
        let z_trim_back = z_cuff + (lm.leg_c.0 - lm.heel_back_x) * tilt;
        let oo_c = p.ease + p.liner + p.wall;
        let sole_bottom = -oo_c - p.sole;
        let top = z_trim_back + p.guard_above_cuff + 10.0;
        let bounds = Aabb::new(
            Point3::new(base_bounds.min.x, base_bounds.min.y, sole_bottom - 2.0),
            Point3::new(base_bounds.max.x, base_bounds.max.y, top),
        );
        let mut g = BootGeom {
            p,
            lm,
            base,
            z_ext,
            prof: Profile::new(vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)]),
            s_front: 0.0,
            s_cuff: 0.0,
            s_tongue_top: 0.0,
            z_cuff,
            z_trim_back,
            toe_cap_x: 0.0,
            toe_cap_y: 0.0,
            eyelets: Vec::new(),
            lace_paths: Vec::new(),
            bounds,
            sole_bottom,
        };
        // The dorsal line, on the base surface: rays from behind the ankle.
        let c = (lm.heel_back_x + 8.0, lm.medial.z.min(lm.lateral.z) - 10.0);
        let d = |q: Point3<f64>| g.d(q);
        let pts = dorsal_profile(&d, c, top - 5.0, bounds.min.y + 15.0, bounds.max.y - 15.0);
        g.prof = Profile::new(pts);
        // Centre the line across the foot and leg: the middle of the section's width 15 mm in
        // from the line, not the ridge's most forward point, which on the shin and the ankle's
        // front sits toward the medial side (y 20 against the leg's centre at 11 on stand-in 022)
        // and put the medial eyelets on the side of the ankle.
        let mut centred = g.prof.pts.clone();
        for (k, q) in centred.iter_mut().enumerate() {
            let nn = g.prof.normal(g.prof.s[k]);
            let (x, z) = (q.x - nn.x * 15.0, q.z - nn.z * 15.0);
            if g.d(Point3::new(x, q.y, z)) >= 0.0 {
                continue;
            }
            let edge = |dir: f64| {
                let mut y = q.y;
                while g.d(Point3::new(x, y + dir * 0.5, z)) < 0.0 && (y - q.y).abs() < 150.0 {
                    y += dir * 0.5;
                }
                y
            };
            q.y = 0.5 * (edge(-1.0) + edge(1.0));
        }
        let half = 6usize;
        let smooth: Vec<Point3<f64>> = (0..centred.len())
            .map(|k| {
                let h = half.min(k).min(centred.len() - 1 - k);
                let w = &centred[k - h..=k + h];
                Point3::new(centred[k].x, w.iter().map(|p| p.y).sum::<f64>() / w.len() as f64, centred[k].z)
            })
            .collect();
        g.prof = Profile::new(smooth);
        let x_front = p.throat_front * lm.length;
        g.s_front = g.prof.find(|q| q.x <= x_front).unwrap_or(0.0);
        g.s_cuff = g.prof.find(|q| q.z >= g.z_trim(q.x)).unwrap_or(g.prof.len());
        g.s_tongue_top = g.prof.find(|q| q.z >= g.z_trim(q.x) + p.tongue_above_cuff).unwrap_or(g.prof.len());
        let (front, _) = g.prof.at(g.s_front);
        g.toe_cap_x = front.x + 2.0;
        g.toe_cap_y = front.y;
        g.place_eyelets();
        g.lace();
        g
    }

    /// The last's base, the leg carried up as a prism above `z_ext`.
    pub fn d(&self, q: Point3<f64>) -> f64 {
        self.base.evaluate(&Point3::new(q.x, q.y, q.z.min(self.z_ext)))
    }

    /// Unit outward normal of the base (central differences).
    pub fn grad(&self, q: Point3<f64>) -> Vector3<f64> {
        let h = 0.5;
        let g = Vector3::new(
            self.d(q + Vector3::x() * h) - self.d(q - Vector3::x() * h),
            self.d(q + Vector3::y() * h) - self.d(q - Vector3::y() * h),
            self.d(q + Vector3::z() * h) - self.d(q - Vector3::z() * h),
        );
        if g.norm() > 1e-12 { g.normalize() } else { Vector3::z() }
    }

    /// The cuff's top edge, rising toward the back.
    pub fn z_trim(&self, x: f64) -> f64 {
        self.z_cuff + (self.lm.leg_c.0 - x) * self.p.trim_tilt_deg.to_radians().tan()
    }

    /// The cuff's flare: grows (quadratically) from `flare_from` above the higher ankle bone to
    /// the top, and mostly at the sides, where the ankle tips (30% at the front and back, where
    /// the tongue and the tendon guard are).
    fn flare(&self, q: Point3<f64>) -> f64 {
        let z0 = self.lm.medial.z.max(self.lm.lateral.z) + self.p.flare_from;
        let t = ((q.z - z0) / (self.z_trim(q.x) - z0)).clamp(0.0, 1.0);
        let (dx, dy) = (q.x - self.lm.leg_c.0, q.y - self.lm.leg_c.1);
        let side = dy.abs() / (dx * dx + dy * dy).sqrt().max(1e-9);
        self.p.flare * t * t * (0.3 + 0.7 * side)
    }

    fn pocket(&self, q: Point3<f64>) -> f64 {
        let s2 = 2.0 * self.p.ankle_sigma * self.p.ankle_sigma;
        [self.lm.medial, self.lm.lateral].iter().map(|m| self.p.ankle_pocket * (-(q - m).norm_squared() / s2).exp()).sum()
    }

    /// Where the liner ends and the shell begins (offset from the base).
    pub fn inner_off(&self, q: Point3<f64>) -> f64 {
        self.p.ease + self.p.liner + self.flare(q) + self.pocket(q)
    }

    fn throat_w(&self, s: f64) -> f64 {
        let t = ((s - self.s_front) / (self.s_cuff - self.s_front).max(1.0)).clamp(0.0, 1.0);
        self.p.throat_w0 + (self.p.throat_w1 - self.p.throat_w0) * t
    }

    /// The throat's outline in (arc length, y): negative inside; a round front end, open at the
    /// top. Only meaningful near the dorsal line (`n.d < reach`).
    fn throat(&self, n: &Near, y: f64) -> f64 {
        let hw = 0.5 * self.throat_w(n.s);
        let a = (y - n.y).abs();
        let s0 = self.s_front + hw;
        if n.s >= s0 { a - hw } else { ((s0 - n.s).powi(2) + a * a).sqrt() - hw }
    }

    fn facing(&self, n: &Near, throat: f64) -> f64 {
        let a = 1.0 - smoothstep(self.p.facing_w - 3.0, self.p.facing_w, throat);
        let b = 1.0 - smoothstep(self.p.reach - 6.0, self.p.reach, n.d);
        self.p.facing_t * a * b
    }

    /// The tongue's outline in (arc length, y): a rounded rectangle.
    fn tongue_outline(&self, n: &Near, y: f64) -> f64 {
        let hw = 0.5 * self.throat_w(n.s) + self.p.tongue_overlap;
        let (s0, s1) = (self.s_front - 15.0, self.s_tongue_top);
        rounded_rect(n.s - 0.5 * (s0 + s1), y - n.y, 0.5 * (s1 - s0), hw, 12.0)
    }

    /// The shell's outer surface offset (with the facing).
    fn outer_off(&self, q: Point3<f64>, n: &Near) -> f64 {
        let th = self.throat(n, q.y);
        self.inner_off(q) + self.p.wall + self.facing(n, th)
    }

    /// The tendon guard's band: from the shell's outer surface, kicking back at the top.
    fn guard(&self, q: Point3<f64>, d: f64) -> f64 {
        let p = &self.p;
        let (zb, z1) = (self.z_trim_back, self.z_trim_back + p.guard_above_cuff);
        let z0 = zb - 30.0;
        let u = ((q.z - zb) / (z1 - zb)).clamp(0.0, 1.0);
        let off = self.inner_off(q) + p.wall + p.guard_kick * u * u;
        let band = (d - (off + p.guard_t)).max(off - d);
        let hw = 22.0 - 5.0 * ((q.z - z0) / (z1 - z0)).clamp(0.0, 1.0);
        let rect = rounded_rect(q.z - 0.5 * (z0 + z1), q.y - self.lm.leg_c.1, 0.5 * (z1 - z0), hw, 8.0);
        let back = q.x - (self.lm.leg_c.0 - 10.0);
        band.max(rect).max(back)
    }

    /// Signed distance of one part (negative inside), mm; approximate away from the surface.
    pub fn sdf(&self, k: PartKind, q: Point3<f64>) -> f64 {
        let p = &self.p;
        let d = self.d(q);
        // Every part lies between ease and ~35 mm out from the base.
        if d > 50.0 {
            return d - 45.0;
        }
        if d < -10.0 {
            return -d - 5.0;
        }
        let io = self.inner_off(q);
        let zt = self.z_trim(q.x);
        match k {
            PartKind::Shell => {
                let n = self.prof.closest(q.x, q.z);
                let th = self.throat(&n, q.y);
                let oo = io + p.wall + self.facing(&n, th);
                let mut s = (d - oo).max(io - d).max(q.z - zt);
                if s > 3.0 {
                    return s;
                }
                s = s.max(-(th.max(n.d - p.reach)));
                for (c, a) in &self.eyelets {
                    let v = q - c;
                    if v.norm_squared() < 144.0 {
                        let along = v.dot(a);
                        let r = (v - a * along).norm();
                        s = s.max(-((r - 3.4).max(along.abs() - 8.0)));
                    }
                }
                s
            }
            PartKind::ToeCap => {
                let n = self.prof.closest(q.x, q.z);
                let oo = self.outer_off(q, &n);
                let band = (d - (oo + p.toe_cap_t)).max(oo - d);
                let edge = self.toe_cap_x + 25.0 * ((q.y - self.toe_cap_y) / 45.0).powi(2) - q.x;
                let low = -(p.ease + p.liner + p.wall) - q.z;
                let throat = self.throat(&n, q.y).max(n.d - p.reach);
                band.max(edge).max(low).max(-throat)
            }
            PartKind::Outsole => {
                let oo_c = p.ease + p.liner + p.wall;
                let outline = self.d(Point3::new(q.x, q.y, 3.0)) - oo_c;
                let slab = (self.sole_bottom - q.z).max(q.z - (-oo_c + 1.5));
                let n = self.prof.closest(q.x, q.z);
                let outside_shell = self.outer_off(q, &n) - d;
                outline.max(slab).max(outside_shell)
            }
            PartKind::Liner | PartKind::AnklePads | PartKind::Footbed => {
                let band = (d - io).max(p.ease - d).max(q.z - (zt + 3.0));
                let pad = [self.lm.medial, self.lm.lateral].iter().map(|m| (q - m).norm() - p.ankle_pad_r).fold(f64::MAX, f64::min);
                let below = q.z + p.ease; // < 0 under the foot
                match k {
                    PartKind::Footbed => band.max(below),
                    PartKind::AnklePads => band.max(pad).max(-below),
                    _ => {
                        if band > 3.0 {
                            return band;
                        }
                        let n = self.prof.closest(q.x, q.z);
                        let tongue = self.tongue_outline(&n, q.y).max(n.d - p.reach);
                        band.max(-pad).max(-below).max(-tongue)
                    }
                }
            }
            PartKind::Tongue => {
                let n = self.prof.closest(q.x, q.z);
                let outline = self.tongue_outline(&n, q.y).max(n.d - p.reach);
                if outline > 3.0 {
                    return outline;
                }
                let pad = (d - io).max(p.ease - d);
                let raised = (d - (io + p.wall + p.tongue_proud)).max(p.ease - d);
                let open = (self.throat(&n, q.y) + 1.0).min(zt + 0.5 - q.z);
                pad.min(raised.max(open)).max(outline)
            }
            PartKind::TendonGuard => self.guard(q, d),
        }
    }

    /// Where each part can be (a box per part keeps the meshing fast).
    pub fn part_bounds(&self, k: PartKind) -> Aabb {
        let b = self.bounds;
        let p = &self.p;
        let oo_c = p.ease + p.liner + p.wall;
        let lm = &self.lm;
        let mk = |x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64| {
            Aabb::new(Point3::new(x0.max(b.min.x), y0.max(b.min.y), z0.max(b.min.z)), Point3::new(x1.min(b.max.x), y1.min(b.max.y), z1.min(b.max.z)))
        };
        match k {
            PartKind::Shell | PartKind::Liner => mk(b.min.x, b.min.y, -oo_c - 2.0, b.max.x, b.max.y, self.z_trim_back + 6.0),
            PartKind::ToeCap => mk(self.toe_cap_x - 3.0, b.min.y, -oo_c - 2.0, self.lm.length + 14.0, b.max.y, 75.0),
            PartKind::Outsole => mk(b.min.x, b.min.y, self.sole_bottom - 1.5, b.max.x, b.max.y, -oo_c + 3.0),
            PartKind::Footbed => mk(b.min.x, b.min.y, -oo_c - 1.0, b.max.x, b.max.y, -p.ease + 1.0),
            PartKind::AnklePads => {
                let r = p.ankle_pad_r + 2.0;
                let (lo, hi) = (lm.medial.coords.inf(&lm.lateral.coords), lm.medial.coords.sup(&lm.lateral.coords));
                mk(lo.x - r, lo.y - r, lo.z - r, hi.x + r, hi.y + r, hi.z + r)
            }
            PartKind::Tongue => {
                let (mut lo, mut hi) = (Vector3::repeat(f64::MAX), Vector3::repeat(f64::MIN));
                for (pt, s) in self.prof.pts.iter().zip(&self.prof.s) {
                    if *s >= self.s_front - 20.0 && *s <= self.s_tongue_top + 5.0 {
                        lo = lo.inf(&pt.coords);
                        hi = hi.sup(&pt.coords);
                    }
                }
                let m = 0.5 * p.throat_w1 + p.tongue_overlap + 4.0;
                mk(lo.x - 30.0, lo.y - m, lo.z - 30.0, hi.x + 30.0, hi.y + m, hi.z + 30.0)
            }
            PartKind::TendonGuard => {
                let (yc, xc) = (lm.leg_c.1, lm.leg_c.0);
                mk(b.min.x, yc - 26.0, self.z_trim_back - 33.0, xc - 8.0, yc + 26.0, self.z_trim_back + p.guard_above_cuff + 3.0)
            }
        }
    }

    /// One part as a solid, optionally cut to the half-space `n . x < off`.
    pub fn solid(self: &Arc<Self>, k: PartKind, cut: Option<(Vector3<f64>, f64)>) -> Solid {
        let g = self.clone();
        let b = self.part_bounds(k);
        Solid::user_fn(
            move |q: Point3<f64>| {
                let v = g.sdf(k, q);
                match cut {
                    Some((n, off)) => v.max(n.dot(&q.coords) - off),
                    None => v,
                }
            },
            b,
        )
    }

    /// The part with a conservative interval bound (centre value +- `LIP` x half-diagonal), so
    /// cf-design's adaptive mesher can prune empty space. Used for the light display meshes
    /// (`mesh_simplified`); the parts are weighed and saved from the uniform meshes of `solid`.
    /// The layered offsets are not exact distances, hence the margin over 1.
    pub fn solid_bounded(self: &Arc<Self>, k: PartKind) -> Solid {
        const LIP: f64 = 3.0;
        let g = self.clone();
        let f = move |q: Point3<f64>| g.sdf(k, q);
        let f2 = f.clone();
        Solid::user_fn_with_interval(
            f,
            move |bb: &Aabb| {
                let c = Point3::from((bb.min.coords + bb.max.coords) * 0.5);
                let h = 0.5 * (bb.max - bb.min).norm();
                let v = f2(c);
                (v - LIP * h, v + LIP * h)
            },
            self.part_bounds(k),
        )
    }

    fn place_eyelets(&mut self) {
        let p = self.p;
        let (s0, s1) = (self.s_front + 10.0, self.s_cuff - 12.0);
        if s1 <= s0 {
            return;
        }
        let n = ((s1 - s0) / p.eyelet_spacing).round() as usize + 1;
        for k in 0..n {
            let s = s0 + (s1 - s0) * k as f64 / (n - 1).max(1) as f64;
            let (pt, _) = self.prof.at(s);
            let nn = self.prof.normal(s);
            let hw = 0.5 * self.throat_w(s);
            for side in [-1.0, 1.0] {
                // In from outside along the dorsal line's normal, at the eyelet row's y, to the
                // facing's outer surface: the first surface met is the front-facing quarter. (Out
                // from inside, a row far round a narrow leg met the side wall at a grazing angle.)
                let y = pt.y + side * (hw + p.eyelet_inset);
                let at = |t: f64| Point3::new(pt.x + nn.x * t, y, pt.z + nn.z * t);
                let f = |t: f64| {
                    let q = at(t);
                    let near = self.prof.closest(q.x, q.z);
                    self.d(q) - self.outer_off(q, &near)
                };
                let mut t = 45.0;
                while t > -30.0 && f(t) > 0.0 {
                    t -= 0.25;
                }
                let (mut a, mut b) = (t, t + 0.25);
                for _ in 0..30 {
                    let m = 0.5 * (a + b);
                    if f(m) <= 0.0 {
                        a = m;
                    } else {
                        b = m;
                    }
                }
                let c = at(0.5 * (a + b));
                let axis = self.grad(c);
                self.eyelets.push((c, axis));
            }
        }
    }

    /// `q` moved out along the last's normal until the lace (radius `lace_r`) clears the shell and
    /// the tongue.
    fn lift(&self, q: Point3<f64>) -> Point3<f64> {
        let r = self.p.lace_r;
        let dir = self.grad(q);
        let mut q = q;
        for _ in 0..200 {
            let env = self.sdf(PartKind::Shell, q).min(self.sdf(PartKind::Tongue, q));
            if env >= r + 0.4 {
                break;
            }
            q += dir * 0.25;
        }
        q
    }

    /// Criss-cross lacing: a bar across the lowest pair, then each end crosses to the other
    /// side's next eyelet up; the two ends meet in a knot above the top pair, tied in a bow.
    /// Between eyelets the lace is lifted clear of the shell and tongue.
    fn lace(&mut self) {
        let n = self.eyelets.len() / 2;
        if n < 2 {
            return;
        }
        let r = self.p.lace_r;
        let top = |k: usize, side: usize| {
            let (c, a) = self.eyelets[2 * k + side];
            c + a * (0.9 + r)
        };
        let strand = |start: usize| -> Vec<Point3<f64>> { (0..n).map(|k| top(k, (start + k) % 2)).collect() };
        let mut nodes: Vec<Point3<f64>> = strand(1).into_iter().rev().collect();
        nodes.extend(strand(0));
        // The knot: above the middle of the top pair.
        let (ta, tb) = (top(n - 1, 0), top(n - 1, 1));
        let (_, tan) = self.prof.at(self.s_cuff - 12.0);
        let knot = self.lift(Point3::from((ta.coords + tb.coords) * 0.5) + tan * 6.0);
        nodes.insert(0, knot);
        nodes.push(knot);
        let mut path = vec![nodes[0]];
        for w in nodes.windows(2) {
            for j in 1..10 {
                path.push(self.lift(w[0] + (w[1] - w[0]) * (j as f64 / 10.0)));
            }
            path.push(w[1]);
        }
        let mut paths = vec![path];
        // The bow: a loop and a tail each side, lying on the tongue.
        let u = (tb - ta).normalize();
        let nk = self.grad(knot);
        let mut v = nk.cross(&u).normalize();
        if v.dot(&tan) < 0.0 {
            v = -v;
        }
        for side in [-1.0, 1.0] {
            let lp: Vec<Point3<f64>> = (0..=24)
                .map(|i| {
                    let th = i as f64 / 24.0 * std::f64::consts::TAU;
                    self.lift(knot + u * (side * 11.0 * (1.0 - th.cos())) + v * (7.0 * th.sin()))
                })
                .collect();
            paths.push(lp);
            let tail: Vec<Point3<f64>> = (0..=8).map(|i| {
                let t = i as f64 / 8.0;
                self.lift(knot + u * (side * 6.0 * t) - v * (32.0 * t))
            }).collect();
            paths.push(tail);
        }
        self.lace_paths = paths;
    }

    /// The eyelets as solids: a grommet each (barrel through the facing, flange outside, bore).
    pub fn grommets(&self) -> Vec<Solid> {
        let depth = self.p.wall + self.p.facing_t + 0.4;
        self.eyelets
            .iter()
            .map(|(c, a)| {
                let rot = UnitQuaternion::rotation_between(&Vector3::z(), a).unwrap_or_else(UnitQuaternion::identity);
                let barrel = Solid::cylinder(3.4, 0.5 * depth).translate(Vector3::new(0.0, 0.0, -0.5 * depth));
                let flange = Solid::cylinder(5.0, 0.45).translate(Vector3::new(0.0, 0.0, 0.45));
                let bore = Solid::cylinder(2.2, depth + 5.0);
                barrel.union(flange).subtract(bore).rotate(rot).translate(c.coords)
            })
            .collect()
    }

    pub fn laces(&self) -> Option<Solid> {
        self.lace_paths
            .iter()
            .filter(|p| p.len() >= 2)
            .map(|p| Solid::pipe(p.clone(), self.p.lace_r))
            .reduce(|a, b| a.union(b))
    }
}

/// Standard base64 (RFC 4648, with padding).
pub fn base64(data: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for c in data.chunks(3) {
        let b = [c[0], *c.get(1).unwrap_or(&0), *c.get(2).unwrap_or(&0)];
        let n = (u32::from(b[0]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[2]);
        for k in 0..4 {
            if k <= c.len() {
                out.push(T[((n >> (18 - 6 * k)) & 63) as usize] as char);
            } else {
                out.push('=');
            }
        }
    }
    out
}

fn merge(meshes: &[IndexedMesh]) -> IndexedMesh {
    let mut out = IndexedMesh::default();
    for m in meshes {
        let o = out.vertices.len() as u32;
        out.vertices.extend_from_slice(&m.vertices);
        out.faces.extend(m.faces.iter().map(|f| f.map(|i| i + o)));
    }
    out
}

fn translated(m: &IndexedMesh, v: Vector3<f64>) -> IndexedMesh {
    let mut out = m.clone();
    for p in &mut out.vertices {
        *p += v;
    }
    out
}

/// Enclosed volume of a closed, outward-wound mesh, mm^3.
pub fn mesh_volume(m: &IndexedMesh) -> f64 {
    m.faces
        .iter()
        .map(|f| {
            let [a, b, c] = f.map(|i| m.vertices[i as usize].coords);
            a.dot(&b.cross(&c)) / 6.0
        })
        .sum()
}

fn mesh_bounds(m: &IndexedMesh) -> (Vector3<f64>, Vector3<f64>) {
    m.vertices.iter().fold((Vector3::repeat(f64::MAX), Vector3::repeat(f64::MIN)), |(lo, hi), v| (lo.inf(&v.coords), hi.sup(&v.coords)))
}

/// `boot <scan> [out]`: the hockey boot on a foot, every part meshed, weighed, saved, drawn.
pub fn run(scan_path: &Path, out: &Path) -> Result<(), String> {
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let t0 = Instant::now();
    let (raw, _, _) = scan::load(scan_path)?;
    let m = scan::measure(&raw, 20);
    let foot = foot::build(&raw, &m, 1.5, 24.0)?;
    println!("foot: {} ({:.1} x {:.1} mm, scan {:.0} mm tall)", scan_path.display(), foot.length, foot.width, foot.height);
    // The grid reaches 35 mm past the closed foot: the tendon guard's top stands ~30 mm off it.
    let lb = foot::last_base(&foot, 35.0)?;
    let lm = landmarks(&foot);
    let side = |s: f64| if s > 0.0 { "+y" } else { "-y" };
    println!(
        "landmarks: medial ankle bone ({:.0}, {:.0}, {:.0}) on the {} side, lateral ({:.0}, {:.0}, {:.0}); heel back x {:.0}; leg centre ({:.0}, {:.0}) at the scan's top, z {:.0}",
        lm.medial.x, lm.medial.y, lm.medial.z, side(lm.medial_side), lm.lateral.x, lm.lateral.y, lm.lateral.z, lm.heel_back_x, lm.leg_c.0, lm.leg_c.1, lm.leg_top
    );
    let p = HockeyParams::default();
    let t = Instant::now();
    let g = BootGeom::new(lb.solid, lb.bounds, lm, p);
    let (front, _) = g.prof.at(g.s_front);
    let (cuff, _) = g.prof.at(g.s_cuff);
    let (ttop, _) = g.prof.at(g.s_tongue_top);
    println!(
        "dorsal line: {} points, {:.0} mm; throat from ({:.0}, {:.0}) to the cuff at ({:.0}, {:.0}), tongue top ({:.0}, {:.0}); cuff top z {:.0} at the leg's centre, {:.0} at the back; {} eyelets a side ({:.1} s)",
        g.prof.pts.len(),
        g.prof.len(),
        front.x,
        front.z,
        cuff.x,
        cuff.z,
        ttop.x,
        ttop.z,
        g.z_cuff,
        g.z_trim_back,
        g.eyelets.len() / 2,
        t.elapsed().as_secs_f64()
    );
    if std::env::var_os("BOOT_DEBUG").is_some() {
        for (i, (c, a)) in g.eyelets.iter().enumerate() {
            let near = g.prof.closest(c.x, c.z);
            println!(
                "  eyelet {i:2}: ({:6.1}, {:6.1}, {:6.1}) axis ({:+.2}, {:+.2}, {:+.2}); ridge y {:.1} at s {:.0}, {:.1} mm off the line",
                c.x, c.y, c.z, a.x, a.y, a.z, near.y, near.s, near.d
            );
        }
        for (k, q) in g.prof.pts.iter().enumerate().step_by(8) {
            println!("  dorsal line {k:3}: s {:5.0}  ({:6.1}, {:6.1}, {:6.1})", g.prof.s[k], q.x, q.y, q.z);
        }
        if std::env::var("BOOT_DEBUG").as_deref() == Ok("geom") {
            return Ok(());
        }
    }
    let g = Arc::new(g);

    // Mesh every part, in parallel.
    let t = Instant::now();
    let specs = part_specs();
    let grommets = g.grommets();
    let laces = g.laces();
    let (meshes, eyelet_mesh, lace_mesh) = std::thread::scope(|sc| {
        let hs: Vec<_> = specs
            .iter()
            .map(|sp| {
                let g = g.clone();
                let (kind, cell) = (sp.kind, sp.cell);
                sc.spawn(move || mesh_of(&g.solid(kind, None), cell))
            })
            .collect();
        let eh = sc.spawn(|| merge(&grommets.iter().map(|s| mesh_of(s, 0.25)).collect::<Vec<_>>()));
        let lh = sc.spawn(|| laces.as_ref().map(|l| mesh_of(l, 0.5)).unwrap_or_default());
        (hs.into_iter().map(|h| h.join().unwrap()).collect::<Vec<_>>(), eh.join().unwrap(), lh.join().unwrap())
    });
    println!("meshed {} parts + eyelets + laces ({:.1} s)", specs.len(), t.elapsed().as_secs_f64());

    // Weigh (from the meshes' enclosed volumes), save.
    let mut summary = String::new();
    let _ = writeln!(summary, "hockey boot (track B, round 2 of the boot), {}", scan_path.display());
    let _ = writeln!(summary, "foot {:.1} x {:.1} mm; scan {:.0} mm tall, leg carried up as a prism above z {:.0}", foot.length, foot.width, foot.height, g.z_ext);
    let _ = writeln!(
        summary,
        "landmarks: medial ankle bone ({:.0}, {:.0}, {:.0}), lateral ({:.0}, {:.0}, {:.0}), medial side {}",
        lm.medial.x, lm.medial.y, lm.medial.z, lm.lateral.x, lm.lateral.y, lm.lateral.z, side(lm.medial_side)
    );
    let _ = writeln!(
        summary,
        "cuff top z {:.0} (leg centre) to {:.0} (back); throat from x {:.0}; tongue top z {:.0}; tendon guard top z {:.0}; {} eyelets a side",
        g.z_cuff,
        g.z_trim_back,
        front.x,
        ttop.z,
        g.z_trim_back + p.guard_above_cuff,
        g.eyelets.len() / 2
    );
    let _ = writeln!(summary, "parts (mass from each mesh's enclosed volume):");
    let mut total = 0.0;
    let save = |name: &str, mesh: &IndexedMesh| -> Result<(), String> {
        let path = out.join(name);
        save_mesh(mesh, &path).map_err(|e| format!("{}: {e}", path.display()))
    };
    let mut rows: Vec<(String, f64, &'static str, f64)> = Vec::new();
    for (sp, mesh) in specs.iter().zip(&meshes) {
        let vol = mesh_volume(mesh);
        rows.push((sp.name.to_string(), vol, sp.material.name, vol * sp.material.density * 1e-6));
        save(&format!("{}.stl", sp.name.replace(' ', "_")), mesh)?;
    }
    for (name, mesh, mat) in [("eyelets", &eyelet_mesh, &AL_6061_T6), ("laces", &lace_mesh, &LACE)] {
        let vol = mesh_volume(mesh);
        rows.push((name.to_string(), vol, mat.name, vol * mat.density * 1e-6));
        save(&format!("{name}.stl"), mesh)?;
    }
    for (name, vol, mat, grams) in &rows {
        total += grams;
        let line = format!("  {name:<13} {:7.1} cm3  {grams:6.1} g  ({mat})", vol / 1000.0);
        println!("{line}");
        let _ = writeln!(summary, "{line}");
    }
    let shell_cf = rows[0].1 * CFRP.density * 1e-6;
    let line = format!(
        "  total {total:.0} g (shell in {}; in {} the shell alone would be {shell_cf:.0} g); foam and lace densities assumed",
        PA12_MJF.name, CFRP.name
    );
    println!("{line}");
    let _ = writeln!(summary, "{line}");
    // Do the weights hold at a finer mesh? The shell (the heaviest part) and the tendon guard (a
    // thin one), meshed finer in parallel. And mass_properties on the tendon guard, for comparison:
    // it integrates the field, so it leans on the field being a true distance, which these
    // layered offsets only roughly are; a mesh's volume needs only the surface.
    let fine_cells = [(0usize, PartKind::Shell, 0.5), (7, PartKind::TendonGuard, 0.3)];
    let fine: Vec<f64> = std::thread::scope(|sc| {
        let hs: Vec<_> = fine_cells
            .iter()
            .map(|&(i, kind, cell)| {
                let g = g.clone();
                let rho = specs[i].material.density;
                sc.spawn(move || mesh_volume(&mesh_of(&g.solid(kind, None), cell)) * rho * 1e-6)
            })
            .collect();
        hs.into_iter().map(|h| h.join().unwrap()).collect()
    });
    for (&(i, _, cell), f) in fine_cells.iter().zip(&fine) {
        let line = format!(
            "  check: {} {:.2} g at {} mm cells, {f:.2} g at {cell} mm ({:+.1}%)",
            specs[i].name,
            rows[i].3,
            specs[i].cell,
            (f / rows[i].3 - 1.0) * 100.0
        );
        println!("{line}");
        let _ = writeln!(summary, "{line}");
    }
    let mp = mass_properties(&g.solid(PartKind::TendonGuard, None), TPU.density, 0.3).map_or(0.0, |m| m.mass * 1000.0);
    let line = format!("  check: tendon guard by mass_properties at 0.3 mm: {mp:.2} g");
    println!("{line}");
    let _ = writeln!(summary, "{line}");

    // Draw.
    let t = Instant::now();
    let pretty = Style::pretty();
    let mut layers: Vec<Layer> = specs.iter().zip(&meshes).map(|(sp, m)| Layer::solid(m, sp.color)).collect();
    layers.push(Layer::solid(&eyelet_mesh, [0.80, 0.81, 0.84]));
    layers.push(Layer::solid(&lace_mesh, [0.96, 0.96, 0.94]));
    let e = |e: std::io::Error| e.to_string();
    render_styled(&layers, &standard_views(), 2.0, &out.join("boot.png"), pretty).map_err(e)?;
    let ms = lm.medial_side;
    let hero = vec![
        View::looking(Vector3::new(-0.62, 0.62 * ms, -0.40), Vector3::z()),
        View::looking(Vector3::new(0.60, -0.62 * ms, -0.38), Vector3::z()),
    ];
    render_styled(&layers, &hero, 3.0, &out.join("boot_hero.png"), pretty).map_err(e)?;

    // The parts side by side.
    let groups: Vec<(Vec<usize>, &str)> = vec![
        (vec![0, 1, 2], "shell, toe cap, outsole"),
        (vec![3, 4, 5], "liner, ankle pads, footbed"),
        (vec![6], "tongue"),
        (vec![7], "tendon guard"),
        (vec![8, 9], "eyelets, laces"),
    ];
    let all: Vec<&IndexedMesh> = meshes.iter().chain([&eyelet_mesh, &lace_mesh]).collect();
    let colors: Vec<[f64; 3]> = layers.iter().map(|l| l.color).collect();
    let mut placed: Vec<(IndexedMesh, [f64; 3])> = Vec::new();
    let mut x = 0.0;
    for (ids, _) in &groups {
        let (lo, hi) = ids.iter().fold((Vector3::repeat(f64::MAX), Vector3::repeat(f64::MIN)), |(lo, hi), i| {
            let (a, b) = mesh_bounds(all[*i]);
            (lo.inf(&a), hi.sup(&b))
        });
        for i in ids {
            placed.push((translated(all[*i], Vector3::new(x - lo.x, 0.0, 0.0)), colors[*i]));
        }
        x += hi.x - lo.x + 40.0;
    }
    let lineup: Vec<Layer> = placed.iter().map(|(m, c)| Layer::solid(m, *c)).collect();
    render_styled(&lineup, &[View::looking(Vector3::new(-0.35, 0.75 * ms, -0.45), Vector3::z())], 1.6, &out.join("boot_parts.png"), pretty)
        .map_err(e)?;

    // Sections: across the ankle (looking back at the cut), and lengthwise through the leg's
    // centre (looking at the cut from the medial side), with the foot in them.
    let skin = [0.86, 0.68, 0.58];
    let cuts = [
        (Vector3::x(), 0.5 * (lm.medial.x + lm.lateral.x), "ankle"),
        (Vector3::y() * ms, lm.leg_c.1 * ms, "lengthwise"),
    ];
    let mut cut_meshes: Vec<Vec<(IndexedMesh, [f64; 3])>> = Vec::new();
    for (n, off, _) in cuts {
        let foot_cut = foot.solid().intersect(Solid::plane(n, off));
        let ms_cut: Vec<(IndexedMesh, [f64; 3])> = std::thread::scope(|sc| {
            let mut hs: Vec<_> = specs
                .iter()
                .map(|sp| {
                    let g = g.clone();
                    let (kind, cell, col) = (sp.kind, sp.cell.max(0.6), sp.color);
                    sc.spawn(move || (mesh_of(&g.solid(kind, Some((n, off))), cell), col))
                })
                .collect();
            let fc = foot_cut.clone();
            hs.push(sc.spawn(move || (mesh_of(&fc, 1.0), skin)));
            // The leg above the scan's top (the last's prism), lighter: no scan there.
            let gl = g.clone();
            let top = lm.leg_top - 1.0;
            let mut lb = g.part_bounds(PartKind::Shell);
            lb.min.z = top - 1.0;
            lb.max.z = g.z_trim_back + p.guard_above_cuff;
            let leg = Solid::user_fn(move |q: Point3<f64>| gl.d(q).max(top - q.z).max(n.dot(&q.coords) - off), lb);
            hs.push(sc.spawn(move || (mesh_of(&leg, 1.0), [0.93, 0.83, 0.77])));
            hs.into_iter().map(|h| h.join().unwrap()).collect()
        });
        cut_meshes.push(ms_cut);
    }
    for ((n, _, name), cm) in cuts.iter().zip(&cut_meshes) {
        let cl: Vec<Layer> = cm.iter().filter(|(m, _)| !m.faces.is_empty()).map(|(m, c)| Layer::solid(m, *c)).collect();
        let view = View::looking(-n, Vector3::z());
        render_styled(&cl, &[view], 3.0, &out.join(format!("boot_section_{name}.png")), pretty).map_err(e)?;
    }
    println!("drew boot.png, boot_hero.png, boot_parts.png, boot_section_*.png ({:.1} s)", t.elapsed().as_secs_f64());

    // Light meshes for the 3D viewer: opt-in, since cf-design's mesh_simplified (adaptive
    // re-mesh, then decimation) took ~18 min for the ten parts on 12 threads.
    if std::env::var_os("BOOT_VIEWER").is_none() {
        println!("viewer meshes skipped (set BOOT_VIEWER=1 to build them, ~18 min)");
    } else {
        // Light meshes for the 3D viewer (viewer/boot.html), decimated by cf-design's
        // mesh_simplified; each part with its colour, weight and the way it moves when exploded.
        let t = Instant::now();
        let targets = [140_000usize, 25_000, 25_000, 70_000, 8_000, 15_000, 40_000, 12_000];
        let mut display: Vec<IndexedMesh> = std::thread::scope(|sc| {
            let hs: Vec<_> = specs
                .iter()
                .zip(targets)
                .map(|(sp, n)| {
                    let g = g.clone();
                    let (kind, cell) = (sp.kind, sp.cell);
                    sc.spawn(move || g.solid_bounded(kind).mesh_simplified(cell, n).geometry)
                })
                .collect();
            hs.into_iter().map(|h| h.join().unwrap()).collect()
        });
        display.push(merge(&grommets.iter().map(|s| s.mesh_simplified(0.3, 700).geometry).collect::<Vec<_>>()));
        display.push(laces.as_ref().map(|l| l.mesh_simplified(0.5, 30_000).geometry).unwrap_or_default());
        display.push(foot.mesh.clone());
        let lat = -lm.medial_side;
        let explode: [[f64; 3]; 11] = [
            [0.0, 0.0, 0.0],
            [70.0, 0.0, 0.0],
            [0.0, 0.0, -60.0],
            [0.0, lat * 150.0, 20.0],
            [0.0, lat * 150.0, 20.0],
            [0.0, lat * 150.0, -40.0],
            [80.0, 0.0, 90.0],
            [-70.0, 0.0, 40.0],
            [0.0, 0.0, 0.0],
            [130.0, 0.0, 130.0],
            [0.0, 0.0, 0.0],
        ];
        let mut names: Vec<(String, [f64; 3], f64, String)> =
            specs.iter().zip(&rows).map(|(sp, r)| (sp.name.to_string(), sp.color, r.3, sp.material.name.to_string())).collect();
        names.push(("eyelets".into(), [0.80, 0.81, 0.84], rows[8].3, AL_6061_T6.name.into()));
        names.push(("laces".into(), [0.96, 0.96, 0.94], rows[9].3, LACE.name.into()));
        names.push(("foot (stand-in scan)".into(), skin, 0.0, "".into()));
        let dir = out.join("viewer");
        std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
        let mut bin: Vec<u8> = Vec::new();
        let mut parts_json = Vec::new();
        let scale = 0.01; // mm per int16 step: +-327 mm
        for (i, m) in display.iter().enumerate() {
            let vo = bin.len();
            for v in &m.vertices {
                for c in [v.x, v.y, v.z] {
                    bin.extend_from_slice(&((c / scale).round().clamp(-32767.0, 32767.0) as i16).to_le_bytes());
                }
            }
            while !bin.len().is_multiple_of(4) {
                bin.push(0);
            }
            let io = bin.len();
            for f in &m.faces {
                for ix in f {
                    bin.extend_from_slice(&ix.to_le_bytes());
                }
            }
            let (name, col, grams, mat) = &names[i];
            let e = explode[i];
            parts_json.push(format!(
                "{{\"name\":\"{name}\",\"color\":[{:.3},{:.3},{:.3}],\"grams\":{grams:.1},\"material\":\"{mat}\",\"explode\":[{},{},{}],\"vertexOffset\":{vo},\"vertexCount\":{},\"indexOffset\":{io},\"indexCount\":{}}}",
                col[0],
                col[1],
                col[2],
                e[0],
                e[1],
                e[2],
                m.vertices.len(),
                m.faces.len() * 3
            ));
        }
        let manifest = format!(
            "{{\"units\":\"mm\",\"scale\":{scale},\"up\":\"z\",\"shellCarbonGrams\":{shell_cf:.1},\"medialSide\":{},\"cuts\":{{\"ankleX\":{:.1},\"legY\":{:.1}}},\"totalGrams\":{total:.1},\"parts\":[\n{}\n]}}\n",
            lm.medial_side,
            0.5 * (lm.medial.x + lm.lateral.x),
            lm.leg_c.1,
            parts_json.join(",\n")
        );
        // Base64 text: the artifact host serves text and images, not raw binary.
        std::fs::write(dir.join("boot.txt"), base64(&bin)).map_err(|e| e.to_string())?;
        std::fs::write(dir.join("manifest.json"), &manifest).map_err(|e| e.to_string())?;
        std::fs::write(dir.join("index.html"), include_str!("../viewer/boot.html")).map_err(|e| e.to_string())?;
        println!(
            "viewer: {} triangles in {:.1} MB -> {} ({:.1} s)",
            display.iter().map(|m| m.faces.len()).sum::<usize>(),
            bin.len() as f64 / 1e6,
            dir.display(),
            t.elapsed().as_secs_f64()
        );
    }
    std::fs::write(out.join("summary.txt"), &summary).map_err(|e| e.to_string())?;
    println!("wrote STL + PNG + summary.txt to {} ({:.1} s total)", out.display(), t0.elapsed().as_secs_f64());
    Ok(())
}
