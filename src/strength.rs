//! Strength by hand formulas (round 2): the load cases, section properties sampled from the
//! real cf-design solids, and the beam, contact, axle and roll checks. CortenForge has no
//! linear-elastic solver (docs/CORTENFORGE.md, Suggestions), so these come first; FEM next.
//!
//! What the formulas assume, so the numbers are read right:
//! - Beams: Euler-Bernoulli bending of the whole cross-section (`section`), stress at the
//!   extreme fibre, no stress concentration factor. Truss members get a local check between
//!   their nodes. Lateral bending, torsion and buckling are not covered here.
//! - The foot load on the upper is two point loads, heel and ball, placed so their resultant
//!   sits over the rocker contact (no spring torque from the coupling): the static balance of a
//!   free rocker. Point loads give the exact moment at the contact for any patch whose load is
//!   all on one side of it.
//! - The lower frame is rigid on four equal wheel springs, so the wheel loads are linear in x.
//!   That puts the largest moment at the centre (38 P mm for a 76 mm wheel pitch), an upper
//!   bound: a flexible frame sends more load to the middle wheels and bends less there.

use cortenforge::cf_design::Solid;
use nalgebra::Point3;

use crate::materials::Strength;

pub const G: f64 = 9.81;

/// The design loads. Every factor here is an assumption until a measured or standard load
/// replaces it (docs/RESEARCH.md, "Still to find"):
/// - `body_mass`: 100 kg, the top of EN 13843 class A (inline skates for users of 20-100 kg).
/// - `stride`: 1.5 x body weight on one skate, repeated (fatigue). The Brock / George Brown
///   poster's in-skate peaks reach 1196 N on ice (agility turns, fixed holder) for collegiate
///   players; insoles miss shear. In speed skating, a single-skater study cited by van der Kruk
///   et al. (2016, Sports Engineering 19:273-281) measured peak lateral force of 0.3-0.6 BW,
///   25-45% of the peak normal force, in the skate's own frame, which puts the normal peak near
///   1.3 BW.
/// - `impact`: 3 x body weight, once (static strength): a hop landing or a hard stop.
/// - `lateral`: lateral / normal force ratio in the skate's own frame, 0.45 (the upper end of
///   that speed-skating range; no hockey number found yet).
#[derive(Debug, Clone, Copy)]
pub struct Loads {
    pub body_mass: f64,
    pub stride: f64,
    pub impact: f64,
    pub lateral: f64,
}

pub const LOADS: Loads = Loads { body_mass: 100.0, stride: 1.5, impact: 3.0, lateral: 0.45 };

impl Loads {
    pub fn bw(&self) -> f64 {
        self.body_mass * G
    }
}

/// Allowable stresses (MPa): static = yield / 1.5; fatigue = a pulsating (R = 0) stress range
/// from the R = -1 fatigue strength by Goodman, knocked down 0.7 for surface and size, for the
/// peak of the stride load.
pub fn allowables(m: &Strength) -> (f64, f64) {
    let stat = m.yield_ / 1.5;
    // R = 0: amplitude = mean = s/2; Goodman: s/2 = Se (1 - (s/2)/Su) -> s = 2 Se Su / (Su + Se).
    let s_pulsating = 2.0 * m.fatigue * m.uts / (m.uts + m.fatigue);
    (stat, 0.7 * s_pulsating)
}

/// Cross-section properties for bending about the horizontal (y) axis, mm units.
#[derive(Debug, Clone, Copy)]
pub struct Section {
    pub area: f64,
    pub zc: f64,
    pub i: f64,
    pub z_min: f64,
    pub z_max: f64,
}

impl Section {
    /// Distance from the neutral axis to the extreme fibre.
    pub fn c(&self) -> f64 {
        (self.z_max - self.zc).max(self.zc - self.z_min)
    }
}

/// The section of `solid` cut by the plane x = `x`, by sampling its field on a y-z grid with
/// the same partial-cell weight as cf-design's `mass_properties` (clamp(0.5 - d / cell)). An
/// optional z window keeps only cells whose centres fall inside it.
pub fn section(solid: &Solid, x: f64, cell: f64, z_window: Option<(f64, f64)>) -> Option<Section> {
    let b = solid.bounds()?;
    let (mut z0, mut z1) = (b.min.z - cell, b.max.z + cell);
    if let Some((lo, hi)) = z_window {
        z0 = z0.max(lo);
        z1 = z1.min(hi);
    }
    let (y0, y1) = (b.min.y - cell, b.max.y + cell);
    let ny = ((y1 - y0) / cell).ceil() as usize;
    let nz = ((z1 - z0) / cell).ceil() as usize;
    let (mut a, mut sz, mut szz) = (0.0, 0.0, 0.0);
    let (mut zmin, mut zmax) = (f64::INFINITY, f64::NEG_INFINITY);
    let cell2 = cell * cell;
    for iz in 0..nz {
        let z = z0 + (iz as f64 + 0.5) * cell;
        for iy in 0..ny {
            let y = y0 + (iy as f64 + 0.5) * cell;
            let d = solid.evaluate(&Point3::new(x, y, z));
            let w = (0.5 - d / cell).clamp(0.0, 1.0);
            if w > 0.0 {
                let da = w * cell2;
                a += da;
                sz += da * z;
                szz += da * (z * z + cell2 / 12.0);
                if w >= 0.5 {
                    zmin = zmin.min(z - 0.5 * cell);
                    zmax = zmax.max(z + 0.5 * cell);
                }
            }
        }
    }
    if a <= 0.0 {
        return None;
    }
    let zc = sz / a;
    Some(Section { area: a, zc, i: szz - a * zc * zc, z_min: zmin, z_max: zmax })
}

/// Bending moment in the upper at x (N mm; positive = hogging, top in tension): foot load `p`
/// (N) split between a heel point `xh` and a ball point `xb` so it balances over the contact `x_c`.
pub fn upper_moment(x: f64, x_c: f64, p: f64, xh: f64, xb: f64) -> f64 {
    let ph = p * (xb - x_c) / (xb - xh);
    let pb = p - ph;
    if x <= x_c {
        if x <= xh { 0.0 } else { ph * (x - xh) }
    } else if x >= xb {
        0.0
    } else {
        pb * (xb - x)
    }
}

/// Wheel loads (N) of a rigid frame on four equal wheel springs under `p` at `x_c`.
pub fn wheel_reactions(axle_x: &[f64; 4], x_c: f64, p: f64) -> [f64; 4] {
    let m = axle_x.iter().sum::<f64>() / 4.0;
    let sxx: f64 = axle_x.iter().map(|a| (a - m) * (a - m)).sum();
    let mut r = [0.0; 4];
    for (ri, a) in r.iter_mut().zip(axle_x) {
        *ri = p / 4.0 + p * (x_c - m) * (a - m) / sxx;
    }
    r
}

/// Bending moment in the lower frame at x (N mm; positive = sagging, bottom in tension).
pub fn lower_moment(x: f64, axle_x: &[f64; 4], x_c: f64, p: f64) -> f64 {
    let r = wheel_reactions(axle_x, x_c, p);
    let mut m = 0.0;
    for (ri, a) in r.iter().zip(axle_x) {
        if *a < x {
            m += ri * (x - a);
        }
    }
    if x > x_c {
        m -= p * (x - x_c);
    }
    m
}

/// How far the contact can move from the centre with every wheel still loaded (rigid frame,
/// equal wheels): the end wheel unloads at e = sum (a - m)^2 / (4 max |a - m|).
pub fn all_wheels_down(axle_x: &[f64; 4]) -> f64 {
    let m = axle_x.iter().sum::<f64>() / 4.0;
    let sxx: f64 = axle_x.iter().map(|a| (a - m) * (a - m)).sum();
    let amax = axle_x.iter().map(|a| (a - m).abs()).fold(0.0, f64::max);
    sxx / (4.0 * amax)
}

/// Hertz line contact of a cylinder (radius `r`) on a flat of the same material, load `p` over
/// length `len`: contact half-width b (mm) and peak pressure p0 (MPa). Johnson, Contact
/// Mechanics, 4.2: b^2 = 4 p r / (pi len E*), p0 = 2 p / (pi b len), E* = E / (2 (1 - nu^2)).
pub fn hertz_line(p: f64, r: f64, len: f64, m: &Strength) -> (f64, f64) {
    let e_star = m.e / (2.0 * (1.0 - m.nu * m.nu));
    let b = (4.0 * p * r / (std::f64::consts::PI * len * e_star)).sqrt();
    (b, 2.0 * p / (std::f64::consts::PI * b * len))
}

/// Bending stress (MPa) in an axle carrying wheel load `w` (N) through two bearings at
/// +-`bearing_y`, supported by the rails at +-`support_y` (mm): M = w/2 (support_y - bearing_y).
pub fn axle_stress(w: f64, support_y: f64, bearing_y: f64, d: f64) -> f64 {
    let m = 0.5 * w * (support_y - bearing_y);
    32.0 * m / (std::f64::consts::PI * d * d * d)
}

/// The lateral / vertical ratio at which one rocker track would unload: the vertical load
/// alone holds the roll moment while lateral x height < vertical x half the track spacing.
pub fn roll_ratio(track_spacing: f64, track_height: f64) -> f64 {
    track_spacing / (2.0 * track_height)
}

/// The lean angle (deg) at which a point (|y|, z above the ground at rest) touches the ground,
/// for a wheel whose cross-section is a circle of radius `rho` (its tread) on a wheel of radius
/// `big_r`: the ground stays tangent to the tread circle as the skate leans.
pub fn lean_limit(y: f64, z: f64, big_r: f64, rho: f64) -> f64 {
    let a = y.abs();
    let b = z - (big_r - rho);
    let n = a.hypot(b);
    if n <= rho {
        return 0.0;
    }
    ((-rho / n).acos() - a.atan2(b)).to_degrees()
}
