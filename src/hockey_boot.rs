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
//! - `liner_in` .. `inner_off`: the liner. `liner_in` is the fit allowance (`ease`), plus, above
//!   the ankle bones, the cuff's flare: the liner and shell flare out together, leaving room
//!   between the leg and the liner at the top for the ankle to flex (Jon: tight from heel to toe,
//!   open at the top; a little room in front, a slight press at the back to pitch you forward).
//!   The liner thickens round the ankle bones (the ankle pads, in pockets the shell bulges out
//!   for). Under the foot it is the footbed. Under the throat it is the tongue's padding instead.
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
    /// The shell's outer surface is built on the base closed by this much more (hollows up to
    /// about twice this wide filled), above the sole: a taut outside; the inside still follows
    /// the foot. The shell is thicker over the hollows.
    pub fair_closing: f64,
    /// Outsole under the shell.
    pub sole: f64,
    /// The outsole sits this far inside the shell's outline, tucked under its rounded bottom edge.
    pub sole_inset: f64,
    /// The sole is only two flat mount pads, heel and forefoot, where the holder or chassis
    /// attaches (Jon, 2026-11-11: "we only need that flat piece where the skate meets the blade
    /// holder"): centred on the chassis's mounts (`sizing::MOUNT_SPACING` apart about
    /// `FrameParams::center`), `pad_len` long and `pad_w` wide. Estimates until the holder is
    /// designed and the TF9's rivet pattern measured.
    pub pad_len: f64,
    pub pad_w: f64,
    /// The cuff's top at the sides: this far above the outer (lateral) ankle bone, just above
    /// the ankle pocket (Jon, 2026-11-11: skates "usually go just above that exterior ankle
    /// bulge. to protect the ankles just enough, but the offer max mobility"). Estimate.
    pub cuff_above_ankle: f64,
    /// The cuff's top line, round the leg: low at the sides (just above the ankle bone), rising
    /// this much toward the front (the eyelet panels carry the lacing up the shin) and
    /// `trim_back_rise` toward the back (into the tendon guard). Estimates.
    pub trim_front_rise: f64,
    pub trim_back_rise: f64,
    /// The heel-lock eyelets (Jon, 2026-11-11: "the most important one, to lock in heel
    /// pressure"): the pair nearest the ankle crease, heavy duty: barrel radius `lock_r`, on a
    /// boss `lock_boss` mm proud of the eyelet facing.
    pub lock_r: f64,
    pub lock_boss: f64,
    /// The padded collar: the liner rises this far above the shell's top edge and covers it.
    pub collar: f64,
    /// Forward lean of the cuff, degrees: above the ankle bones everything is sheared forward (a
    /// shear keeps volumes and horizontal sections). Estimate (MEASURE.md B7).
    pub lean_deg: f64,
    /// The cuff's flare at its top edge, mm: how far the liner and shell stand off the leg there,
    /// on top of the fit allowance. At the sides (both the same), at the front, and at the back
    /// (negative: the liner presses the calf, pitching the leg forward). In between, blended by
    /// the direction round the leg. Estimates, to tune with Jon.
    pub flare_side: f64,
    pub flare_front: f64,
    pub flare_back: f64,
    /// The flare starts this far above the higher ankle bone and grows (as t^1.5) to the top.
    pub flare_from: f64,
    /// Ankle pockets: the shell bulges out (and the liner thickens) round each ankle bone by up
    /// to this much, Gaussian with `ankle_sigma`. Estimate.
    pub ankle_pocket: f64,
    pub ankle_sigma: f64,
    /// Ankle pads: the liner within this radius of an ankle bone.
    pub ankle_pad_r: f64,
    /// The throat starts here, as a fraction of the foot's length (round 1's throat ended at 0.70).
    pub throat_front: f64,
    /// Throat gap between the quarters' edges: at the front, and at the cuff top. It widens as
    /// t^`throat_splay` along the throat, so the upper eyelets swing out into a V: room for the
    /// shin to drive forward (Jon, 2026-11-11, from a skate whose upper eyelets splay out).
    /// Estimates.
    pub throat_w0: f64,
    pub throat_w1: f64,
    pub throat_splay: f64,
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
    /// How far the tongue's top flares out (forward) beyond its thickness, growing over the part
    /// above the cuff. Estimate.
    pub tongue_curl: f64,
    /// Tendon guard: height above the cuff at the back, thickness, how far its top kicks back
    /// off the leg. Estimate.
    pub guard_above_cuff: f64,
    pub guard_t: f64,
    pub guard_kick: f64,
    /// The tendon guard is a shield (Jon, 2026-11-11, from two pro skates' guards): `guard_w0`
    /// wide where it rivets over the shell's back, flaring to `guard_w1` at the top, whose edge
    /// has rounded shoulders and a `guard_notch` dip in the middle; a raised rim and a centre
    /// spine `guard_rib` proud (stiffness, and the look). Estimates.
    pub guard_w0: f64,
    pub guard_w1: f64,
    pub guard_notch: f64,
    pub guard_rib: f64,
    /// Toe cap thickness over the shell.
    pub toe_cap_t: f64,
    /// Lace radius (drawn round).
    pub lace_r: f64,
    /// Vents: slots through the toe cap, the quarters and the liner. The roller boot is vented,
    /// the ice boot sealed (Jon, 2026-11-11: cold rink, ice gets in and melts).
    pub vented: bool,
}

impl Default for HockeyParams {
    fn default() -> Self {
        Self {
            ease: 1.0,
            liner: 4.0,
            wall: 2.5,
            fair_closing: 18.0,
            sole: 3.0,
            sole_inset: 2.0,
            pad_len: 46.0,
            pad_w: 48.0,
            cuff_above_ankle: 40.0,
            trim_front_rise: 28.0,
            trim_back_rise: 18.0,
            lock_r: 4.2,
            lock_boss: 1.5,
            collar: 4.0,
            lean_deg: 6.0,
            flare_side: 12.0,
            flare_front: 6.0,
            flare_back: -2.0,
            flare_from: 5.0,
            ankle_pocket: 3.0,
            ankle_sigma: 12.0,
            ankle_pad_r: 24.0,
            throat_front: 0.70,
            throat_w0: 26.0,
            throat_w1: 48.0,
            throat_splay: 2.2,
            reach: 32.0,
            facing_w: 18.0,
            facing_t: 1.5,
            eyelet_spacing: 19.0,
            eyelet_inset: 9.0,
            tongue_overlap: 10.0,
            tongue_above_cuff: 40.0,
            tongue_proud: 1.5,
            tongue_curl: 8.0,
            guard_above_cuff: 70.0,
            guard_t: 2.5,
            guard_kick: 20.0,
            guard_w0: 34.0,
            guard_w1: 64.0,
            guard_notch: 4.0,
            guard_rib: 1.0,
            toe_cap_t: 1.5,
            lace_r: 1.6,
            vented: false,
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

/// Smooth maximum (polynomial, blend width `k`): rounds the corner where two fields meet.
fn smax(a: f64, b: f64, k: f64) -> f64 {
    let h = (0.5 + 0.5 * (a - b) / k).clamp(0.0, 1.0);
    b + (a - b) * h + k * h * (1.0 - h)
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
    /// Diagram colour (the sections, where the layers must read apart).
    pub color: [f64; 3],
    /// The look (Jon, 2026-11-11: black carbon against black thermoplastic and leather, a black
    /// tongue), and its finish (the viewer's material; the renders' sheen).
    pub look: [f64; 3],
    pub finish: &'static str,
    pub cell: f64,
}

/// The eyelets (gunmetal) and laces (white hockey laces: Jon, 2026-11-11) in the look.
pub const EYELET_LOOK: [f64; 3] = [0.42, 0.43, 0.46];
pub const LACE_LOOK: [f64; 3] = [0.94, 0.94, 0.92];

/// Sheen in the renders, by finish.
pub fn finish_spec(finish: &str) -> f64 {
    match finish {
        "carbon" => 0.55,
        "gloss" => 0.6,
        "metal" => 0.8,
        "leather" => 0.3,
        "matte" => 0.12,
        _ => 0.04,
    }
}

pub fn part_specs() -> Vec<PartSpec> {
    use PartKind::*;
    vec![
        PartSpec { kind: Shell, name: "shell", material: PA12_MJF, color: [0.16, 0.25, 0.42], look: [0.16, 0.16, 0.18], finish: "carbon", cell: 0.75 },
        PartSpec { kind: ToeCap, name: "toe cap", material: PA12_MJF, color: [0.13, 0.13, 0.15], look: [0.09, 0.09, 0.10], finish: "gloss", cell: 0.3 },
        PartSpec { kind: Outsole, name: "mount pads", material: PA12_MJF, color: [0.09, 0.09, 0.10], look: [0.07, 0.07, 0.075], finish: "matte", cell: 0.75 },
        PartSpec { kind: Liner, name: "liner", material: LINER_FOAM, color: [0.42, 0.44, 0.48], look: [0.23, 0.23, 0.25], finish: "fabric", cell: 1.0 },
        PartSpec { kind: AnklePads, name: "ankle pads", material: LINER_FOAM, color: [0.78, 0.36, 0.22], look: [0.27, 0.27, 0.29], finish: "fabric", cell: 1.0 },
        PartSpec { kind: Footbed, name: "footbed", material: LINER_FOAM, color: [0.22, 0.55, 0.45], look: [0.19, 0.19, 0.21], finish: "fabric", cell: 1.0 },
        PartSpec { kind: Tongue, name: "tongue", material: LINER_FOAM, color: [0.90, 0.90, 0.87], look: [0.12, 0.12, 0.13], finish: "leather", cell: 1.0 },
        PartSpec { kind: TendonGuard, name: "tendon guard", material: TPU, color: [0.22, 0.22, 0.25], look: [0.10, 0.10, 0.11], finish: "gloss", cell: 0.6 },
    ]
}

/// A vent: a slot through the toe cap or shell and the liner, centred on the outer surface at
/// `c`, cut along the normal `n`, long along `t`.
#[derive(Debug, Clone, Copy)]
pub struct Vent {
    pub c: Point3<f64>,
    pub n: Vector3<f64>,
    pub t: Vector3<f64>,
    pub half_len: f64,
    pub r: f64,
}

/// The boot's shared geometry: the last's base with the leg extended, the landmarks, the dorsal
/// line and the stations along it, the eyelets and the lace path. Built upright; the forward
/// lean is applied when a part is made a solid (`solid`, `leaned`).
pub struct BootGeom {
    pub p: HockeyParams,
    pub lm: Landmarks,
    base: Solid,
    /// The base with its hollows filled more (`fair_closing`), for the shell's outer surface.
    base_fair: Solid,
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
    /// The mount pads' centres (x, y): heel and forefoot.
    pub mounts: [(f64, f64); 2],
    /// Eyelet centres on the facing's outer surface and their outward axes, in pairs from the
    /// toe end: [2k] on the -y side, [2k + 1] on the +y side.
    pub eyelets: Vec<(Point3<f64>, Vector3<f64>)>,
    /// The lace: the criss-cross with its knot, then the bow's loops and tails.
    pub lace_paths: Vec<Vec<Point3<f64>>>,
    /// Vents (the roller boot).
    pub vents: Vec<Vent>,
    /// The heel-lock eyelet pair (level from the toe end), and the ankle crease's height.
    pub lock: Option<usize>,
    pub z_crease: f64,
    /// The ankle crease on the dorsal line.
    pub crease: Point3<f64>,
    /// The tongue's top, z.
    pub z_tongue_top: f64,
    pub bounds: Aabb,
    /// The outsole's bottom (the chassis or holder mounts here).
    pub sole_bottom: f64,
}

impl BootGeom {
    pub fn new(base: Solid, base_fair: Solid, base_bounds: Aabb, lm: Landmarks, p: HockeyParams) -> Self {
        let z_ext = lm.leg_top - 15.0;
        let z_cuff = lm.lateral.z + p.cuff_above_ankle;
        let z_trim_back = z_cuff + p.trim_back_rise;
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
            base_fair,
            z_ext,
            prof: Profile::new(vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)]),
            s_front: 0.0,
            s_cuff: 0.0,
            s_tongue_top: 0.0,
            z_cuff,
            z_trim_back,
            toe_cap_x: 0.0,
            toe_cap_y: 0.0,
            mounts: [(0.0, 0.0); 2],
            eyelets: Vec::new(),
            lace_paths: Vec::new(),
            vents: Vec::new(),
            lock: None,
            z_crease: 0.0,
            crease: Point3::origin(),
            z_tongue_top: 0.0,
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
        g.s_cuff = g.prof.find(|q| q.z >= g.z_trim(*q)).unwrap_or(g.prof.len());
        g.s_tongue_top = g.prof.find(|q| q.z >= g.z_trim(*q) + p.tongue_above_cuff).unwrap_or(g.prof.len());
        g.z_tongue_top = g.prof.at(g.s_tongue_top).0.z;
        let (front, _) = g.prof.at(g.s_front);
        g.toe_cap_x = front.x + 2.0;
        g.toe_cap_y = front.y;
        // The mounts: the chassis's, centred across the footprint there (at z 3).
        let xc = crate::frame::FrameParams::default().center * lm.length;
        for (i, side) in [-1.0, 1.0].into_iter().enumerate() {
            let x = xc + side * 0.5 * crate::sizing::MOUNT_SPACING;
            let edge = |dir: f64| {
                let mut y = 0.0;
                while g.d(Point3::new(x, y + dir * 0.5, 3.0)) < 0.0 && y.abs() < 150.0 {
                    y += dir * 0.5;
                }
                y
            };
            g.mounts[i] = (x, 0.5 * (edge(-1.0) + edge(1.0)));
        }
        g.place_eyelets();
        g.place_lock();
        g.place_vents();
        g.lace();
        g
    }

    /// The last's base, the leg carried up as a prism above `z_ext`.
    pub fn d(&self, q: Point3<f64>) -> f64 {
        self.base.evaluate(&Point3::new(q.x, q.y, q.z.min(self.z_ext)))
    }

    /// The field the shell's outer surface is built on: the fair base above the sole (blended in
    /// from 5 to 25 mm up, so the arch isn't filled with shell), never inside the base.
    pub fn d_out(&self, q: Point3<f64>) -> f64 {
        let d = self.d(q);
        let w = smoothstep(5.0, 25.0, q.z);
        if w <= 0.0 {
            return d;
        }
        let f = self.base_fair.evaluate(&Point3::new(q.x, q.y, q.z.min(self.z_ext)));
        d + w * (f.min(d) - d)
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

    /// The cuff's top line at a point (upright frame): `z_cuff` at the sides, higher by
    /// `trim_front_rise` at the front and by `trim_back_rise` at the back, blended by the
    /// direction round the leg (cos^2).
    pub fn z_trim(&self, q: Point3<f64>) -> f64 {
        let (dx, dy) = (q.x - self.lm.leg_c.0, q.y - self.lm.leg_c.1);
        let c2 = dx * dx / (dx * dx + dy * dy).max(1e-18);
        self.z_cuff + c2 * if dx > 0.0 { self.p.trim_front_rise } else { self.p.trim_back_rise }
    }

    /// The forward lean as a shear: above the ankle bones x moves forward by this much (ramped in
    /// from 10 mm below to 30 mm above their height, so there's no kink).
    pub fn lean_dx(&self, z: f64) -> f64 {
        let zp = 0.5 * (self.lm.medial.z + self.lm.lateral.z);
        smoothstep(zp - 10.0, zp + 30.0, z) * (z - zp) * self.p.lean_deg.to_radians().tan()
    }

    pub fn lean(&self, q: Point3<f64>) -> Point3<f64> {
        Point3::new(q.x + self.lean_dx(q.z), q.y, q.z)
    }

    pub fn unlean(&self, q: Point3<f64>) -> Point3<f64> {
        Point3::new(q.x - self.lean_dx(q.z), q.y, q.z)
    }

    /// A box in the upright frame, widened to hold its leaned image.
    pub fn leaned_bounds(&self, b: Aabb) -> Aabb {
        let (mut lo, mut hi) = (0.0f64, 0.0f64);
        let mut z = b.min.z;
        while z <= b.max.z + 1.0 {
            let dx = self.lean_dx(z.min(b.max.z));
            lo = lo.min(dx);
            hi = hi.max(dx);
            z += 1.0;
        }
        Aabb::new(Point3::new(b.min.x + lo - 0.5, b.min.y, b.min.z), Point3::new(b.max.x + hi + 0.5, b.max.y, b.max.z))
    }

    /// A solid built upright, leaned (grommets, laces, the foot).
    pub fn leaned(self: &Arc<Self>, s: Solid, bounds: Aabb) -> Solid {
        let g = self.clone();
        let lb = self.leaned_bounds(bounds);
        Solid::user_fn(move |q: Point3<f64>| s.evaluate(&g.unlean(q)), lb)
    }

    /// A mesh built upright, leaned.
    pub fn lean_mesh(&self, m: &IndexedMesh) -> IndexedMesh {
        let mut out = m.clone();
        for v in &mut out.vertices {
            *v = self.lean(*v);
        }
        out
    }

    /// The cuff's flare: grows (as t^1.5, so it opens soon above the bones) from `flare_from`
    /// above the higher ankle bone to the cuff's top; `flare_side` at the sides, `flare_front`
    /// in front, `flare_back` behind, blended by the direction round the leg (cos^2, sin^2).
    fn flare(&self, q: Point3<f64>) -> f64 {
        let z0 = self.lm.medial.z.max(self.lm.lateral.z) + self.p.flare_from;
        let t = ((q.z - z0) / (self.z_trim(q) - z0)).clamp(0.0, 1.0);
        let (dx, dy) = (q.x - self.lm.leg_c.0, q.y - self.lm.leg_c.1);
        let r2 = (dx * dx + dy * dy).max(1e-18);
        let (c2, s2) = (dx * dx / r2, dy * dy / r2);
        let fb = if dx > 0.0 { self.p.flare_front } else { self.p.flare_back };
        t.powf(1.5) * (self.p.flare_side * s2 + fb * c2)
    }

    /// Where the liner starts: the fit allowance plus the flare (offset from the base).
    pub fn liner_in(&self, q: Point3<f64>) -> f64 {
        self.p.ease + self.flare(q)
    }

    fn pocket(&self, q: Point3<f64>) -> f64 {
        let s2 = 2.0 * self.p.ankle_sigma * self.p.ankle_sigma;
        [self.lm.medial, self.lm.lateral].iter().map(|m| self.p.ankle_pocket * (-(q - m).norm_squared() / s2).exp()).sum()
    }

    /// Where the liner ends and the shell begins (offset from the base).
    pub fn inner_off(&self, q: Point3<f64>) -> f64 {
        self.liner_in(q) + self.p.liner + self.pocket(q)
    }

    fn throat_w(&self, s: f64) -> f64 {
        let t = ((s - self.s_front) / (self.s_cuff - self.s_front).max(1.0)).clamp(0.0, 1.0);
        self.p.throat_w0 + (self.p.throat_w1 - self.p.throat_w0) * t.powf(self.p.throat_splay)
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

    /// The tendon guard's outline in (y, z) on the back of the leg: a shield, narrow at the
    /// bottom, flaring (as v^1.5) to the top, the top edge dipping `guard_notch` in the middle,
    /// the corners rounded (smooth max). Negative inside.
    fn guard_outline(&self, q: Point3<f64>) -> f64 {
        let p = &self.p;
        let (z0, z1) = (self.z_trim_back - 30.0, self.z_trim_back + p.guard_above_cuff);
        let v = ((q.z - z0) / (z1 - z0)).clamp(0.0, 1.0);
        let a = (q.y - self.lm.leg_c.1).abs();
        let hw = 0.5 * p.guard_w0 + 0.5 * (p.guard_w1 - p.guard_w0) * v.powf(1.5);
        let side = a - hw;
        let top = q.z - (z1 - p.guard_notch * (1.0 - smoothstep(0.0, 14.0, a)));
        let bottom = z0 - q.z;
        smax(smax(side, top, 10.0), bottom, 5.0)
    }

    /// The tendon guard's band: from the shell's outer surface, kicking back at the top, with a
    /// raised rim and centre spine.
    fn guard(&self, q: Point3<f64>, d: f64) -> f64 {
        let p = &self.p;
        let (zb, z1) = (self.z_trim_back, self.z_trim_back + p.guard_above_cuff);
        let u = ((q.z - zb) / (z1 - zb)).clamp(0.0, 1.0);
        let off = self.inner_off(q) + p.wall + p.guard_kick * u * u;
        let outline = self.guard_outline(q);
        let rim = 1.0 - smoothstep(-4.5, -3.0, outline);
        let spine = (1.0 - smoothstep(2.0, 3.5, (q.y - self.lm.leg_c.1).abs())) * smoothstep(zb - 20.0, zb - 5.0, q.z);
        let t = p.guard_t + p.guard_rib * rim.max(spine);
        let band = (d - (off + t)).max(off - d);
        let back = q.x - (self.lm.leg_c.0 - 10.0);
        band.max(outline).max(back)
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
        let li = self.liner_in(q);
        let zt = self.z_trim(q);
        match k {
            PartKind::Shell => {
                let n = self.prof.closest(q.x, q.z);
                let th = self.throat(&n, q.y);
                let mut oo = io + p.wall + self.facing(&n, th);
                if let Some(k) = self.lock {
                    for i in [2 * k, 2 * k + 1] {
                        let r = (q - self.eyelets[i].0).norm();
                        oo += p.lock_boss * (1.0 - smoothstep(8.0, 11.0, r));
                    }
                }
                let mut s = (self.d_out(q) - oo).max(io - d).max(q.z - zt);
                if s > 3.0 {
                    return s;
                }
                s = s.max(-(th.max(n.d - p.reach)));
                for (i, (c, a)) in self.eyelets.iter().enumerate() {
                    let v = q - c;
                    if v.norm_squared() < 196.0 {
                        let along = v.dot(a);
                        let r = (v - a * along).norm();
                        s = s.max(-((r - self.grommet_size(i).0).max(along.abs() - 9.0)));
                    }
                }
                s.max(-self.vent_cut(q))
            }
            PartKind::ToeCap => {
                let n = self.prof.closest(q.x, q.z);
                let oo = self.outer_off(q, &n);
                let dout = self.d_out(q);
                let band = (dout - (oo + p.toe_cap_t)).max(oo - dout);
                let edge = self.toe_cap_x + 25.0 * ((q.y - self.toe_cap_y) / 45.0).powi(2) - q.x;
                let low = -(p.ease + p.liner + p.wall) - q.z;
                let throat = self.throat(&n, q.y).max(n.d - p.reach);
                band.max(edge).max(low).max(-throat).max(-self.vent_cut(q))
            }
            PartKind::Outsole => {
                // Two flat pads under the shell, filling up to its rounded underside.
                let oo_c = p.ease + p.liner + p.wall;
                let outline = self.d_out(Point3::new(q.x, q.y, 3.0)) - (oo_c - p.sole_inset);
                let pads = self
                    .mounts
                    .iter()
                    .map(|&(xm, ym)| rounded_rect(q.x - xm, q.y - ym, 0.5 * p.pad_len, 0.5 * p.pad_w, 8.0))
                    .fold(f64::MAX, f64::min);
                let slab = (self.sole_bottom - q.z).max(q.z - (-oo_c + 3.0));
                let n = self.prof.closest(q.x, q.z);
                let outside_shell = self.outer_off(q, &n) - self.d_out(q);
                pads.max(outline).max(slab).max(outside_shell)
            }
            PartKind::Liner | PartKind::AnklePads | PartKind::Footbed => {
                // Up to the shell's top edge the liner fills [li, io]; above it the collar rises
                // `collar` mm and covers the shell's edge (out to 1 mm past the shell).
                let band = (if q.z > zt { self.d_out(q) - (io + p.wall + 1.0) } else { d - io }).max(li - d).max(q.z - (zt + p.collar));
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
                        band.max(-pad).max(-below).max(-tongue).max(-self.vent_cut(q))
                    }
                }
            }
            PartKind::Tongue => {
                let n = self.prof.closest(q.x, q.z);
                let outline = self.tongue_outline(&n, q.y).max(n.d - p.reach);
                if outline > 3.0 {
                    return outline;
                }
                let pad = (d - io).max(li - d);
                let curl = p.tongue_curl * ((q.z - zt) / (self.z_tongue_top - zt).max(1.0)).clamp(0.0, 1.0).powi(2);
                let raised = (d - (io + p.wall + p.tongue_proud + curl)).max(li - d);
                let open = (self.throat(&n, q.y) + 1.0).min(zt + 0.5 - q.z);
                pad.min(raised.max(open)).max(outline)
            }
            PartKind::TendonGuard => self.guard(q, self.d_out(q)),
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
            PartKind::Outsole => mk(b.min.x, b.min.y, self.sole_bottom - 1.5, b.max.x, b.max.y, -oo_c + 4.0),
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
                let (yc, xc, hw) = (lm.leg_c.1, lm.leg_c.0, 0.5 * p.guard_w1 + 4.0);
                mk(b.min.x, yc - hw, self.z_trim_back - 33.0, xc - 8.0, yc + hw, self.z_trim_back + p.guard_above_cuff + 3.0)
            }
        }
    }

    /// One part as a solid, optionally cut to the half-space `n . x < off`.
    pub fn solid(self: &Arc<Self>, k: PartKind, cut: Option<(Vector3<f64>, f64)>) -> Solid {
        let g = self.clone();
        let b = self.leaned_bounds(self.part_bounds(k));
        Solid::user_fn(
            move |q: Point3<f64>| {
                let v = g.sdf(k, g.unlean(q));
                match cut {
                    Some((n, off)) => v.max(n.dot(&q.coords) - off),
                    None => v,
                }
            },
            b,
        )
    }

    /// The ankle crease: where the dorsal line, climbing from the instep, turns past 60 degrees
    /// from horizontal toward the shin. The eyelet pair nearest it is the heel lock: its lace
    /// pulls the heel back and down into the pocket. Moved out onto its boss.
    fn place_lock(&mut self) {
        let mut s = self.s_front;
        let mut crease = None;
        while s < self.prof.len() {
            let (pt, t) = self.prof.at(s);
            if t.z.atan2(-t.x).to_degrees() >= 60.0 {
                crease = Some((s, pt));
                break;
            }
            s += 0.5;
        }
        let Some((sc, pc)) = crease else { return };
        self.z_crease = pc.z;
        self.crease = pc;
        let n = self.eyelets.len() / 2;
        let k = (0..n)
            .min_by(|&a, &b| {
                let sa = self.prof.closest(self.eyelets[2 * a].0.x, self.eyelets[2 * a].0.z).s;
                let sb = self.prof.closest(self.eyelets[2 * b].0.x, self.eyelets[2 * b].0.z).s;
                (sa - sc).abs().total_cmp(&(sb - sc).abs())
            });
        if let Some(k) = k {
            self.lock = Some(k);
            for i in [2 * k, 2 * k + 1] {
                let (c, a) = self.eyelets[i];
                self.eyelets[i] = (c + a * self.p.lock_boss, a);
            }
        }
    }

    fn is_lock(&self, i: usize) -> bool {
        self.lock == Some(i / 2)
    }

    /// Barrel radius, flange radius, flange half-thickness of eyelet `i`.
    fn grommet_size(&self, i: usize) -> (f64, f64, f64) {
        if self.is_lock(i) { (self.p.lock_r, 6.8, 0.6) } else { (3.4, 5.0, 0.45) }
    }

    /// The vents' cut: negative inside a slot.
    fn vent_cut(&self, q: Point3<f64>) -> f64 {
        let mut m = f64::MAX;
        for v in &self.vents {
            let w = q - v.c;
            if w.norm_squared() > 400.0 {
                continue;
            }
            let a = w.dot(&v.n);
            let along = w.dot(&v.t);
            let side = (w - v.n * a - v.t * along).norm();
            m = m.min(((along.abs() - v.half_len).max(0.0).hypot(side) - v.r).max(a.abs() - 12.0));
        }
        m
    }

    /// The roller boot's vents: four slots across the toe box, and three a side low on the
    /// quarters (forefoot and midfoot), all lengthwise; the liner is cut under them.
    fn place_vents(&mut self) {
        if !self.p.vented {
            return;
        }
        let along_x = |n: Vector3<f64>| {
            let t = Vector3::x() - n * n.x;
            if t.norm() > 1e-6 { t.normalize() } else { Vector3::y() }
        };
        let x = self.toe_cap_x + 24.0;
        for k in 0..4 {
            let y = self.toe_cap_y + (k as f64 - 1.5) * 11.0;
            let mut z = 120.0;
            while z > 0.0 && self.sdf(PartKind::ToeCap, Point3::new(x, y, z)) > 0.0 {
                z -= 0.25;
            }
            if z > 0.0 {
                let c = Point3::new(x, y, z);
                let n = self.grad(c);
                self.vents.push(Vent { c, n, t: along_x(n), half_len: 6.0, r: 2.0 });
            }
        }
        for side in [-1.0, 1.0] {
            for f in [0.42, 0.50, 0.58] {
                let (x, z) = (f * self.lm.length, 24.0);
                let mut y = side * 120.0;
                while y * side > 0.0 && self.sdf(PartKind::Shell, Point3::new(x, y, z)) > 0.0 {
                    y -= side * 0.25;
                }
                if y * side > 0.0 {
                    let c = Point3::new(x, y, z);
                    let n = self.grad(c);
                    self.vents.push(Vent { c, n, t: along_x(n), half_len: 7.0, r: 2.2 });
                }
            }
        }
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
                    self.d_out(q) - self.outer_off(q, &near)
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
        // Outside the liner layer too: inside the foot every part's field reads "clear" (the
        // early-out in `sdf`), a chord between the splayed top eyelets runs ~12 mm inside the leg,
        // and the open cuff's room between the leg and the tongue would also read clear.
        for _ in 0..320 {
            let env = self.sdf(PartKind::Shell, q).min(self.sdf(PartKind::Tongue, q));
            if env >= r + 0.4 && self.d(q) > self.inner_off(q) {
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
            let i = 2 * k + side;
            let (c, a) = self.eyelets[i];
            c + a * (2.0 * self.grommet_size(i).2 + r)
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
        self.eyelets
            .iter()
            .enumerate()
            .map(|(i, (c, a))| {
                let (rb, rf, hf) = self.grommet_size(i);
                let depth = self.p.wall + self.p.facing_t + 0.4 + if self.is_lock(i) { self.p.lock_boss } else { 0.0 };
                let rot = UnitQuaternion::rotation_between(&Vector3::z(), a).unwrap_or_else(UnitQuaternion::identity);
                let barrel = Solid::cylinder(rb, 0.5 * depth).translate(Vector3::new(0.0, 0.0, -0.5 * depth));
                let flange = Solid::cylinder(rf, hf).translate(Vector3::new(0.0, 0.0, hf));
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

/// Display-only decimation of a closed mesh by vertex clustering: vertices are grouped by a
/// `cell` mm grid and by which of six directions their normal faces, so the two faces of a thin
/// wall never merge; each group becomes its mean, moved onto the surface by up to three Newton
/// steps on `sdf` if given (a step longer than a cell is refused). Triangles that collapse,
/// repeat, or turn over are dropped.
pub fn cluster_decimate(m: &IndexedMesh, cell: f64, sdf: Option<&(dyn Fn(Point3<f64>) -> f64 + Sync)>) -> IndexedMesh {
    use std::collections::{HashMap, HashSet};
    // Which way each vertex faces: the field's gradient if there is a field (averaged mesh
    // normals point the wrong way at a few vertices of a thin wall, ~0.3% on a 2 mm shell, and
    // those would merge the wall's two faces), else the area-weighted mesh normal.
    let grad = |f: &(dyn Fn(Point3<f64>) -> f64 + Sync), v: Point3<f64>, h: f64| {
        Vector3::new(
            f(v + Vector3::x() * h) - f(v - Vector3::x() * h),
            f(v + Vector3::y() * h) - f(v - Vector3::y() * h),
            f(v + Vector3::z() * h) - f(v - Vector3::z() * h),
        ) / (2.0 * h)
    };
    let normals: Vec<Vector3<f64>> = match sdf {
        Some(f) => m.vertices.iter().map(|v| grad(f, *v, 0.02 * cell)).collect(),
        None => {
            let mut n = vec![Vector3::zeros(); m.vertices.len()];
            for f in &m.faces {
                let [a, b, c] = f.map(|i| m.vertices[i as usize]);
                let x = (b - a).cross(&(c - a));
                for i in f {
                    n[*i as usize] += x;
                }
            }
            n
        }
    };
    let bucket = |n: &Vector3<f64>| -> u8 {
        let a = n.abs();
        let axis = if a.x >= a.y && a.x >= a.z { 0 } else if a.y >= a.z { 1 } else { 2 };
        axis * 2 + u8::from(n[axis as usize] < 0.0)
    };
    let mut ids: HashMap<(i64, i64, i64, u8), u32> = HashMap::new();
    let mut sums: Vec<(Vector3<f64>, f64)> = Vec::new();
    let map: Vec<u32> = m
        .vertices
        .iter()
        .zip(&normals)
        .map(|(v, n)| {
            let key = ((v.x / cell).floor() as i64, (v.y / cell).floor() as i64, (v.z / cell).floor() as i64, bucket(n));
            let id = *ids.entry(key).or_insert_with(|| {
                sums.push((Vector3::zeros(), 0.0));
                (sums.len() - 1) as u32
            });
            sums[id as usize].0 += v.coords;
            sums[id as usize].1 += 1.0;
            id
        })
        .collect();
    let mut vertices: Vec<Point3<f64>> = sums.iter().map(|(s, n)| Point3::from(s / *n)).collect();
    if let Some(f) = sdf {
        for v in &mut vertices {
            for _ in 0..3 {
                let d = f(*v);
                let g = grad(f, *v, 0.05 * cell);
                let step = g * (d / g.norm_squared().max(1e-12));
                if !step.norm().is_finite() || step.norm() > cell {
                    break;
                }
                *v -= step;
            }
        }
    }
    let mut seen = HashSet::new();
    let mut faces = Vec::new();
    for f in &m.faces {
        let [a, b, c] = f.map(|i| map[i as usize]);
        if a == b || b == c || a == c {
            continue;
        }
        let [pa, pb, pc] = f.map(|i| m.vertices[i as usize]);
        let before = (pb - pa).cross(&(pc - pa));
        let after = (vertices[b as usize] - vertices[a as usize]).cross(&(vertices[c as usize] - vertices[a as usize]));
        if before.dot(&after) <= 0.0 {
            continue;
        }
        let mut key = [a, b, c];
        key.sort_unstable();
        if seen.insert(key) {
            faces.push([a, b, c]);
        }
    }
    // Keep only the vertices some triangle uses.
    let mut remap = vec![u32::MAX; vertices.len()];
    let mut out = IndexedMesh::default();
    for f in &mut faces {
        for i in f.iter_mut() {
            if remap[*i as usize] == u32::MAX {
                remap[*i as usize] = out.vertices.len() as u32;
                out.vertices.push(vertices[*i as usize]);
            }
            *i = remap[*i as usize];
        }
    }
    out.faces = faces;
    out
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

/// Girth of a solid's section by the plane through `a` and `b` that contains the y direction
/// (across the foot), mm: rays fanned from the midpoint in the plane, 0.5 degree apart, each to
/// where `f` turns positive, the polygon's perimeter. Assumes the section is star-shaped about the
/// midpoint (true for a loop round the heel and instep). NaN if the midpoint is outside.
pub fn plane_girth(f: &dyn Fn(Point3<f64>) -> f64, a: Point3<f64>, b: Point3<f64>) -> f64 {
    let mut e2 = b - a;
    e2.y = 0.0;
    let e2 = e2.normalize();
    let c = Point3::from((a.coords + b.coords) * 0.5);
    if f(c) >= 0.0 {
        return f64::NAN;
    }
    let pts: Vec<Point3<f64>> = (0..720)
        .map(|i| {
            let th = i as f64 * std::f64::consts::PI / 360.0;
            let dir = Vector3::y() * th.cos() + e2 * th.sin();
            let mut t = 0.0;
            while t < 300.0 && f(c + dir * (t + 0.5)) < 0.0 {
                t += 0.5;
            }
            let (mut lo, mut hi) = (t, t + 0.5);
            for _ in 0..30 {
                let m = 0.5 * (lo + hi);
                if f(c + dir * m) < 0.0 {
                    lo = m;
                } else {
                    hi = m;
                }
            }
            c + dir * (0.5 * (lo + hi))
        })
        .collect();
    (0..pts.len()).map(|i| (pts[(i + 1) % pts.len()] - pts[i]).norm()).sum()
}

/// Taking the foot out (going in is the same path backwards), Jon, 2026-11-11: the heel locked by
/// the shell itself, tolerances as tight as they can be ("scott van horn appears to do it with
/// the boot around the heel ... way more responsive"), and the boot "almost rigid", unlike a shoe.
/// So the question is the tightest heel cup the foot still gets past.
///
/// The foot below the ankle bones is rigid, hinged at the ball (the toes stay flat as the heel
/// lifts). Poses: the heel lifted by `beta` about the ball, then the foot moved up `h` and back
/// `b` (upright frame: up along the leg). At each pose, a foot point's push is how far it gets
/// into the shell, past the liner squeezed to `liner_min`; points above the cuff's top line or
/// out through the open throat (laces loose, tongue pulled forward) are free.
///
/// The shell isn't equally rigid everywhere, so the pushes are kept by zone of the foot: the heel
/// (the rigid heel cup: only the heel's soft tissue gives), the midfoot (the quarters, which
/// spread when unlaced), the forefoot (the toe box: the forefoot compresses a little). The way
/// out is the path over the pose grid whose worst push, each zone's divided by its budget, is
/// least (a bottleneck search). The budgets only steer the path; the pushes are reported as is.
#[derive(Debug, Clone, Copy)]
pub struct Removal {
    /// Along the way out: the worst push in each zone (heel cup, quarters, toe box), mm, and where
    /// on the foot (foot frame).
    pub push: [f64; 3],
    pub at: [Point3<f64>; 3],
    /// The budgets that steered the path, mm (assumptions, to calibrate on test shells).
    pub budget: [f64; 3],
    /// The liner's squeezed thickness assumed, the foot points checked, the push in place (0: the
    /// foot fits its own boot), and the worst pose on the path (heel lift deg, up mm, back mm).
    pub liner_min: f64,
    pub points: usize,
    pub in_place: f64,
    pub pose: (f64, f64, f64),
}

pub const ZONES: [&str; 3] = ["heel cup", "quarters", "toe box"];

impl BootGeom {
    /// How far a foot point (upright boot frame) pushes into the rigid shell, mm (0 if free).
    /// The barrier is the band from the squeezed liner to 25 mm past the shell's outside: a point
    /// further out is outside the boot (free), and the band is thicker than any one step of the
    /// search (at most ~9 mm at the heel), so no point can jump it.
    fn push(&self, v: Point3<f64>, liner_min: f64) -> f64 {
        let d = self.d(v);
        let io = self.inner_off(v);
        let pen = d - (io - liner_min);
        if pen <= 0.0 || d > io + self.p.wall + 25.0 || v.z > self.z_trim(v) {
            return 0.0;
        }
        // Out through the open throat: in its band, and no deeper than 12 mm under the dorsal
        // line (below that the shell's quarters and sole are there).
        let n = self.prof.closest(v.x, v.z);
        if self.throat(&n, v.y) < 0.0 {
            let (pt, _) = self.prof.at(n.s);
            let nn = self.prof.normal(n.s);
            if (v.x - pt.x) * nn.x + (v.z - pt.z) * nn.z > -12.0 {
                return 0.0;
            }
        }
        pen
    }
}

pub fn removal_check(g: &BootGeom, foot: &IndexedMesh, liner_min: f64, budget: [f64; 3]) -> Removal {
    // The foot below the ankle bones (the leg above moves with the knee), every 6th scan point,
    // and its zone by length: heel to 30%, midfoot to 68%, forefoot beyond.
    let l = g.lm.length;
    let z_cut = g.lm.medial.z.max(g.lm.lateral.z) + 5.0;
    let pts: Vec<Point3<f64>> = foot.vertices.iter().filter(|v| v.z < z_cut).step_by(6).copied().collect();
    let zone = |v: &Point3<f64>| if v.x < 0.30 * l { 0 } else if v.x < 0.68 * l { 1 } else { 2 };
    let ball = Point3::new(0.72 * l, 0.0, 15.0);
    let (nb, nh, nk) = (29usize, 51usize, 31usize);
    let (db, dh, dk) = (2.5f64, 4.0f64, 4.0f64);
    let place = |v: &Point3<f64>, beta: f64, h: f64, back: f64| {
        let (mut x, mut z) = (v.x, v.z);
        if v.x < ball.x {
            let (sn, cs) = beta.to_radians().sin_cos();
            let (rx, rz) = (v.x - ball.x, v.z - ball.z);
            x = ball.x + rx * cs + rz * sn;
            z = ball.z + rz * cs - rx * sn;
        }
        Point3::new(x - back, v.y, z + h)
    };
    // The worst push per zone, and its point, at every pose; in parallel over the heel lift.
    type Cell = ([f64; 3], [usize; 3]);
    let grid: Vec<Cell> = std::thread::scope(|sc| {
        let hs: Vec<_> = (0..nb)
            .map(|i| {
                let (pts, place, zone) = (&pts, &place, &zone);
                sc.spawn(move || {
                    let mut out = Vec::with_capacity(nh * nk);
                    for j in 0..nh {
                        for k in 0..nk {
                            let (beta, h, back) = (i as f64 * db, j as f64 * dh, k as f64 * dk);
                            let mut cell: Cell = ([0.0; 3], [0; 3]);
                            for (pi, v) in pts.iter().enumerate() {
                                let push = g.push(place(v, beta, h, back), liner_min);
                                let z = zone(v);
                                if push > cell.0[z] {
                                    cell.0[z] = push;
                                    cell.1[z] = pi;
                                }
                            }
                            out.push(cell);
                        }
                    }
                    out
                })
            })
            .collect();
        hs.into_iter().flat_map(|h| h.join().unwrap()).collect()
    });
    let cost = |c: &Cell| (0..3).map(|z| c.0[z] / budget[z]).fold(0.0, f64::max);
    let idx = |i: usize, j: usize, k: usize| (i * nh + j) * nk + k;
    // Bottleneck search from the foot in place (0, 0, 0) to any pose at the top of the grid.
    let mut best = vec![f64::INFINITY; grid.len()];
    let mut from = vec![usize::MAX; grid.len()];
    let mut heap = std::collections::BinaryHeap::new();
    best[0] = cost(&grid[0]);
    heap.push((std::cmp::Reverse(OrdF(best[0])), 0usize));
    let mut goal = None;
    while let Some((std::cmp::Reverse(OrdF(c)), n)) = heap.pop() {
        if c > best[n] {
            continue;
        }
        let (i, rest) = (n / (nh * nk), n % (nh * nk));
        let (j, k) = (rest / nk, rest % nk);
        if j == nh - 1 {
            goal = Some(n);
            break;
        }
        let mut step = |m: usize| {
            let cm = c.max(cost(&grid[m]));
            if cm < best[m] {
                best[m] = cm;
                from[m] = n;
                heap.push((std::cmp::Reverse(OrdF(cm)), m));
            }
        };
        if i + 1 < nb {
            step(idx(i + 1, j, k));
        }
        if i > 0 {
            step(idx(i - 1, j, k));
        }
        if j + 1 < nh {
            step(idx(i, j + 1, k));
        }
        if j > 0 {
            step(idx(i, j - 1, k));
        }
        if k + 1 < nk {
            step(idx(i, j, k + 1));
        }
        if k > 0 {
            step(idx(i, j, k - 1));
        }
    }
    // Along that path: each zone's worst push, and the worst pose overall.
    let (mut push, mut at) = ([0.0f64; 3], [Point3::origin(); 3]);
    let mut worst = (f64::MIN, 0usize);
    let mut n = goal.unwrap_or(0);
    while n != usize::MAX {
        let c = &grid[n];
        for z in 0..3 {
            if c.0[z] > push[z] {
                push[z] = c.0[z];
                at[z] = pts[c.1[z]];
            }
        }
        if cost(c) > worst.0 {
            worst = (cost(c), n);
        }
        n = from[n];
    }
    let (i, rest) = (worst.1 / (nh * nk), worst.1 % (nh * nk));
    let (j, k) = (rest / nk, rest % nk);
    if std::env::var("BOOT_DEBUG").as_deref() == Ok("removal") {
        let (beta, h, back) = (i as f64 * db, j as f64 * dh, k as f64 * dk);
        let mut v: Vec<(f64, Point3<f64>, Point3<f64>)> =
            pts.iter().map(|p| { let q = place(p, beta, h, back); (g.push(q, liner_min), *p, q) }).filter(|x| x.0 > 0.0).collect();
        v.sort_by(|a, b| b.0.total_cmp(&a.0));
        println!("  removal debug: pose {beta} deg, up {h}, back {back}: {} points pushing", v.len());
        for (push, p, q) in v.iter().step_by((v.len() / 12).max(1)).take(12) {
            println!(
                "    push {push:5.1}: foot ({:5.0},{:5.0},{:5.0}) -> ({:5.0},{:5.0},{:5.0}); d {:5.1}, inner {:4.1}, trim z {:4.0}",
                p.x, p.y, p.z, q.x, q.y, q.z, g.d(*q), g.inner_off(*q), g.z_trim(*q)
            );
        }
    }
    Removal {
        push,
        at,
        budget,
        liner_min,
        points: pts.len(),
        in_place: grid[0].0.iter().copied().fold(0.0, f64::max),
        pose: (i as f64 * db, j as f64 * dh, k as f64 * dk),
    }
}

/// f64 with a total order, for the search's heap.
#[derive(Debug, Clone, Copy, PartialEq)]
struct OrdF(f64);
impl Eq for OrdF {}
impl PartialOrd for OrdF {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for OrdF {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        self.0.total_cmp(&o.0)
    }
}

/// Rigidly align `src` onto the `target` foot: point-to-surface ICP on the target's distance
/// field (each sample moved to its projection on the surface, then the best rotation and shift by
/// Kabsch's SVD), using only `src`'s points with z in `band` (the top of the foot, its sides and
/// the back of the heel: what load and posture change least). Returns the aligned mesh and the
/// final RMS distance of the band's points to the target, mm.
pub fn align_to(src: &IndexedMesh, target: &Foot, band: (f64, f64)) -> (IndexedMesh, f64) {
    let mut out = src.clone();
    let mut rms = f64::NAN;
    for _ in 0..40 {
        let sample: Vec<Point3<f64>> = out.vertices.iter().filter(|v| v.z > band.0 && v.z < band.1).step_by(4).copied().collect();
        if sample.len() < 10 {
            break;
        }
        let proj: Vec<Point3<f64>> = sample
            .iter()
            .map(|p| {
                let gr = target.sdf.gradient(*p);
                let gn = gr.norm().max(1e-9);
                p - gr / gn * target.eval(*p)
            })
            .collect();
        rms = (sample.iter().zip(&proj).map(|(a, b)| (a - b).norm_squared()).sum::<f64>() / sample.len() as f64).sqrt();
        let n = sample.len() as f64;
        let ca = sample.iter().fold(Vector3::zeros(), |s, p| s + p.coords) / n;
        let cb = proj.iter().fold(Vector3::zeros(), |s, p| s + p.coords) / n;
        let mut h = nalgebra::Matrix3::zeros();
        for (a, b) in sample.iter().zip(&proj) {
            h += (a.coords - ca) * (b.coords - cb).transpose();
        }
        let svd = h.svd(true, true);
        let (u, vt) = (svd.u.unwrap(), svd.v_t.unwrap());
        let mut r = vt.transpose() * u.transpose();
        if r.determinant() < 0.0 {
            let mut v = vt.transpose();
            v.column_mut(2).neg_mut();
            r = v * u.transpose();
        }
        let t = cb - r * ca;
        for v in &mut out.vertices {
            *v = Point3::from(r * v.coords + t);
        }
        if (r - nalgebra::Matrix3::identity()).norm() < 1e-6 && t.norm() < 1e-4 {
            break;
        }
    }
    (out, rms)
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

/// `boot <scan> [out] [ice|roller]`: the hockey boot on a foot, every part meshed, weighed, saved,
/// drawn. The roller boot is vented, the ice boot sealed.
pub fn run(scan_path: &Path, out: &Path, vented: bool) -> Result<(), String> {
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
    let p = HockeyParams { vented, ..HockeyParams::default() };
    let variant = if vented { "roller (vented)" } else { "ice (sealed)" };
    let t = Instant::now();
    // The fair base for the shell's outside: the closed foot closed again, plus the toe box.
    let fair = lb.closed.closed_with_margin(p.fair_closing, 1.5, 20.0)?;
    let base_fair = fair.solid().smooth_union(lb.toe.clone(), foot::BLEND);
    let (base, base_fair_c) = (lb.solid.clone(), base_fair.clone());
    let g = BootGeom::new(lb.solid, base_fair, lb.bounds, lm, p);
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
    // Room at the cuff's top, leg to liner, and how far the leg can tip before it meets the liner
    // (geometric, pivoting at the ankle bones' height; the liner's own give comes on top).
    let z_ank = 0.5 * (lm.medial.z + lm.lateral.z);
    let room: Vec<(&str, f64, f64)> = [("sides", 0.0, 1.0), ("front", 1.0, 0.0), ("back", -1.0, 0.0)]
        .iter()
        .map(|&(name, c, s)| {
            let q0 = Point3::new(lm.leg_c.0 + 35.0 * c, lm.leg_c.1 + 35.0 * s, 0.0);
            let top = g.z_trim(q0);
            let r = g.liner_in(Point3::new(q0.x, q0.y, top));
            (name, r, (r / (top - z_ank)).atan().to_degrees())
        })
        .collect();
    let room_line = format!(
        "cuff room at the top, leg to liner: {}",
        room.iter()
            .map(|(n, r, a)| if *r >= 0.0 { format!("{n} {r:.1} mm ({a:.1} deg of tip)") } else { format!("{n} {r:.1} mm (presses the calf)") })
            .collect::<Vec<_>>()
            .join(", ")
    );
    println!("{room_line}");
    let shape_line = format!(
        "{variant}; forward lean {:.0} deg; cuff top line z {:.0} at the front, {:.0} at the sides ({:.0} above the outer ankle bone), {:.0} at the back, padded collar {:.0} mm; {} vents; heel-lock eyelets: pair {} from the top, at the ankle crease (z {:.0})",
        p.lean_deg,
        g.z_cuff + p.trim_front_rise,
        g.z_cuff,
        p.cuff_above_ankle,
        g.z_trim_back,
        p.collar,
        g.vents.len(),
        g.lock.map_or(0, |k| g.eyelets.len() / 2 - k),
        g.z_crease
    );
    println!("{shape_line}");
    let t = Instant::now();
    // Off and on, the foot carries no weight, so it's checked with the unloaded scan when there
    // is one (the stand-in set has each foot at full and at minimal body weight: fBW, mBW),
    // aligned onto the loaded one. The loaded scan stays the shape the boot fits.
    let unloaded_path = scan_path.file_name().and_then(|n| n.to_str()).filter(|n| n.contains("fBW")).map(|n| scan_path.with_file_name(n.replace("fBW", "mBW")));
    let (out_foot, out_note) = match unloaded_path.filter(|p| p.exists()) {
        Some(up) => {
            let (raw_u, _, _) = scan::load(&up)?;
            let mu = scan::measure(&raw_u, 20);
            let (aligned, rms) = align_to(&foot::canonical(&raw_u, &mu), &foot, (20.0, 70.0));
            (aligned, format!("the unloaded scan {}, aligned to the loaded one (RMS {rms:.1} mm over 20-70 mm up)", up.display()))
        }
        None => (foot.mesh.clone(), "the loaded scan (no unloaded one found): a strict check, the sole spreads under load".to_string()),
    };
    let rm = removal_check(&g, &out_foot, 1.5, [2.0, 8.0, 3.0]);
    let removal_line = format!(
        "taking it off, with {out_note} (liner squeezed to {} mm, {} foot points, in place {:.2} mm): the way out needs {} (budgets steering the path {:.0}/{:.0}/{:.0} mm, assumed); worst pose: heel lifted {:.1} deg, up {:.0} mm, back {:.0} mm ({:.1} s)",
        rm.liner_min,
        rm.points,
        rm.in_place,
        (0..3)
            .map(|z| format!("{} {:.1} mm at ({:.0}, {:.0}, {:.0})", ZONES[z], rm.push[z], rm.at[z].x, rm.at[z].y, rm.at[z].z))
            .collect::<Vec<_>>()
            .join(", "),
        rm.budget[0],
        rm.budget[1],
        rm.budget[2],
        rm.pose.0,
        rm.pose.1,
        rm.pose.2,
        t.elapsed().as_secs_f64()
    );
    println!("{removal_line}");
    // BOOT_SWEEP=throat: the same rigid shell with the throat wider at the front (w0) and at the
    // top (w1), the on/off check for each; the narrowest throat that lets the foot out is the
    // design value (Jon, 2026-11-11: a wide throat like the CCM Jetspeed FT6 Pro's, the shell
    // still low tolerance round the foot, no flexible bands).
    if std::env::var("BOOT_SWEEP").as_deref() == Ok("throat") {
        println!("throat sweep (rigid shell; pushes in mm: heel cup / quarters / toe box):");
        for w0 in [26.0, 38.0, 50.0, 62.0] {
            for w1 in [48.0, 64.0, 80.0] {
                let ps = HockeyParams { throat_w0: w0, throat_w1: w1, ..p };
                let gs = BootGeom::new(base.clone(), base_fair_c.clone(), lb.bounds, lm, ps);
                let r = removal_check(&gs, &out_foot, 1.5, [2.0, 8.0, 3.0]);
                println!(
                    "  throat {w0:3.0} -> {w1:3.0} mm: {:5.1} / {:5.1} / {:5.1}  (worst pose {:.1} deg, up {:.0}, back {:.0})",
                    r.push[0], r.push[1], r.push[2], r.pose.0, r.pose.1, r.pose.2
                );
            }
        }
    }
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
    let grommets: Vec<Solid> = g
        .grommets()
        .into_iter()
        .map(|s| {
            let b = s.bounds().unwrap_or(g.bounds);
            g.leaned(s, b)
        })
        .collect();
    let laces = g.laces().map(|s| {
        let b = s.bounds().unwrap_or(g.bounds);
        g.leaned(s, b)
    });
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
    let _ = writeln!(summary, "hockey boot (track B, round 2 of the boot), {variant}, {}", scan_path.display());
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
    let _ = writeln!(summary, "{room_line}");
    let _ = writeln!(summary, "{shape_line}");
    let _ = writeln!(summary, "{removal_line}");
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
    let mut layers: Vec<Layer> =
        specs.iter().zip(&meshes).map(|(sp, m)| Layer::solid(m, sp.look).with_spec(finish_spec(sp.finish))).collect();
    layers.push(Layer::solid(&eyelet_mesh, EYELET_LOOK).with_spec(finish_spec("metal")));
    layers.push(Layer::solid(&lace_mesh, LACE_LOOK).with_spec(finish_spec("fabric")));
    let e = |e: std::io::Error| e.to_string();
    render_styled(&layers, &standard_views(), 2.0, &out.join("boot.png"), pretty).map_err(e)?;
    let ms = lm.medial_side;
    let hero = vec![
        View::looking(Vector3::new(-0.62, 0.62 * ms, -0.40), Vector3::z()),
        View::looking(Vector3::new(0.60, -0.62 * ms, -0.38), Vector3::z()),
        View::looking(Vector3::new(1.0, 0.0, -0.22), Vector3::z()),
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
    let colors: Vec<([f64; 3], f64)> = layers.iter().map(|l| (l.color, l.spec)).collect();
    let mut placed: Vec<(IndexedMesh, ([f64; 3], f64))> = Vec::new();
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
    let lineup: Vec<Layer> = placed.iter().map(|(m, (c, sp))| Layer::solid(m, *c).with_spec(*sp)).collect();
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
        let foot_cut = g.leaned(foot.solid(), foot.bounds).intersect(Solid::plane(n, off));
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
            let lb = g.leaned_bounds(lb);
            let leg = Solid::user_fn(move |q: Point3<f64>| gl.d(gl.unlean(q)).max(top - q.z).max(n.dot(&q.coords) - off), lb);
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

    // Light meshes for the 3D viewer (viewer/boot.html), opt-in: each part's uniform mesh
    // decimated by `cluster_decimate`, with its colour, weight and the way it moves when exploded.
    // (cf-design's mesh_simplified took ~18 min here and threw vertices off the surface: see
    // docs/CORTENFORGE.md.)
    if std::env::var_os("BOOT_VIEWER").is_none() {
        println!("viewer meshes skipped (set BOOT_VIEWER=1 to build them)");
    } else {
        let t = Instant::now();
        let cells = [2.0, 1.5, 2.0, 2.0, 1.5, 2.0, 2.0, 1.5];
        let grommet_union = grommets.iter().cloned().reduce(|a, b| a.union(b));
        let mut display: Vec<IndexedMesh> = std::thread::scope(|sc| {
            let mut hs: Vec<_> = specs
                .iter()
                .zip(&meshes)
                .zip(cells)
                .map(|((sp, m), cell)| {
                    let g = g.clone();
                    let kind = sp.kind;
                    sc.spawn(move || cluster_decimate(m, cell, Some(&|q| g.sdf(kind, g.unlean(q)))))
                })
                .collect();
            let gu = grommet_union.as_ref();
            hs.push(sc.spawn(move || match gu {
                Some(u) => cluster_decimate(&eyelet_mesh, 0.6, Some(&|q| u.evaluate(&q))),
                None => IndexedMesh::default(),
            }));
            let lu = laces.as_ref();
            hs.push(sc.spawn(move || match lu {
                Some(l) => cluster_decimate(&lace_mesh, 1.0, Some(&|q| l.evaluate(&q))),
                None => IndexedMesh::default(),
            }));
            hs.into_iter().map(|h| h.join().unwrap()).collect()
        });
        display.push(g.lean_mesh(&foot.mesh));
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
        type Row = (String, [f64; 3], [f64; 3], &'static str, f64, String);
        let mut names: Vec<Row> = specs
            .iter()
            .zip(&rows)
            .map(|(sp, r)| (sp.name.to_string(), sp.color, sp.look, sp.finish, r.3, sp.material.name.to_string()))
            .collect();
        names.push(("eyelets".into(), [0.80, 0.81, 0.84], EYELET_LOOK, "metal", rows[8].3, AL_6061_T6.name.into()));
        names.push(("laces".into(), [0.96, 0.96, 0.94], LACE_LOOK, "fabric", rows[9].3, LACE.name.into()));
        names.push(("foot (stand-in scan)".into(), skin, skin, "skin", 0.0, "".into()));
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
            let (name, col, look, finish, grams, mat) = &names[i];
            let e = explode[i];
            parts_json.push(format!(
                "{{\"name\":\"{name}\",\"color\":[{:.3},{:.3},{:.3}],\"look\":[{:.3},{:.3},{:.3}],\"finish\":\"{finish}\",\"grams\":{grams:.1},\"material\":\"{mat}\",\"explode\":[{},{},{}],\"vertexOffset\":{vo},\"vertexCount\":{},\"indexOffset\":{io},\"indexCount\":{}}}",
                col[0],
                col[1],
                col[2],
                look[0],
                look[1],
                look[2],
                e[0],
                e[1],
                e[2],
                m.vertices.len(),
                m.faces.len() * 3
            ));
        }
        let manifest = format!(
            "{{\"units\":\"mm\",\"scale\":{scale},\"up\":\"z\",\"variant\":\"{variant}\",\"vents\":{},\"leanDeg\":{},\"shellCarbonGrams\":{shell_cf:.1},\"medialSide\":{},\"cuts\":{{\"ankleX\":{:.1},\"legY\":{:.1}}},\"totalGrams\":{total:.1},\"parts\":[\n{}\n]}}\n",
            g.vents.len(),
            p.lean_deg,
            lm.medial_side,
            0.5 * (lm.medial.x + lm.lateral.x),
            lm.leg_c.1,
            parts_json.join(",\n")
        );
        // Base64 text: the artifact host serves text and images, not raw binary.
        let sfx = if vented { "_roller" } else { "" };
        std::fs::write(dir.join(format!("boot{sfx}.txt")), base64(&bin)).map_err(|e| e.to_string())?;
        std::fs::write(dir.join(format!("manifest{sfx}.json")), &manifest).map_err(|e| e.to_string())?;
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
