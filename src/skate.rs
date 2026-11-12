//! The whole skate, for display and weight: the hockey boot on what goes under it.
//!
//! - Roller: the round-2 rocker chassis (`frame`), with the ribbed upright walls that came out
//!   stiffest within the fatigue allowable when bonded to the sole (upper 240 mm, 6 panels,
//!   5 mm ribs on every member; PLAN.md, round 2 step 2), and display wheels.
//! - Ice: a FIRST LOOK at the holder, not yet designed (Track C): two windowed towers under the
//!   boot's mount pads on a beam that holds the runner in a channel; our own steel runner rocking
//!   against our own insert in the channel's roof (the rocking where the i2 has it, nothing
//!   proprietary), on a pivot pin. Its sizes are estimates to be measured or designed.

use crate::frame::{self, FrameParams};
use crate::hockey_boot::{ALT_BLUE, ALT_RED, ALT_WHITE};
use crate::materials::{Material, AL_6061_T6, AL_7075_T6, PA12_MJF, PU_BUSHING, STAINLESS, TI_6AL_4V};
use cortenforge::cf_design::{Aabb, Solid};
use nalgebra::{Point3, Vector3};

/// One part under the boot, and how it is drawn in each edition.
pub struct UnderPart {
    pub name: &'static str,
    pub solid: Solid,
    pub material: Material,
    /// Mesh cell for the STL, the renders and the weight; and for the viewer's decimation.
    pub cell: f64,
    pub view_cell: f64,
    pub look: [f64; 3],
    pub finish: &'static str,
    pub alt: [f64; 3],
    pub alt_finish: &'static str,
    /// False for display-only parts (the wheels), left out of the weight.
    pub weigh: bool,
}

pub struct Under {
    pub name: &'static str,
    pub parts: Vec<UnderPart>,
    /// The ice or the floor, z.
    pub ground_z: f64,
    pub note: String,
}

const STEEL_LOOK: [f64; 3] = [0.80, 0.81, 0.83];
const BOLT_LOOK: [f64; 3] = [0.55, 0.56, 0.58];

/// The ice holder's sizes: all estimates until measured or designed (Track C), except the length.
#[derive(Debug, Clone, Copy)]
pub struct HolderParams {
    /// Holder length: Jon's i2 is a 254 (PLAN.md, Jon's fit).
    pub len: f64,
    /// Platform under each mount pad: thickness; the pad's length and width (the boot's).
    pub top_t: f64,
    pub pad_len: f64,
    pub pad_w: f64,
    /// From the boot's sole to the holder's bottom; the beam along the bottom, its height and
    /// half-width; the towers' frame width round their windows.
    pub body_h: f64,
    pub beam_h: f64,
    pub beam_hw: f64,
    pub frame_w: f64,
    /// The runner's channel: half-width, height.
    pub channel_hw: f64,
    pub channel_h: f64,
    /// The runner: thickness, exposed height (new), profile radius; the insert it rocks
    /// against: radius, thickness at the pivot, length. The i2's 11 ft runner on a 23 ft insert
    /// (RESEARCH.md 14, the insert radius unverified) as the reference.
    pub runner_t: f64,
    pub exposed: f64,
    pub runner_r: f64,
    pub insert_r: f64,
    pub insert_t: f64,
    pub insert_len: f64,
}

impl Default for HolderParams {
    fn default() -> Self {
        Self {
            len: 254.0,
            top_t: 5.0,
            pad_len: 46.0,
            pad_w: 48.0,
            body_h: 46.0,
            beam_h: 15.0,
            beam_hw: 6.5,
            frame_w: 7.0,
            channel_hw: 2.3,
            channel_h: 13.0,
            runner_t: 2.9,
            exposed: 16.0,
            runner_r: 11.0 * 304.8,
            insert_r: 23.0 * 304.8,
            insert_t: 2.5,
            insert_len: 180.0,
        }
    }
}

fn rounded_rect(x: f64, y: f64, hx: f64, hy: f64, r: f64) -> f64 {
    let qx = x.abs() - (hx - r);
    let qy = y.abs() - (hy - r);
    qx.max(0.0).hypot(qy.max(0.0)) + qx.max(qy).min(0.0) - r
}

fn smin(a: f64, b: f64, k: f64) -> f64 {
    let h = (k - (a - b).abs()).max(0.0) / k;
    a.min(b) - h * h * k * 0.25
}

/// A convex polygon in the (x, z) plane, vertices counter-clockwise: the largest distance past
/// its edges' lines (exact inside, a little short outside its corners).
fn convex(x: f64, z: f64, v: &[(f64, f64)]) -> f64 {
    let mut d = f64::MIN;
    for i in 0..v.len() {
        let (a, b) = (v[i], v[(i + 1) % v.len()]);
        let (ex, ez) = (b.0 - a.0, b.1 - a.1);
        let l = ex.hypot(ez).max(1e-12);
        d = d.max(((x - a.0) * ez - (z - a.1) * ex) / l);
    }
    d
}

fn bounded(f: impl Fn(Point3<f64>) -> f64 + Send + Sync + 'static, lo: Point3<f64>, hi: Point3<f64>) -> Solid {
    Solid::user_fn(f, Aabb::new(lo, hi))
}

/// The ice skate's holder, insert, runner and pivot under a boot whose sole is at `zs`, with its
/// mount pads at `mounts` (heel, toe).
pub fn ice(zs: f64, mounts: [(f64, f64); 2], p: HolderParams) -> Under {
    let (xh, xt) = (mounts[0].0, mounts[1].0);
    let ym = 0.5 * (mounts[0].1 + mounts[1].1);
    let xc = 0.5 * (xh + xt);
    let (x0, x1) = (xc - 0.5 * p.len, xc + 0.5 * p.len);
    let zb = zs - p.body_h;
    let zbt = zb + p.beam_h;
    let zp_top = zs - p.top_t;
    let hl = 0.5 * p.pad_len;
    // The towers in side view: from each platform down to the beam, flaring toward the holder's
    // ends (the heel tower's back and the toe tower's front slope like a holder's).
    let heel = [(x0 + 4.0, zbt - 2.0), (xh + hl + 12.0, zbt - 2.0), (xh + hl, zp_top + 1.0), (xh - hl, zp_top + 1.0)];
    let toe = [(xt - hl - 12.0, zbt - 2.0), (x1 - 4.0, zbt - 2.0), (xt + hl, zp_top + 1.0), (xt - hl, zp_top + 1.0)];
    let fw = p.frame_w;
    let side = move |x: f64, z: f64| -> f64 {
        let towers = convex(x, z, &heel).min(convex(x, z, &toe));
        let platforms = rounded_rect(x - xh, z - (zs - 0.5 * p.top_t), hl, 0.5 * p.top_t, 1.5)
            .min(rounded_rect(x - xt, z - (zs - 0.5 * p.top_t), hl, 0.5 * p.top_t, 1.5));
        let beam = rounded_rect(x - xc, z - (zb + 0.5 * p.beam_h), 0.5 * p.len, 0.5 * p.beam_h, 5.0);
        let boss = (x - xc).hypot(z - zbt) - 9.0;
        let body = smin(smin(smin(towers, platforms, 4.0), beam, 6.0), boss, 4.0);
        // Windows through the towers, leaving a frame `frame_w` wide.
        let windows = (convex(x, z, &heel) + fw).min(convex(x, z, &toe) + fw);
        body.max(-windows)
    };
    let zp = zb + 7.0; // the pivot
    let hw = move |z: f64| -> f64 {
        let t = ((z - zbt) / (zp_top - zbt)).clamp(0.0, 1.0);
        p.beam_hw + (0.5 * p.pad_w - p.beam_hw) * t.powf(1.4)
    };
    let holder = bounded(
        move |q| {
            let body = side(q.x, q.z).max((q.y - ym).abs() - hw(q.z));
            let channel = ((q.y - ym).abs() - p.channel_hw).max(q.z - (zb + p.channel_h)).max((q.x - xc).abs() - (0.5 * p.len - 2.0));
            let pin_hole = (q.x - xc).hypot(q.z - zp) - 3.0;
            body.max(-channel).max(-pin_hole)
        },
        Point3::new(x0 - 2.0, ym - 0.5 * p.pad_w - 2.0, zb - 2.0),
        Point3::new(x1 + 2.0, ym + 0.5 * p.pad_w + 2.0, zs + 0.5),
    );
    let run_top = zb + p.channel_h - p.insert_t;
    let ice_z = zb - p.exposed;
    let half_len = 0.5 * p.len - 8.0;
    let runner = bounded(
        move |q| {
            let dx = q.x - xc;
            let rect = rounded_rect(dx, q.z - 0.5 * (ice_z + run_top), half_len, 0.5 * (run_top - ice_z), 20.0);
            let rocker = ice_z + dx * dx / (2.0 * p.runner_r) - q.z;
            rect.max(rocker).max((q.y - ym).abs() - 0.5 * p.runner_t)
        },
        Point3::new(xc - half_len - 1.0, ym - 2.0, ice_z - 1.0),
        Point3::new(xc + half_len + 1.0, ym + 2.0, run_top + 1.0),
    );
    let insert = bounded(
        move |q| {
            let dx = q.x - xc;
            ((q.y - ym).abs() - (p.channel_hw - 0.2))
                .max(dx.abs() - 0.5 * p.insert_len)
                .max(run_top + dx * dx / (2.0 * p.insert_r) - q.z)
                .max(q.z - (zb + p.channel_h))
        },
        Point3::new(xc - 0.5 * p.insert_len - 1.0, ym - 3.0, run_top - 1.0),
        Point3::new(xc + 0.5 * p.insert_len + 1.0, ym + 3.0, zb + p.channel_h + 1.0),
    );
    let reach = p.beam_hw + 1.2;
    let pin = bounded(
        move |q| {
            let r = (q.x - xc).hypot(q.z - zp);
            let a = (q.y - ym).abs();
            let shaft = (r - 3.0).max(a - reach);
            let heads = (r - 5.5).max(a - reach).max(p.beam_hw - a);
            shaft.min(heads)
        },
        Point3::new(xc - 7.0, ym - reach - 1.0, zp - 7.0),
        Point3::new(xc + 7.0, ym + reach + 1.0, zp + 7.0),
    );
    let note = format!(
        "ice holder, a first look (estimates, Track C): {:.0} mm long, the sole {:.0} mm above the holder's bottom, runner {:.1} mm, {:.0} mm exposed, {:.0} ft profile, rocking on a {:.0} ft insert; ice {:.0} mm below the sole",
        p.len,
        p.body_h,
        p.runner_t,
        p.exposed,
        p.runner_r / 304.8,
        p.insert_r / 304.8,
        zs - ice_z
    );
    Under {
        name: "ice holder",
        parts: vec![
            UnderPart {
                name: "holder",
                solid: holder,
                material: PA12_MJF,
                cell: 0.8,
                view_cell: 1.2,
                look: [0.09, 0.09, 0.10],
                finish: "gloss",
                alt: ALT_BLUE,
                alt_finish: "gloss",
                weigh: true,
            },
            UnderPart {
                name: "runner",
                solid: runner,
                material: STAINLESS,
                cell: 0.4,
                view_cell: 0.6,
                look: STEEL_LOOK,
                finish: "metal",
                alt: STEEL_LOOK,
                alt_finish: "metal",
                weigh: true,
            },
            UnderPart {
                name: "rocker insert",
                solid: insert,
                material: PA12_MJF,
                cell: 0.4,
                view_cell: 0.6,
                look: [0.30, 0.30, 0.32],
                finish: "matte",
                alt: ALT_RED,
                alt_finish: "satin",
                weigh: true,
            },
            UnderPart {
                name: "pivot pin",
                solid: pin,
                material: TI_6AL_4V,
                cell: 0.3,
                view_cell: 0.5,
                look: BOLT_LOOK,
                finish: "metal",
                alt: BOLT_LOOK,
                alt_finish: "metal",
                weigh: true,
            },
        ],
        ground_z: ice_z,
        note,
    }
}

/// The roller skate's chassis under a boot whose sole is at `zs`, centred under its mount pads,
/// and display wheels: urethane with a rounded tread on a hub.
pub fn roller(zs: f64, mounts: [(f64, f64); 2], length: f64) -> Under {
    let fp = FrameParams { upper_len: 240.0, wall_panels: 6, rib_d: 5.0, rib_chords: 2, ..FrameParams::default() };
    let ym = 0.5 * (mounts[0].1 + mounts[1].1);
    let f = frame::build(zs, length, fp);
    let l = f.layout;
    let shift = Vector3::new(0.0, ym, 0.0);
    let (r_out, w, hub_r) = (0.5 * fp.wheel_d, fp.wheel_w, 24.0);
    let axles = l.axle_x;
    let az = l.axle_z;
    let wheel_box = (
        Point3::new(axles[0] - r_out - 1.0, ym - 0.5 * w - 1.0, az - r_out - 1.0),
        Point3::new(axles[3] + r_out + 1.0, ym + 0.5 * w + 1.0, az + r_out + 1.0),
    );
    let tyres = bounded(
        move |q| {
            let yy = (q.y - ym).abs();
            axles
                .iter()
                .map(|&ax| {
                    let r = (q.x - ax).hypot(q.z - az);
                    let (a, b) = (r_out - hub_r, 0.5 * w);
                    let e = (((r - hub_r) / a).powi(2) + (yy / b).powi(2)).sqrt() - 1.0;
                    (e * a.min(b)).max(hub_r - 1.0 - r)
                })
                .fold(f64::MAX, f64::min)
        },
        wheel_box.0,
        wheel_box.1,
    );
    let hubs = bounded(
        move |q| {
            let yy = (q.y - ym).abs();
            axles
                .iter()
                .map(|&ax| {
                    let r = (q.x - ax).hypot(q.z - az);
                    (r - hub_r).max(yy - (0.5 * w - 2.0)).max(0.5 * fp.axle_d + 0.6 - r)
                })
                .fold(f64::MAX, f64::min)
        },
        wheel_box.0,
        wheel_box.1,
    );
    let frame_look = [0.13, 0.13, 0.14];
    let part = |name, solid: Solid, material, cell, view_cell, look, finish, alt, alt_finish| UnderPart {
        name,
        solid: solid.translate(shift),
        material,
        cell,
        view_cell,
        look,
        finish,
        alt,
        alt_finish,
        weigh: true,
    };
    let mut parts = vec![
        part("chassis upper", f.upper, AL_7075_T6, 0.6, 1.0, frame_look, "metal", ALT_BLUE, "metal"),
        part("chassis lower", f.lower, AL_7075_T6, 0.6, 1.0, frame_look, "metal", ALT_BLUE, "metal"),
    ];
    if let Some(t) = f.ties {
        parts.push(part("ties", t, AL_7075_T6, 0.4, 0.6, frame_look, "metal", ALT_BLUE, "metal"));
    }
    parts.extend([
        part("axles", f.axles, AL_7075_T6, 0.4, 0.6, BOLT_LOOK, "metal", BOLT_LOOK, "metal"),
        part("spacers", f.spacers, AL_6061_T6, 0.3, 0.5, BOLT_LOOK, "metal", BOLT_LOOK, "metal"),
        part("bolt", f.bolt, TI_6AL_4V, 0.3, 0.5, BOLT_LOOK, "metal", BOLT_LOOK, "metal"),
        part("nut", f.nut, TI_6AL_4V, 0.3, 0.5, BOLT_LOOK, "metal", BOLT_LOOK, "metal"),
        part("bushing", f.bushing, PU_BUSHING, 0.4, 0.6, [0.25, 0.25, 0.27], "matte", ALT_RED, "satin"),
    ]);
    for (name, solid, look, alt, finish) in [("wheels", tyres, [0.07, 0.07, 0.08], ALT_WHITE, "matte"), ("hubs", hubs, [0.35, 0.36, 0.38], ALT_RED, "satin")] {
        parts.push(UnderPart {
            name,
            solid,
            material: PU_BUSHING,
            cell: 0.8,
            view_cell: 1.4,
            look,
            finish,
            alt,
            alt_finish: finish,
            weigh: false,
        });
    }
    let note = format!(
        "roller chassis (round 2: ribbed upright walls, upper {:.0} mm, {} panels, {:.0} mm ribs; 4 x {:.0} mm wheels, {:.1} ft rocker); floor {:.0} mm below the sole; wheels for display, not weighed",
        fp.upper_len,
        fp.wall_panels,
        fp.rib_d,
        fp.wheel_d,
        fp.rocker_r / 304.8,
        zs - l.ground_z
    );
    Under { name: "roller chassis", parts, ground_z: l.ground_z, note }
}
