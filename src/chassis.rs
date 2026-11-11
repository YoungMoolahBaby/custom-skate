//! The roller chassis, round 1: a rocker fixed under the boot's sole rolling on a flat lower
//! frame, held by a bolt through a bushing. Built in the boot's frame (x heel to toe, z up, mm).
//!
//! Round-1 geometry is a plausible first layout, not a design: every dimension below is a
//! parameter, picked to look like a hockey frame, then weighed. Phase 2-6 (PLAN.md) size them.

use cortenforge::cf_design::Solid;
use nalgebra::{UnitQuaternion, Vector3};

#[derive(Debug, Clone, Copy)]
pub struct ChassisParams {
    /// Wheel diameter and width, mm; gap between wheels, mm.
    pub wheel_d: f64,
    pub wheel_w: f64,
    pub wheel_gap: f64,
    /// Frame centre, as a fraction of foot length from the heel.
    pub center: f64,
    /// Rocker: radius (mm), thickness at the centre, width, length; pocket floor (skin) thickness.
    pub rocker_r: f64,
    pub rocker_t: f64,
    pub rocker_w: f64,
    pub rocker_len: f64,
    pub rocker_skin: f64,
    /// Lower frame: rail thickness, bridge thickness, bridge length, axle diameter, clearance
    /// between the wheel tops and the bridge, side play (spacers) each side of a wheel.
    pub rail_t: f64,
    pub bridge_t: f64,
    pub bridge_len: f64,
    pub axle_d: f64,
    pub clearance: f64,
    pub spacer: f64,
    /// Lightening holes in the rails between axles: radius, mm.
    pub hole_r: f64,
    /// Coupling: bolt diameter and bushing outer diameter / length, mm.
    pub bolt_d: f64,
    pub bushing_od: f64,
    pub bushing_len: f64,
}

impl Default for ChassisParams {
    fn default() -> Self {
        Self {
            wheel_d: 72.0,
            wheel_w: 24.0,
            wheel_gap: 4.0,
            center: 0.47,
            rocker_r: 7.5 * 304.8,
            rocker_t: 6.0,
            rocker_w: 30.0,
            rocker_len: 180.0,
            rocker_skin: 2.5,
            rail_t: 2.5,
            bridge_t: 4.0,
            bridge_len: 180.0,
            axle_d: 8.0,
            clearance: 3.0,
            spacer: 1.5,
            hole_r: 15.0,
            bolt_d: 6.0,
            bushing_od: 18.0,
            bushing_len: 10.0,
        }
    }
}

pub struct Chassis {
    pub params: ChassisParams,
    pub rocker: Solid,
    pub lower: Solid,
    pub axles: Solid,
    pub bolt: Solid,
    pub bushing: Solid,
    /// Wheels, for display only.
    pub wheels: Solid,
    /// Axle x positions and height, the ground height, the bridge top, mm.
    pub axle_x: [f64; 4],
    pub axle_z: f64,
    pub ground_z: f64,
    pub bridge_top: f64,
    pub xc: f64,
}

fn slab(x0: f64, x1: f64, y0: f64, y1: f64, z0: f64, z1: f64) -> Solid {
    Solid::cuboid(Vector3::new(0.5 * (x1 - x0), 0.5 * (y1 - y0), 0.5 * (z1 - z0)))
        .translate(Vector3::new(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1)))
}

/// A cylinder along y (an axle, a wheel), centred at (x, 0, z).
fn along_y(r: f64, len: f64, x: f64, z: f64) -> Solid {
    let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2);
    Solid::cylinder(r, 0.5 * len).rotate(rot).translate(Vector3::new(x, 0.0, z))
}

/// `sole_bottom`: z of the boot's flat sole, mm; `length`: foot length, mm.
pub fn build(sole_bottom: f64, length: f64, p: ChassisParams) -> Chassis {
    let xc = p.center * length;
    let pitch = p.wheel_d + p.wheel_gap;
    let axle_x = [xc - 1.5 * pitch, xc - 0.5 * pitch, xc + 0.5 * pitch, xc + 1.5 * pitch];

    // Stack: sole -> rocker -> bridge -> clearance -> wheel tops.
    let bridge_top = sole_bottom - p.rocker_t;
    let bridge_bottom = bridge_top - p.bridge_t;
    let axle_z = bridge_bottom - p.clearance - 0.5 * p.wheel_d;
    let ground_z = axle_z - 0.5 * p.wheel_d;

    // Rocker: a block under the sole whose bottom is an arc of radius rocker_r, lowest at xc.
    let half_w = 0.5 * p.rocker_w;
    // 0.1 mm above the bridge: a line contact, not an overlap (a tangent arc read as a 117 mm3 clash).
    let arc_center_z = bridge_top + p.rocker_r + 0.1;
    let rot_y = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2);
    let drum = Solid::cylinder(p.rocker_r, half_w).rotate(rot_y).translate(Vector3::new(xc, 0.0, arc_center_z));
    let block = slab(xc - 0.5 * p.rocker_len, xc + 0.5 * p.rocker_len, -half_w, half_w, bridge_top - 1.0, sole_bottom);
    let mut rocker = block.intersect(drum);
    // Two pockets from the top (closed by the sole), leaving the skin, the rim and a boss round the bolt.
    for side in [-1.0, 1.0] {
        let (a, b) = if side < 0.0 { (xc - 0.5 * p.rocker_len + 8.0, xc - 14.0) } else { (xc + 14.0, xc + 0.5 * p.rocker_len - 8.0) };
        rocker = rocker.subtract(slab(a, b, -half_w + 3.0, half_w - 3.0, bridge_top + p.rocker_skin, sole_bottom + 1.0));
    }
    rocker = rocker.subtract(Solid::cylinder(0.5 * p.bolt_d + 0.2, 20.0).translate(Vector3::new(xc, 0.0, sole_bottom)));

    // Lower frame: two rails and the bridge on top.
    let inner_half = 0.5 * p.wheel_w + p.spacer;
    let outer_half = inner_half + p.rail_t;
    let (x0, x1) = (axle_x[0] - 0.5 * p.axle_d - 6.0, axle_x[3] + 0.5 * p.axle_d + 6.0);
    let rail_bottom = axle_z - 0.5 * p.axle_d - 6.0;
    let mut lower = slab(x0, x1, -outer_half, -inner_half, rail_bottom, bridge_top)
        .union(slab(x0, x1, inner_half, outer_half, rail_bottom, bridge_top));
    lower = lower.union(slab(xc - 0.5 * p.bridge_len, xc + 0.5 * p.bridge_len, -outer_half, outer_half, bridge_bottom, bridge_top));
    // Lightening holes through both rails, between the axles, and the axle bores.
    for k in 0..3 {
        let hx = 0.5 * (axle_x[k] + axle_x[k + 1]);
        lower = lower.subtract(along_y(p.hole_r, 4.0 * outer_half, hx, axle_z + 2.0));
    }
    for &ax in &axle_x {
        lower = lower.subtract(along_y(0.5 * p.axle_d, 4.0 * outer_half, ax, axle_z));
    }
    // The bushing pocket under the bridge and the bolt hole.
    lower = lower.subtract(Solid::cylinder(0.5 * p.bolt_d + 0.2, 20.0).translate(Vector3::new(xc, 0.0, bridge_top)));

    // Axles (rail to rail plus a 1.5 mm head each side).
    let axle_len = 2.0 * outer_half + 3.0;
    let mut axles = along_y(0.5 * p.axle_d, axle_len, axle_x[0], axle_z);
    for &ax in &axle_x[1..] {
        axles = axles.union(along_y(0.5 * p.axle_d, axle_len, ax, axle_z));
    }

    // Coupling: bolt from the rocker's top through the bridge and the bushing hung below it.
    let bolt_top = sole_bottom;
    let bolt_bottom = bridge_bottom - p.bushing_len - 4.0;
    let bolt = Solid::cylinder(0.5 * p.bolt_d, 0.5 * (bolt_top - bolt_bottom))
        .translate(Vector3::new(xc, 0.0, 0.5 * (bolt_top + bolt_bottom)));
    let bushing = Solid::cylinder(0.5 * p.bushing_od, 0.5 * p.bushing_len)
        .translate(Vector3::new(xc, 0.0, bridge_bottom - 0.5 * p.bushing_len))
        .subtract(Solid::cylinder(0.5 * p.bolt_d + 0.25, p.bushing_len).translate(Vector3::new(xc, 0.0, bridge_bottom - 0.5 * p.bushing_len)));

    let mut wheels = along_y(0.5 * p.wheel_d, p.wheel_w, axle_x[0], axle_z);
    for &ax in &axle_x[1..] {
        wheels = wheels.union(along_y(0.5 * p.wheel_d, p.wheel_w, ax, axle_z));
    }

    Chassis { params: p, rocker, lower, axles, bolt, bushing, wheels, axle_x, axle_z, ground_z, bridge_top, xc }
}

/// The ice skate's holder and runner, round 1: a PLACEHOLDER shaped like a holder (Jon's
/// Marsblade i2 254 to start) until the real one is measured. `len`: holder length, mm.
pub struct Holder {
    pub holder: Solid,
    pub runner: Solid,
    pub ice_z: f64,
}

pub fn holder(sole_bottom: f64, length: f64, len: f64) -> Holder {
    let xc = 0.5 * length;
    let (x0, x1) = (xc - 0.5 * len, xc + 0.5 * len);
    let body_h = 48.0;
    let z_low = sole_bottom - body_h;
    // Two towers under heel and ball, joined by a beam along the bottom; a window between them.
    let body = slab(x0, x1, -7.0, 7.0, z_low, sole_bottom)
        .subtract(slab(x0 + 70.0, x1 - 70.0, -10.0, 10.0, z_low + 12.0, sole_bottom - 8.0));
    // Runner: 3 mm steel, 22 mm tall above the holder's bottom + 12 mm exposed, its bottom a 9 ft arc.
    let r = 9.0 * 304.8;
    let ice_z = z_low - 12.0;
    let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2);
    let profile = Solid::cylinder(r, 2.0).rotate(rot).translate(Vector3::new(xc, 0.0, ice_z + r));
    let runner = slab(x0 + 5.0, x1 - 5.0, -1.5, 1.5, ice_z - 1.0, z_low + 22.0).intersect(profile);
    Holder { holder: body, runner, ice_z }
}
