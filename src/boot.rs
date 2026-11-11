//! The last and the boot shell, grown from a base surface (round 1: simple, uniform).
//!
//! The base surface is the foot made last-like: closed (arch and small hollows filled) and blended
//! with a convex toe box (`round1`). Layers out from it: `ease` (fit allowance; the last is the
//! base plus this), the liner (foam, incl. the footbed), the shell wall. The shell is trimmed at
//! `top`, opened over the instep for the laces (the throat), and given a flat sole plate to mount a
//! chassis or a holder on. Round 1 leaves out: a locked heel pocket, the tendon guard, eyelets,
//! variable wall thickness, the tongue.

use cortenforge::cf_design::{Aabb, Solid};
use nalgebra::{Point3, Vector3};

#[derive(Debug, Clone, Copy)]
pub struct BootParams {
    /// Fit allowance between the base surface and the liner, mm.
    pub ease: f64,
    /// Liner foam (and footbed) thickness, mm.
    pub liner: f64,
    /// Shell wall, mm.
    pub wall: f64,
    /// Shell trim height above the floor, mm.
    pub top: f64,
    /// Throat (lace opening) from / to, as fractions of foot length, and its width, mm.
    pub throat_from: f64,
    pub throat_to: f64,
    pub throat_width: f64,
    /// Sole plate thickness below the shell, mm.
    pub sole: f64,
}

impl Default for BootParams {
    fn default() -> Self {
        Self { ease: 1.0, liner: 4.0, wall: 2.5, top: 110.0, throat_from: 0.32, throat_to: 0.70, throat_width: 40.0, sole: 3.0 }
    }
}

pub struct Boot {
    pub params: BootParams,
    /// The base plus the fit allowance.
    pub last: Solid,
    /// The shell (walls, trimmed, throat cut, sole plate).
    pub shell: Solid,
    /// The liner: between the last and the shell's inside, trimmed like the shell.
    pub liner: Solid,
    /// z of the sole plate's flat bottom (the chassis or holder mounts here), mm.
    pub sole_bottom: f64,
    pub bounds: Aabb,
}

/// `base`: the last-like surface (negative inside), covering `bounds`; `length`: heel to toe, mm.
pub fn build(base: Solid, bounds: Aabb, length: f64, p: BootParams) -> Boot {
    let inner_off = p.ease + p.liner;
    let outer_off = inner_off + p.wall;
    let last = base.clone().offset(p.ease);
    let inner = base.clone().offset(inner_off);
    let outer = base.clone().offset(outer_off);

    // Keep below the trim height.
    let below_top = || Solid::plane(Vector3::z(), p.top);
    let mut shell = outer.subtract(inner.clone()).intersect(below_top());

    // Throat: a strip over the instep, from just above the sole up.
    let (x0, x1) = (p.throat_from * length, p.throat_to * length);
    let throat = Solid::cuboid(Vector3::new(0.5 * (x1 - x0), 0.5 * p.throat_width, 0.5 * (p.top + 20.0)))
        .translate(Vector3::new(0.5 * (x0 + x1), 0.0, 0.5 * (p.top + 20.0) + 25.0));
    shell = shell.subtract(throat);

    // Sole plate: the shell's outline at the footprint, extruded down to a flat bottom.
    let sole_top = -p.ease - p.liner;
    let sole_bottom = -outer_off - p.sole;
    let z_ref = 3.0;
    let plate_bounds = Aabb::new(
        Point3::new(bounds.min.x, bounds.min.y, sole_bottom - 1.0),
        Point3::new(bounds.max.x, bounds.max.y, sole_top + 1.0),
    );
    let outline_of = base.clone();
    let plate = Solid::user_fn(
        move |q: Point3<f64>| {
            let outline = outline_of.evaluate(&Point3::new(q.x, q.y, z_ref)) - outer_off;
            let slab = (sole_bottom - q.z).max(q.z - sole_top);
            outline.max(slab)
        },
        plate_bounds,
    );
    shell = shell.union(plate);

    let liner = inner.subtract(last.clone()).intersect(below_top());
    let mut bounds = bounds;
    bounds.min.z = sole_bottom - 2.0;
    Boot { params: p, last, shell, liner, sole_bottom, bounds }
}
