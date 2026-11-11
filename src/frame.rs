//! The roller chassis, round 2: the rocker moved down and out.
//!
//! Round 1 put a 6 mm rocker on top of the lower frame. Two problems showed up when the loads were
//! followed by hand (`strength.rs`, docs/PLAN.md round 2): the thin rocker bridges from heel to
//! ball and bends far too much, and the lateral push at the wheels makes a roll moment (lateral
//! force x ~80 mm) that a narrow rocking joint on top can only carry by bending thin walls.
//!
//! Round 2 layout, in the boot's frame (x heel to toe, y across, z up, mm):
//! - **Upper**: a web under the boot sole and two walls hanging outside the lower rails. Each
//!   wall's bottom edge is the rocker arc (radius `rocker_r`). The walls are Warren trusses in
//!   their own plane: deep, so the upper is stiff in bending for its weight.
//! - **Lower**: two rails (a Warren truss in each rail's plane: a bottom chord through the axle
//!   bosses, a top chord near the web, diagonals and posts) with a short ledge on each rail's
//!   outer face, just above the axle heads. The walls' arcs roll on the ledges. The ledges are
//!   low (~45 mm above the ground) and wide apart (~37 mm), so the vertical load alone holds the
//!   roll up to a lateral/vertical ratio of ~0.4 before one side would lift (`strength.rs`).
//! - **Coupling**: a bolt from the upper's web down through a bushing seated in a cross-member
//!   between the rails, in the gap between the middle wheels, with a nut below.
//! - Optional ties across the walls' feet, through the rails' truss windows, close the upper into
//!   a box against the lateral push.
//!
//! Every dimension is a parameter; the defaults are a first sizing, weighed and checked by
//! `cargo run --release -- chassis`.

use cortenforge::cf_design::{Aabb, Solid};
use nalgebra::{Point3, UnitQuaternion, Vector3};

#[derive(Debug, Clone, Copy)]
pub struct FrameParams {
    /// Wheels: diameter, width, gap between wheels, mm; frame centre as a fraction of foot length.
    pub wheel_d: f64,
    pub wheel_w: f64,
    pub wheel_gap: f64,
    pub center: f64,
    /// Axle diameter; side play (spacer) between a wheel and a rail, mm.
    pub axle_d: f64,
    pub spacer: f64,
    /// Gap from the wheel tops to the underside of the upper's web, mm (sets the stack height).
    pub web_clear: f64,
    /// Lower rails: thickness; boss ring width round the axle bore; bottom chord from
    /// `bot_z0` below the axle line up to the ledge top; top chord height (0 = no top chord, a
    /// shallow rail); gap from the rail top to the web; diagonal and post widths; fillet radius;
    /// rail length past the end bosses.
    pub rail_t: f64,
    pub boss_w: f64,
    pub bot_z0: f64,
    pub top_h: f64,
    pub top_clear: f64,
    pub diag_w: f64,
    pub post_w: f64,
    pub fillet: f64,
    pub rail_end: f64,
    /// Ledges (the rocker tracks) on the rails' outer faces: height of the ledge top above the
    /// axle line, width out from the rail, thickness, length along x (centred).
    pub ledge_h: f64,
    pub ledge_w: f64,
    pub ledge_t: f64,
    pub ledge_len: f64,
    /// Bottom chord top outside the ledge span, above the axle line (under the ledge the chord
    /// reaches up to the ledge).
    pub bot_h_out: f64,
    /// Centre cross-member (holds the bushing): a ring round the bushing bore (wall), an arm
    /// to each rail (width along x), height.
    pub cm_wall: f64,
    pub cm_arm: f64,
    pub cm_h: f64,
    /// Upper: length, rocker radius, web thickness, wall thickness, gap between a wall and the
    /// rail it hangs beside, the wall's top and bottom (foot) chord heights, its diagonal
    /// width and number of panels; web windows per side of the bolt and the rib between them.
    pub upper_len: f64,
    pub rocker_r: f64,
    pub web_t: f64,
    pub wall_t: f64,
    pub side_gap: f64,
    pub wall_top_h: f64,
    pub wall_bot_h: f64,
    pub wall_diag_w: f64,
    pub wall_panels: usize,
    pub web_windows: usize,
    pub web_rib: f64,
    /// Ties across the walls' feet at the outer wheel gaps (diameter, 0 = none), mm.
    pub tie_d: f64,
    /// Coupling: bolt diameter, bushing outer diameter and length, mm.
    pub bolt_d: f64,
    pub bushing_od: f64,
    pub bushing_len: f64,
    /// Rocking travel: the contact moves +- this far from the centre, mm (an assumption).
    pub travel: f64,
}

impl Default for FrameParams {
    fn default() -> Self {
        Self {
            wheel_d: 72.0,
            wheel_w: 24.0,
            wheel_gap: 4.0,
            center: 0.47,
            axle_d: 8.0,
            spacer: 1.5,
            web_clear: 8.0,
            rail_t: 2.0,
            boss_w: 4.0,
            bot_z0: -3.0,
            top_h: 5.0,
            top_clear: 4.0,
            diag_w: 4.0,
            post_w: 4.0,
            fillet: 2.0,
            rail_end: 2.0,
            ledge_h: 10.0,
            ledge_w: 4.0,
            ledge_t: 3.0,
            ledge_len: 140.0,
            bot_h_out: 5.0,
            cm_wall: 2.0,
            cm_arm: 8.0,
            cm_h: 12.0,
            upper_len: 180.0,
            rocker_r: 7.5 * 304.8,
            web_t: 1.5,
            wall_t: 2.0,
            side_gap: 0.5,
            wall_top_h: 5.0,
            wall_bot_h: 9.0,
            wall_diag_w: 4.0,
            wall_panels: 4,
            web_windows: 3,
            web_rib: 6.0,
            tie_d: 5.0,
            bolt_d: 6.0,
            bushing_od: 18.0,
            bushing_len: 10.0,
            travel: 50.0,
        }
    }
}

/// Where everything is, derived from the parameters.
#[derive(Debug, Clone, Copy)]
pub struct Layout {
    pub xc: f64,
    pub axle_x: [f64; 4],
    pub axle_z: f64,
    pub ground_z: f64,
    /// Sole bottom (top of the upper) and the web's underside.
    pub z_s: f64,
    pub z_wb: f64,
    /// Ledge top (the rocker track) and the rail top.
    pub z_t: f64,
    pub rail_top: f64,
    /// Rail inner and outer faces (|y|), wall inner and outer faces.
    pub y_in: f64,
    pub y_out: f64,
    pub y_wi: f64,
    pub y_wo: f64,
    /// Rail ends, upper ends.
    pub x0: f64,
    pub x1: f64,
    pub xu0: f64,
    pub xu1: f64,
    /// Centre of the rocker arc at rest.
    pub arc_c: Point3<f64>,
    pub r_boss: f64,
}

impl Layout {
    pub fn new(sole_bottom: f64, length: f64, p: &FrameParams) -> Self {
        let xc = p.center * length;
        let pitch = p.wheel_d + p.wheel_gap;
        let axle_x = [xc - 1.5 * pitch, xc - 0.5 * pitch, xc + 0.5 * pitch, xc + 1.5 * pitch];
        let z_s = sole_bottom;
        let z_wb = z_s - p.web_t;
        let axle_z = z_wb - p.web_clear - 0.5 * p.wheel_d;
        let ground_z = axle_z - 0.5 * p.wheel_d;
        let z_t = axle_z + p.ledge_h;
        let rail_top = z_wb - p.top_clear;
        let y_in = 0.5 * p.wheel_w + p.spacer;
        let y_out = y_in + p.rail_t;
        let y_wi = y_out + p.side_gap;
        let y_wo = y_wi + p.wall_t;
        let r_boss = 0.5 * p.axle_d + p.boss_w;
        let x0 = axle_x[0] - r_boss - p.rail_end;
        let x1 = axle_x[3] + r_boss + p.rail_end;
        // 0.1 mm above the ledge: a line contact, not an overlap (round 1: a tangent arc read as a clash).
        let arc_c = Point3::new(xc, 0.0, z_t + 0.1 + p.rocker_r);
        Self {
            xc,
            axle_x,
            axle_z,
            ground_z,
            z_s,
            z_wb,
            z_t,
            rail_top,
            y_in,
            y_out,
            y_wi,
            y_wo,
            x0,
            x1,
            xu0: xc - 0.5 * p.upper_len,
            xu1: xc + 0.5 * p.upper_len,
            arc_c,
            r_boss,
        }
    }
}

pub struct Frame {
    pub params: FrameParams,
    pub layout: Layout,
    pub upper: Solid,
    pub lower: Solid,
    /// Ties across the walls' feet (moving with the upper), if any.
    pub ties: Option<Solid>,
    pub axles: Solid,
    pub spacers: Solid,
    pub bolt: Solid,
    pub nut: Solid,
    pub bushing: Solid,
    /// Wheels, for display and clash checks only.
    pub wheels: Solid,
}

// ── 2D helpers in the (x, z) plane ──────────────────────────────────────

fn box2(x: f64, z: f64, x0: f64, x1: f64, z0: f64, z1: f64) -> f64 {
    let dx = (x - 0.5 * (x0 + x1)).abs() - 0.5 * (x1 - x0);
    let dz = (z - 0.5 * (z0 + z1)).abs() - 0.5 * (z1 - z0);
    dx.max(0.0).hypot(dz.max(0.0)) + dx.max(dz).min(0.0)
}

/// Distance to the segment a-b, minus a half-width.
fn seg2(x: f64, z: f64, a: (f64, f64), b: (f64, f64), half_w: f64) -> f64 {
    let (px, pz) = (x - a.0, z - a.1);
    let (vx, vz) = (b.0 - a.0, b.1 - a.1);
    let h = ((px * vx + pz * vz) / (vx * vx + vz * vz)).clamp(0.0, 1.0);
    (px - h * vx).hypot(pz - h * vz) - half_w
}

fn circle2(x: f64, z: f64, cx: f64, cz: f64, r: f64) -> f64 {
    (x - cx).hypot(z - cz) - r
}

/// Polynomial smooth minimum: a fillet of radius ~k where two members meet.
fn smin(a: f64, b: f64, k: f64) -> f64 {
    if k <= 0.0 {
        return a.min(b);
    }
    let h = (k - (a - b).abs()).max(0.0) / k;
    a.min(b) - h * h * k * 0.25
}

/// A Warren truss in the (x, z) plane: a top chord box, a bottom chord box, bottom nodes at
/// `nodes` (x, on the bottom chord's centre line), diagonals from each bottom node to the top
/// chord half-way to the next, end diagonals to the top chord's ends, and optional posts over
/// every bottom node.
#[derive(Clone)]
struct Warren {
    top: [f64; 4],
    bot: [f64; 4],
    nodes: Vec<f64>,
    diag_w: f64,
    post_w: f64,
    fillet: f64,
}

impl Warren {
    fn eval(&self, x: f64, z: f64) -> f64 {
        let [tx0, tx1, tz0, tz1] = self.top;
        let [bx0, bx1, bz0, bz1] = self.bot;
        let k = self.fillet;
        let mut d = box2(x, z, bx0, bx1, bz0, bz1);
        if tz1 <= tz0 {
            return d;
        }
        d = smin(d, box2(x, z, tx0, tx1, tz0, tz1), k);
        let zt = 0.5 * (tz0 + tz1);
        let zb = 0.5 * (bz0 + bz1);
        let r = 0.5 * self.diag_w;
        let n = &self.nodes;
        let last = n.len() - 1;
        d = smin(d, seg2(x, z, (n[0], zb), (tx0 + r, zt), r), k);
        d = smin(d, seg2(x, z, (n[last], zb), (tx1 - r, zt), r), k);
        for i in 0..last {
            let xm = 0.5 * (n[i] + n[i + 1]);
            d = smin(d, seg2(x, z, (n[i], zb), (xm, zt), r), k);
            d = smin(d, seg2(x, z, (xm, zt), (n[i + 1], zb), r), k);
        }
        if self.post_w > 0.0 {
            for &xn in n {
                d = smin(d, seg2(x, z, (xn, zb), (xn, zt), 0.5 * self.post_w), k);
            }
        }
        d
    }
}

fn slab(x0: f64, x1: f64, y0: f64, y1: f64, z0: f64, z1: f64) -> Solid {
    Solid::cuboid(Vector3::new(0.5 * (x1 - x0), 0.5 * (y1 - y0), 0.5 * (z1 - z0)))
        .translate(Vector3::new(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1)))
}

/// A cylinder along y, centred at (x, 0, z).
fn along_y(r: f64, len: f64, x: f64, z: f64) -> Solid {
    let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2);
    Solid::cylinder(r, 0.5 * len).rotate(rot).translate(Vector3::new(x, 0.0, z))
}

/// A vertical cylinder from z0 to z1 at (x, 0).
fn along_z(r: f64, x: f64, z0: f64, z1: f64) -> Solid {
    Solid::cylinder(r, 0.5 * (z1 - z0)).translate(Vector3::new(x, 0.0, 0.5 * (z0 + z1)))
}

/// A profile in the (x, z) plane extruded over both |y| in [y0, y1] (mirrored pair).
fn extrude_pair(profile: impl Fn(f64, f64) -> f64 + Send + Sync + 'static, y0: f64, y1: f64, bounds_xz: [f64; 4]) -> Solid {
    let (ym, hw) = (0.5 * (y0 + y1), 0.5 * (y1 - y0));
    let [x0, x1, z0, z1] = bounds_xz;
    Solid::user_fn(
        move |p: Point3<f64>| profile(p.x, p.z).max((p.y.abs() - ym).abs() - hw),
        Aabb::new(Point3::new(x0 - 1.0, -y1 - 1.0, z0 - 1.0), Point3::new(x1 + 1.0, y1 + 1.0, z1 + 1.0)),
    )
}

/// Height of the rocker arc above the ledge at distance `dx` from its lowest point (exact circle).
pub fn arc_rise(r: f64, dx: f64) -> f64 {
    r - (r * r - dx * dx).max(0.0).sqrt()
}

/// `sole_bottom`: z of the boot's flat sole (the upper's top), mm; `length`: foot length, mm.
pub fn build(sole_bottom: f64, length: f64, p: FrameParams) -> Frame {
    let l = Layout::new(sole_bottom, length, &p);
    let r_rock = p.rocker_r;

    // ── Lower: rails ────────────────────────────────────────────────────
    // The bottom chord is tall (up to the ledge) only where the ledge is; elsewhere it just
    // joins the bosses.
    let rail = Warren {
        top: [l.axle_x[0] - 0.5 * p.post_w, l.axle_x[3] + 0.5 * p.post_w, l.rail_top - p.top_h, l.rail_top],
        bot: [l.axle_x[0], l.axle_x[3], l.axle_z + p.bot_z0, l.axle_z + p.bot_h_out],
        nodes: l.axle_x.to_vec(),
        diag_w: p.diag_w,
        post_w: p.post_w,
        fillet: p.fillet,
    };
    let (axle_x, axle_z, r_boss, r_bore) = (l.axle_x, l.axle_z, l.r_boss, 0.5 * p.axle_d);
    let (xc, z_t, rail_top, k) = (l.xc, l.z_t, l.rail_top, p.fillet);
    let ledge_x = (l.xc - 0.5 * p.ledge_len, l.xc + 0.5 * p.ledge_len);
    let bot_z0 = l.axle_z + p.bot_z0;
    // A tower in each rail at the centre carries the cross-member: as tall as the cross-member
    // when there is a top chord, down to the ledge when there isn't.
    let ring_r = 0.5 * p.bushing_od + 0.2 + p.cm_wall;
    let tower_half = ring_r;
    let tower_z0 = if p.top_h > 0.0 { l.rail_top - p.cm_h } else { l.z_t - 1.0 };
    let rail_profile = move |x: f64, z: f64| {
        let mut d = rail.eval(x, z);
        d = smin(d, box2(x, z, ledge_x.0, ledge_x.1, bot_z0, z_t), k);
        d = smin(d, box2(x, z, xc - tower_half, xc + tower_half, tower_z0, rail_top), k);
        for &ax in &axle_x {
            d = smin(d, circle2(x, z, ax, axle_z, r_boss), k);
        }
        for &ax in &axle_x {
            d = d.max(-circle2(x, z, ax, axle_z, r_bore));
        }
        d
    };
    let rails = extrude_pair(rail_profile, l.y_in, l.y_out, [l.x0, l.x1, axle_z - r_boss, l.rail_top]);
    let mut lower = rails
        .union(slab(ledge_x.0, ledge_x.1, l.y_out - 0.5, l.y_out + p.ledge_w, l.z_t - p.ledge_t, l.z_t))
        .union(slab(ledge_x.0, ledge_x.1, -l.y_out - p.ledge_w, -l.y_out + 0.5, l.z_t - p.ledge_t, l.z_t));
    // Cross-member between the rails, in the gap between the middle wheels: a ring round the
    // bushing bore and an arm out to each rail.
    let cm_bottom = l.rail_top - p.cm_h;
    lower = lower
        .union(along_z(ring_r, l.xc, cm_bottom, l.rail_top))
        .union(slab(l.xc - 0.5 * p.cm_arm, l.xc + 0.5 * p.cm_arm, -l.y_in - 0.5, l.y_in + 0.5, cm_bottom, l.rail_top));
    lower = lower
        .subtract(along_z(0.5 * p.bushing_od + 0.2, l.xc, l.rail_top - p.bushing_len, l.rail_top + 1.0))
        .subtract(along_z(0.5 * p.bolt_d + 1.0, l.xc, cm_bottom - 1.0, l.rail_top));

    // ── Upper: web, walls (rocker arcs at their feet), bolt boss ──────────
    let arc_c = l.arc_c;
    let half_u = 0.5 * p.upper_len;
    let foot_top = l.z_t + 0.1 + arc_rise(r_rock, half_u) + p.wall_bot_h;
    let n = p.wall_panels.max(1);
    let x_first = l.xu0 + 0.5 * p.wall_diag_w;
    let x_last = l.xu1 - 0.5 * p.wall_diag_w;
    let wall = Warren {
        top: [l.xu0, l.xu1, l.z_wb - p.wall_top_h, l.z_s],
        bot: [l.xu0, l.xu1, l.z_t - 1.0, foot_top],
        nodes: (0..=n).map(|i| x_first + (x_last - x_first) * i as f64 / n as f64).collect(),
        diag_w: p.wall_diag_w,
        post_w: 0.0,
        fillet: p.fillet,
    };
    let wall_profile = move |x: f64, z: f64| {
        // Inside the drum = above the arc.
        let above_arc = (x - arc_c.x).hypot(z - arc_c.z) - r_rock;
        wall.eval(x, z).max(above_arc)
    };
    let walls = extrude_pair(wall_profile, l.y_wi, l.y_wo, [l.xu0, l.xu1, l.z_t - 1.0, l.z_s]);
    let web = slab(l.xu0, l.xu1, -l.y_wo, l.y_wo, l.z_wb, l.z_s);
    let boss_half = 9.0;
    let boss = slab(l.xc - boss_half, l.xc + boss_half, -l.y_in + 1.0, l.y_in - 1.0, l.rail_top + 1.0, l.z_wb + 0.5);
    let mut upper = web.union(walls).union(boss);
    // Web windows between the walls, each side of the bolt boss.
    let win_y = l.y_wi - 1.5;
    let side_len = half_u - p.web_rib - boss_half - 0.5 * p.web_rib;
    if p.web_windows > 0 && side_len > 10.0 {
        let m = p.web_windows as f64;
        let w = (side_len - (m - 1.0) * p.web_rib) / m;
        for side in [-1.0, 1.0] {
            for i in 0..p.web_windows {
                let a = boss_half + 0.5 * p.web_rib + i as f64 * (w + p.web_rib);
                let (xa, xb) = if side > 0.0 { (l.xc + a, l.xc + a + w) } else { (l.xc - a - w, l.xc - a) };
                let cut = Solid::cuboid(Vector3::new(0.5 * w - 3.0, win_y - 3.0, p.web_t))
                    .round(3.0)
                    .translate(Vector3::new(0.5 * (xa + xb), 0.0, l.z_wb + 0.5 * p.web_t));
                upper = upper.subtract(cut);
            }
        }
    }
    // Bolt hole and a head recess (countersunk head, as a 11 x 3.5 mm pocket) at the top.
    upper = upper
        .subtract(along_z(0.5 * p.bolt_d + 0.1, l.xc, l.rail_top, l.z_s + 1.0))
        .subtract(along_z(5.5, l.xc, l.z_s - 3.5, l.z_s + 1.0));

    // Ties across the walls' feet at the outer wheel gaps.
    let pitch = p.wheel_d + p.wheel_gap;
    let mut ties: Option<Solid> = None;
    if p.tie_d > 0.0 {
        for dx in [-pitch, pitch] {
            let z = l.z_t + 0.1 + arc_rise(r_rock, dx) + 0.5 * p.wall_bot_h;
            let rod = along_y(0.5 * p.tie_d, 2.0 * l.y_wo, l.xc + dx, z);
            ties = Some(match ties {
                None => rod,
                Some(s) => s.union(rod),
            });
        }
    }

    // ── Axles, spacers, coupling ────────────────────────────────────────
    let mut axles: Option<Solid> = None;
    let mut spacers: Option<Solid> = None;
    for &ax in &l.axle_x {
        let a = along_y(r_bore, 2.0 * l.y_out, ax, l.axle_z)
            .union(along_y(5.5, 2.0, ax, l.axle_z).translate(Vector3::new(0.0, l.y_out + 1.0, 0.0)))
            .union(along_y(5.5, 2.0, ax, l.axle_z).translate(Vector3::new(0.0, -l.y_out - 1.0, 0.0)));
        axles = Some(match axles {
            None => a,
            Some(s) => s.union(a),
        });
        // Bearing spacer between the two 608 bearings (7 mm each), and a side washer each side.
        let inner = 0.5 * p.wheel_w - 7.0;
        let s = along_y(5.25, 2.0 * inner, ax, l.axle_z)
            .union(along_y(6.0, p.spacer, ax, l.axle_z).translate(Vector3::new(0.0, 0.5 * p.wheel_w + 0.5 * p.spacer, 0.0)))
            .union(along_y(6.0, p.spacer, ax, l.axle_z).translate(Vector3::new(0.0, -0.5 * p.wheel_w - 0.5 * p.spacer, 0.0)))
            .subtract(along_y(r_bore + 0.05, p.wheel_w + 4.0 * p.spacer, ax, l.axle_z));
        spacers = Some(match spacers {
            None => s,
            Some(t) => t.union(s),
        });
    }
    let nut_top = cm_bottom;
    let nut = along_z(5.25, l.xc, nut_top - 5.0, nut_top).subtract(along_z(0.5 * p.bolt_d, l.xc, nut_top - 6.0, nut_top + 1.0));
    let bolt = along_z(0.5 * p.bolt_d, l.xc, nut_top - 6.0, l.z_s - 3.5).union(along_z(5.5, l.xc, l.z_s - 3.5, l.z_s - 0.05));
    let bushing = along_z(0.5 * p.bushing_od, l.xc, l.rail_top - p.bushing_len, l.rail_top)
        .subtract(along_z(0.5 * p.bolt_d + 0.25, l.xc, l.rail_top - p.bushing_len - 1.0, l.rail_top + 1.0));

    let mut wheels: Option<Solid> = None;
    for &ax in &l.axle_x {
        let w = along_y(0.5 * p.wheel_d, p.wheel_w, ax, l.axle_z);
        wheels = Some(match wheels {
            None => w,
            Some(s) => s.union(w),
        });
    }

    Frame {
        params: p,
        layout: l,
        upper,
        lower,
        ties,
        axles: axles.expect("four axles"),
        spacers: spacers.expect("four spacer sets"),
        bolt,
        nut,
        bushing,
        wheels: wheels.expect("four wheels"),
    }
}

/// The upper (with its ties and bolt) rolled so the contact sits `e` mm from the centre:
/// rotated about the arc centre by e / r, then moved along by e (rolling without slip).
pub fn rolled(solid: &Solid, l: &Layout, r: f64, e: f64) -> Solid {
    let phi = e / r;
    let c = l.arc_c.coords;
    solid
        .clone()
        .translate(-c)
        .rotate(UnitQuaternion::from_axis_angle(&Vector3::y_axis(), phi))
        .translate(c + Vector3::new(e, 0.0, 0.0))
}

/// A one-piece frame built from the same members, as a like-for-like baseline (PLAN phase 1):
/// two Warren-truss rails from the axle bosses up to the boot sole (same thickness, diagonals,
/// posts, top chord and bosses as the round-2 lower), a plain bottom chord round the bosses
/// (no ledge), and two mount platforms (heel and toe, `mount_spacing` apart, `mount_len` long,
/// `mount_t` thick) across the rails under the sole, each with a 6.5 mm bolt hole. Same wheels,
/// stack, axles and spacers as the rocker, so the difference in grams is what the rocker costs.
pub fn one_piece(sole_bottom: f64, length: f64, p: &FrameParams, mount_spacing: f64, mount_len: f64, mount_t: f64) -> Solid {
    let l = Layout::new(sole_bottom, length, p);
    let top = l.z_s - mount_t;
    let rail = Warren {
        top: [l.axle_x[0] - 0.5 * p.post_w, l.axle_x[3] + 0.5 * p.post_w, top - p.top_h, top],
        bot: [l.axle_x[0], l.axle_x[3], l.axle_z + p.bot_z0, l.axle_z + 6.0],
        nodes: l.axle_x.to_vec(),
        diag_w: p.diag_w,
        post_w: p.post_w,
        fillet: p.fillet,
    };
    let (axle_x, axle_z, r_boss, r_bore, k) = (l.axle_x, l.axle_z, l.r_boss, 0.5 * p.axle_d, p.fillet);
    let profile = move |x: f64, z: f64| {
        let mut d = rail.eval(x, z);
        for &ax in &axle_x {
            d = smin(d, circle2(x, z, ax, axle_z, r_boss), k);
        }
        for &ax in &axle_x {
            d = d.max(-circle2(x, z, ax, axle_z, r_bore));
        }
        d
    };
    let mut body = extrude_pair(profile, l.y_in, l.y_out, [l.x0, l.x1, axle_z - r_boss, top]);
    for side in [-1.0, 1.0] {
        let xm = l.xc + side * 0.5 * mount_spacing;
        body = body
            .union(slab(xm - 0.5 * mount_len, xm + 0.5 * mount_len, -l.y_out, l.y_out, top - 0.5, l.z_s))
            .subtract(along_z(3.25, xm, top - 1.0, l.z_s + 1.0));
    }
    body
}
