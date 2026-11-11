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
//! - Option `vee` (step 2, "less material"): the walls lean out along the line from the wheels'
//!   ground contact through their feet, and the ledges tilt square to that line. In front view
//!   the two walls then meet at the contact patch, so the ground force runs along them instead of
//!   bending them. Options `web_truss` (the web as a truss in plan) and `deck_panels` (a truss
//!   deck across the lower's rails) carry the push lengthwise to the one place a tilted ledge
//!   can take it: the rolling contact.
//! - Option `rib_d` (step 2, upright walls): a rib along every truss member of the walls, on the
//!   outer face, so each member is a T-section against the sideways push (`rib_t`, `rib_taper`,
//!   `rib_chords` choose its thickness, taper and which chords get one).
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
    /// V section (round 2 step 2, "less material"): the walls lean out along the line from the
    /// wheels' ground contact through their feet, and the ledges tilt square to that line, so in
    /// front view the push runs along the walls instead of bending them. False = upright walls.
    pub vee: bool,
    /// The web as a Warren truss in plan (this many panels; 0 = the windowed web).
    pub web_truss: usize,
    /// A Warren-truss deck in plan between the lower's rails, at their top (this many panels,
    /// 0 = none), `deck_t` thick: carries the sideways push along the lower to the contact.
    pub deck_panels: usize,
    pub deck_t: f64,
    /// Bulkheads across the lower's rails in the gaps between wheels, this thick (0 = none).
    pub bulk_t: f64,
    /// Ribs on the upright walls' outer faces, one along every truss member: depth at the top
    /// chord (0 = none), thickness, and depth at the foot chord as a fraction of it.
    pub rib_d: f64,
    pub rib_t: f64,
    pub rib_taper: f64,
    /// Which chords get ribs too: 0 = none (diagonals only), 1 = the foot chord, 2 = both.
    pub rib_chords: usize,
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
            vee: false,
            web_truss: 0,
            deck_panels: 0,
            deck_t: 2.0,
            bulk_t: 0.0,
            rib_d: 0.0,
            rib_t: 2.0,
            rib_taper: 1.0,
            rib_chords: 2,
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
    /// Centre of the rocker arc at rest (in the wall's own plane: x and `up_wall`).
    pub arc_c: Point3<f64>,
    pub r_boss: f64,
    /// The walls' mid-line in the (|y|, z) plane: through (`y_wc`, `z_t`), leaning out by an
    /// angle with this sine and cosine (0 and 1 for upright walls); half the wall thickness.
    pub y_wc: f64,
    pub v_sin: f64,
    pub v_cos: f64,
    pub half_t: f64,
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
        // V section: the mid-line through the wall foot also passes through the wheels' ground
        // contact (y = 0, ground), so any ground force is carried along the two walls.
        let y_wc = 0.5 * (y_wi + y_wo);
        let lean = if p.vee { (y_wc / (z_t - ground_z)).atan() } else { 0.0 };
        let (v_sin, v_cos) = lean.sin_cos();
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
            y_wc,
            v_sin,
            v_cos,
            half_t: 0.5 * p.wall_t,
        }
    }

    /// Signed distance from the wall's mid-line across the wall (+ outward), mirrored in y.
    pub fn across(&self, y: f64, z: f64) -> f64 {
        (y.abs() - self.y_wc) * self.v_cos - (z - self.z_t) * self.v_sin
    }

    /// Signed distance along the wall's mid-line from the ledge top's plane (+ up), mirrored in y.
    pub fn along(&self, y: f64, z: f64) -> f64 {
        (y.abs() - self.y_wc) * self.v_sin + (z - self.z_t) * self.v_cos
    }

    /// Height along the wall's mid-line, as the z it has on the line (= z for upright walls):
    /// the coordinate the wall's (x, z) profile is drawn in.
    pub fn up_wall(&self, y: f64, z: f64) -> f64 {
        self.z_t + self.along(y, z) * self.v_cos
    }

    /// |y| of the wall's outer and inner faces at height z.
    pub fn wall_out(&self, z: f64) -> f64 {
        self.y_wc + (z - self.z_t) * self.v_sin / self.v_cos + self.half_t / self.v_cos
    }
    pub fn wall_in(&self, z: f64) -> f64 {
        self.y_wc + (z - self.z_t) * self.v_sin / self.v_cos - self.half_t / self.v_cos
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
    let mut lower = if p.vee {
        // Tilted ledges: the top square to the walls' mid-line, from inside the rail out past the
        // wall's outer face by the same margin as the flat ledge.
        let (lv, ledge_t, xh) = (l, p.ledge_t, 0.5 * p.ledge_len);
        let reach = l.half_t + (p.ledge_w - p.side_gap - p.wall_t).max(0.5);
        let y_rail = l.y_out - 0.5;
        let y_max = l.wall_out(l.z_t) + reach + 2.0;
        rails.union(Solid::user_fn(
            move |q: Point3<f64>| {
                let dz = (lv.along(q.y, q.z) + 0.5 * ledge_t).abs() - 0.5 * ledge_t;
                dz.max(y_rail - q.y.abs()).max(lv.across(q.y, q.z) - reach).max((q.x - lv.xc).abs() - xh)
            },
            Aabb::new(Point3::new(l.xc - xh - 1.0, -y_max, l.z_t - ledge_t - 4.0), Point3::new(l.xc + xh + 1.0, y_max, l.z_t + 3.0)),
        ))
    } else {
        rails
            .union(slab(ledge_x.0, ledge_x.1, l.y_out - 0.5, l.y_out + p.ledge_w, l.z_t - p.ledge_t, l.z_t))
            .union(slab(ledge_x.0, ledge_x.1, -l.y_out - p.ledge_w, -l.y_out + 0.5, l.z_t - p.ledge_t, l.z_t))
    };
    // Cross-member between the rails, in the gap between the middle wheels: a ring round the
    // bushing bore and an arm out to each rail.
    let cm_bottom = l.rail_top - p.cm_h;
    lower = lower
        .union(along_z(ring_r, l.xc, cm_bottom, l.rail_top))
        .union(slab(l.xc - 0.5 * p.cm_arm, l.xc + 0.5 * p.cm_arm, -l.y_in - 0.5, l.y_in + 0.5, cm_bottom, l.rail_top));
    // Deck: a Warren truss in plan across the rails' tops, over the wheels (2 mm clear at the
    // defaults), so the two rails act together as the flanges of one beam in plan.
    if p.deck_panels > 0 {
        let n = p.deck_panels;
        let r = 0.5 * p.diag_w;
        let (xa, xb) = (l.axle_x[0], l.axle_x[3]);
        let deck = Warren {
            top: [xa, xb, l.y_in - 1.0, l.y_out],
            bot: [xa, xb, -l.y_out, -(l.y_in - 1.0)],
            nodes: (0..=n).map(|i| xa + r + (xb - xa - 2.0 * r) * i as f64 / n as f64).collect(),
            diag_w: p.diag_w,
            post_w: 0.0,
            fillet: p.fillet,
        };
        let (z0, z1) = (l.rail_top - p.deck_t, l.rail_top);
        lower = lower.union(Solid::user_fn(
            move |q: Point3<f64>| deck.eval(q.x, q.y).max((q.z - 0.5 * (z0 + z1)).abs() - 0.5 * (z1 - z0)),
            Aabb::new(Point3::new(xa - 1.0, -l.y_out - 1.0, z0 - 1.0), Point3::new(xb + 1.0, l.y_out + 1.0, z1 + 1.0)),
        ));
    }
    // Bulkheads: plates across the rails in the three gaps between wheels, `bulk_t` thick,
    // clear of the wheels by `BULK_CLEAR` (the gap is narrowest at axle height); the middle one
    // stops below the coupling's nut. They keep the rails' section square (no racking).
    if p.bulk_t > 0.0 {
        const BULK_CLEAR: f64 = 0.75;
        let pitch = p.wheel_d + p.wheel_gap;
        let z0 = l.axle_z + p.bot_z0;
        for (gx, z1) in [(l.xc - pitch, l.rail_top), (l.xc, cm_bottom - 7.0), (l.xc + pitch, l.rail_top)] {
            let mut plate = slab(gx - 0.5 * p.bulk_t, gx + 0.5 * p.bulk_t, -l.y_in - 0.5, l.y_in + 0.5, z0, z1);
            for &ax in &l.axle_x {
                if (ax - gx).abs() < pitch {
                    plate = plate.subtract(along_y(0.5 * p.wheel_d + BULK_CLEAR, p.wheel_w + 2.0, ax, l.axle_z));
                }
            }
            // The upper's ties pass the outer gaps: a hole with room for the rocking (~2 mm).
            if p.tie_d > 0.0 && gx != l.xc {
                let z_tie = l.z_t + 0.1 + arc_rise(r_rock, gx - l.xc) + 0.5 * p.wall_bot_h;
                plate = plate.subtract(along_y(0.5 * p.tie_d + 2.5, 2.0 * l.y_out + 2.0, gx, z_tie));
            }
            lower = lower.union(plate);
        }
    }
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
        // A leaned wall's top is cut off at the sole by z itself, so its chord runs past it.
        top: [l.xu0, l.xu1, l.z_wb - p.wall_top_h, if p.vee { l.z_s + 1.0 } else { l.z_s }],
        bot: [l.xu0, l.xu1, l.z_t - 1.0, foot_top],
        nodes: (0..=n).map(|i| x_first + (x_last - x_first) * i as f64 / n as f64).collect(),
        diag_w: p.wall_diag_w,
        post_w: 0.0,
        fillet: p.fillet,
    };
    // Ribs (upright walls only): every truss member of the wall gets a rib `rib_t` thick standing
    // out from its outer face, so each member is a T-section bending out of the wall's plane.
    // `rib_d` deep at the top chord (the wall's root, where the bending moment is largest),
    // tapering linearly to `rib_d * rib_taper` at the foot chord.
    let ribs = (p.rib_d > 0.0 && !p.vee).then(|| {
        let rt = 0.5 * p.rib_t;
        let (tc, bc) = (0.5 * (l.z_wb - p.wall_top_h + l.z_s), 0.5 * (l.z_t - 1.0 + foot_top));
        let rib = Warren {
            top: [l.xu0, l.xu1, tc - rt, tc + rt],
            bot: [l.xu0, l.xu1, bc - rt, bc + rt],
            nodes: wall.nodes.clone(),
            diag_w: p.rib_t,
            post_w: 0.0,
            fillet: p.fillet,
        };
        let (y_wo, rib_d, taper, chords) = (l.y_wo, p.rib_d, p.rib_taper, p.rib_chords);
        Solid::user_fn(
            move |q: Point3<f64>| {
                let above_arc = (q.x - arc_c.x).hypot(q.z - arc_c.z) - r_rock;
                let s = ((q.z - bc) / (tc - bc)).clamp(0.0, 1.0);
                let depth = rib_d * (taper + (1.0 - taper) * s);
                let ya = q.y.abs();
                let mut d = rib.eval(q.x, q.z).max(above_arc).max(y_wo - 0.5 - ya).max(ya - y_wo - depth);
                // Leave the chords' own ribs off (the diagonals' ribs still reach the chords).
                if chords < 2 {
                    d = d.max(rt + 0.5 - (q.z - tc).abs());
                }
                if chords < 1 {
                    d = d.max(rt + 0.5 - (q.z - bc).abs());
                }
                d
            },
            Aabb::new(Point3::new(l.xu0 - 1.0, -l.y_wo - p.rib_d - 1.0, l.z_t - 1.0), Point3::new(l.xu1 + 1.0, l.y_wo + p.rib_d + 1.0, l.z_s)),
        )
    });
    let wall_profile = move |x: f64, z: f64| {
        // Inside the drum = above the arc.
        let above_arc = (x - arc_c.x).hypot(z - arc_c.z) - r_rock;
        wall.eval(x, z).max(above_arc)
    };
    let walls = if p.vee {
        // The wall's profile drawn along its leaning mid-line; its foot is then square to the
        // line, flat on the tilted ledge.
        let (lv, z_s) = (l, l.z_s);
        let y_max = l.wall_out(l.z_s) + 1.0;
        Solid::user_fn(
            move |q: Point3<f64>| wall_profile(q.x, lv.up_wall(q.y, q.z)).max(lv.across(q.y, q.z).abs() - lv.half_t).max(q.z - z_s),
            Aabb::new(Point3::new(l.xu0 - 1.0, -y_max, l.z_t - 3.0), Point3::new(l.xu1 + 1.0, y_max, l.z_s + 1.0)),
        )
    } else {
        extrude_pair(wall_profile, l.y_wi, l.y_wo, [l.xu0, l.xu1, l.z_t - 1.0, l.z_s])
    };
    let walls = match ribs {
        Some(r) => walls.union(r),
        None => walls,
    };
    // The web spans the walls' tops; its windows (or truss) stop at the walls' inner faces.
    let (y_web, y_web_in) = if p.vee { (l.wall_out(l.z_s), l.wall_in(l.z_wb)) } else { (l.y_wo, l.y_wi) };
    let boss_half = 9.0;
    let web = if p.web_truss > 0 {
        // Warren truss in plan: chords along the walls (1.5 mm inside them, like the windowed
        // web's edges), diagonals `web_rib` wide, a solid strip across round the bolt.
        let n = p.web_truss;
        let r = 0.5 * p.web_rib;
        let chord_in = y_web_in - 1.5;
        let truss = Warren {
            top: [l.xu0, l.xu1, chord_in, y_web],
            bot: [l.xu0, l.xu1, -y_web, -chord_in],
            nodes: (0..=n).map(|i| l.xu0 + r + (l.xu1 - l.xu0 - 2.0 * r) * i as f64 / n as f64).collect(),
            diag_w: p.web_rib,
            post_w: 0.0,
            fillet: p.fillet,
        };
        let (xc, z0, z1, pad, k) = (l.xc, l.z_wb, l.z_s, boss_half + 0.5 * p.web_rib, p.fillet);
        Solid::user_fn(
            move |q: Point3<f64>| {
                let plan = smin(truss.eval(q.x, q.y), (q.x - xc).abs() - pad, k);
                plan.max(q.y.abs() - y_web).max((q.z - 0.5 * (z0 + z1)).abs() - 0.5 * (z1 - z0))
            },
            Aabb::new(Point3::new(l.xu0 - 1.0, -y_web - 1.0, z0 - 1.0), Point3::new(l.xu1 + 1.0, y_web + 1.0, z1 + 1.0)),
        )
    } else {
        slab(l.xu0, l.xu1, -y_web, y_web, l.z_wb, l.z_s)
    };
    let boss = slab(l.xc - boss_half, l.xc + boss_half, -l.y_in + 1.0, l.y_in - 1.0, l.rail_top + 1.0, l.z_wb + 0.5);
    let mut upper = web.union(walls).union(boss);
    // Web windows between the walls, each side of the bolt boss.
    let win_y = y_web_in - 1.5;
    let side_len = half_u - p.web_rib - boss_half - 0.5 * p.web_rib;
    if p.web_truss == 0 && p.web_windows > 0 && side_len > 10.0 {
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
            let rod = along_y(0.5 * p.tie_d, 2.0 * if p.vee { l.wall_out(z) } else { l.y_wo }, l.xc + dx, z);
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
