//! `chassis [foot length mm] [out dir]`: the round-2 roller chassis (`frame.rs`) on its own:
//! weighed part by part, checked for clashes at rest and rocked to both ends of its travel,
//! checked by hand formulas (`strength.rs`), rendered, summarised in `summary.txt`. Takes
//! seconds, not the 2.5 minutes of the whole-skate round, so the frame can be iterated.

use std::fmt::Write as _;
use std::path::Path;
use std::time::Instant;

use cortenforge::cf_design::Solid;
use cortenforge::cf_design::mechanism::mass::mass_properties;
use cortenforge::mesh::io::save_mesh;
use nalgebra::Vector3;

use crate::frame::{self, Frame, FrameParams, arc_rise, rolled};
use crate::materials::{AL_6061_T6, AL_7075_T6, AL_7075_T6_STRENGTH, Material, PU_BUSHING, TI_6AL_4V};
use crate::render::{Layer, View, render, standard_views};
use crate::round1::mesh_of;
use crate::strength::{self, LOADS, allowables, section};

/// Foot length of round 1's stand-in (022_fBW_R), mm.
pub const STAND_IN_LENGTH: f64 = 252.4;

fn grams(solid: &Solid, mat: &Material, cell: f64) -> f64 {
    mass_properties(solid, mat.density, cell).map_or(0.0, |m| m.mass * 1000.0)
}

/// Volume (mm3) where two parts truly overlap: grid cells whose centres are more than 0.05 mm
/// inside both. (`mass_properties` weights cells within half a cell of the surface, so two parts
/// 0.1 mm apart, like the rocker on its ledge, read as a few mm3 of "overlap" at 0.25 mm cells.)
pub fn overlap_mm3(a: &Solid, b: &Solid) -> f64 {
    let both = a.clone().intersect(b.clone());
    let Some(bb) = both.bounds() else { return 0.0 };
    let cell = 0.25;
    let n = |lo: f64, hi: f64| ((hi - lo) / cell).ceil().max(0.0) as usize;
    let (nx, ny, nz) = (n(bb.min.x, bb.max.x), n(bb.min.y, bb.max.y), n(bb.min.z, bb.max.z));
    let mut count = 0usize;
    for iz in 0..nz {
        let z = bb.min.z + (iz as f64 + 0.5) * cell;
        for iy in 0..ny {
            let y = bb.min.y + (iy as f64 + 0.5) * cell;
            for ix in 0..nx {
                let x = bb.min.x + (ix as f64 + 0.5) * cell;
                if both.evaluate(&nalgebra::Point3::new(x, y, z)) < -0.05 {
                    count += 1;
                }
            }
        }
    }
    count as f64 * cell * cell * cell
}

struct Report(String);

impl Report {
    fn line(&mut self, s: impl AsRef<str>) {
        println!("{}", s.as_ref());
        let _ = writeln!(self.0, "{}", s.as_ref());
    }
}

/// The upper with what moves with it (ties, bolt).
fn moving(f: &Frame) -> Solid {
    let mut m = f.upper.clone().union(f.bolt.clone());
    if let Some(t) = &f.ties {
        m = m.union(t.clone());
    }
    m
}

pub fn run(length: f64, out: &Path) -> Result<(), String> {
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let t0 = Instant::now();
    let p = FrameParams::default();
    let f = frame::build(0.0, length, p);
    let l = f.layout;
    let mut r = Report(String::new());
    r.line(format!("round-2 roller chassis, foot length {length:.1} mm (z = 0 at the boot sole)"));
    r.line(format!(
        "  4 x {} mm wheels, pitch {} mm; rocker radius {:.0} mm ({:.1} ft); travel +-{} mm = +-{:.2} deg of foot pitch",
        p.wheel_d,
        p.wheel_d + p.wheel_gap,
        p.rocker_r,
        p.rocker_r / 304.8,
        p.travel,
        (p.travel / p.rocker_r).to_degrees()
    ));
    r.line(format!(
        "  stack: ground {:.1} mm below the sole (round 1: 85.0); rocker tracks {:.1} mm above the ground, {:.1} mm apart",
        l.z_s - l.ground_z,
        l.z_t - l.ground_z,
        2.0 * (l.y_wi + 0.5 * p.wall_t)
    ));

    // ── Mass ────────────────────────────────────────────────────────────
    let t = Instant::now();
    let mut parts: Vec<(&str, &Solid, &Material)> = vec![("upper (web, rocker walls)", &f.upper, &AL_7075_T6)];
    if let Some(ties) = &f.ties {
        parts.push(("ties x2", ties, &AL_7075_T6));
    }
    parts.extend([
        ("lower (rails, ledges, cross-member)", &f.lower, &AL_7075_T6),
        ("axles x4", &f.axles, &AL_7075_T6),
        ("spacers (bearing + side) x4", &f.spacers, &AL_6061_T6),
        ("bolt", &f.bolt, &TI_6AL_4V),
        ("nut", &f.nut, &TI_6AL_4V),
        ("bushing", &f.bushing, &PU_BUSHING),
    ]);
    r.line("mass (0.25 mm cells):");
    let mut total = 0.0;
    let mut running_gear = 0.0;
    for (name, solid, mat) in &parts {
        let g = grams(solid, mat, 0.25);
        total += g;
        if name.starts_with("axles") || name.starts_with("spacers") {
            running_gear += g;
        }
        r.line(format!("  {name:<36} {g:6.1} g  ({})", mat.name));
    }
    r.line(format!(
        "  total {total:.1} g  (round 1: 281.4 g without spacers or nut; one-piece 7075 frames 170-184 g with axles and spacers; Marsblade R1 282-302 g) ({:.1} s)",
        t.elapsed().as_secs_f64()
    ));
    // The same members as a one-piece frame: what the rocker costs, like for like.
    let (mount_spacing, mount_len, mount_t) = (165.0, 36.0, 4.0);
    let op = frame::one_piece(0.0, length, &p, mount_spacing, mount_len, mount_t);
    let op_g = grams(&op, &AL_7075_T6, 0.25);
    r.line(format!(
        "  one-piece from the same members ({mount_spacing} mm mounts {mount_len} x {mount_t} mm): frame {op_g:.1} g + axles and spacers {running_gear:.1} g = {:.1} g; the rocker adds {:.1} g",
        op_g + running_gear,
        total - op_g - running_gear
    ));

    // ── Clashes, at rest and rocked to both ends ─────────────────────────
    let t = Instant::now();
    let mov = moving(&f);
    r.line("clashes (overlap volume, mm3; 0 = clear):");
    for e in [-p.travel, 0.0, p.travel] {
        let up = rolled(&mov, &l, p.rocker_r, e);
        r.line(format!(
            "  rocked {e:+5.0} mm: upper x lower {:.1}, upper x wheels {:.1}",
            overlap_mm3(&up, &f.lower),
            overlap_mm3(&up, &f.wheels)
        ));
    }
    r.line(format!(
        "  wheels x lower {:.1}, wheels x bushing {:.1}, wheels x nut {:.1} ({:.1} s)",
        overlap_mm3(&f.wheels, &f.lower),
        overlap_mm3(&f.wheels, &f.bushing),
        overlap_mm3(&f.wheels, &f.nut),
        t.elapsed().as_secs_f64()
    ));

    // ── Strength by hand formulas ───────────────────────────────────────
    let t = Instant::now();
    let c = checks(&f, length)?;
    let (ps, pi) = (c.stride, c.impact);
    r.line(format!(
        "strength (hand formulas; loads assumed, see strength.rs): body {} kg, stride {} BW = {ps:.0} N (fatigue), impact {} BW = {pi:.0} N (static)",
        LOADS.body_mass, LOADS.stride, LOADS.impact
    ));
    r.line(format!(
        "  allowables {}: static {:.0} MPa (yield / 1.5), fatigue {:.0} MPa (R = 0 range by Goodman x 0.7)",
        AL_7075_T6_STRENGTH.name, c.allow_static, c.allow_fatigue
    ));
    let k = pi / ps;
    r.line(format!(
        "  upper bending: max {:.0} MPa stride / {:.0} MPa impact at x = {:+.0} (contact {:+.0}); heel x {:.0}, ball x {:.0}",
        c.upper.0, c.upper.0 * k, c.upper.1, c.upper.2, c.heel_ball.0, c.heel_ball.1
    ));
    r.line(format!(
        "  lower bending: max {:.0} MPa stride / {:.0} MPa impact at x = {:+.0} (contact {:+.0}); all wheels stay loaded for contact within +-{:.0} mm",
        c.lower.0, c.lower.0 * k, c.lower.1, c.lower.2, c.all_down
    ));
    r.line(format!(
        "  rocker wall feet between nodes ({:.0} mm panels): {:.0} / {:.0} MPa (stride / impact)",
        c.foot.1, c.foot.0, c.foot.0 * k
    ));
    r.line(format!(
        "  ledge + bottom chord between bosses ({:.0} mm): local {:.0} + global {:.0} = {:.0} / {:.0} MPa",
        c.chord.2, c.chord.0, c.chord.1, c.chord.0 + c.chord.1, (c.chord.0 + c.chord.1) * k
    ));
    for (name, (b, p0)) in [("stride", c.hertz_stride), ("impact", c.hertz_impact)] {
        r.line(format!(
            "  rocker contact ({name}): half-width {b:.1} mm, peak pressure {p0:.0} MPa (static limit ~{:.0} MPa = 1.67 x yield)",
            1.67 * AL_7075_T6_STRENGTH.yield_
        ));
    }
    for (name, w, sa, sb) in &c.axles {
        r.line(format!("  axle ({name}, wheel load {w:.0} N): bending {sa:.0} MPa (8 mm 7075, solid); boss bearing {sb:.0} MPa"));
    }
    // Round 1: a 30 mm wide rocker on top of the bridge, 79 mm above the ground.
    let round1_lam = strength::roll_ratio(30.0, 79.0);
    r.line(format!(
        "  roll: tracks {:.1} mm apart, {:.1} mm up: the vertical load alone holds lateral / vertical up to {:.2} (assumed {}; round 1's top rocker ~{round1_lam:.2})",
        c.roll.1, c.roll.2, c.roll.0, LOADS.lateral
    ));
    r.line(format!(
        "  lateral push at {}: {:.0} N stride; NOT checked yet: walls and rails in side bending, torsion, buckling (FEM next)",
        LOADS.lateral,
        LOADS.lateral * ps
    ));
    r.line(format!("  worst stress / allowable over the vertical cases: {:.2} ({:.1} s)", c.worst_ratio(), t.elapsed().as_secs_f64()));

    // ── Lean clearance ──────────────────────────────────────────────────
    let rho = 0.5 * p.wheel_w;
    let big_r = 0.5 * p.wheel_d;
    // Only what sits outside the wheel and its side spacers can touch the ground first.
    let outside = 0.5 * p.wheel_w + p.spacer - 0.01;
    let lean = |m: &cortenforge::mesh::types::IndexedMesh| {
        m.vertices
            .iter()
            .filter(|v| v.y.abs() > outside)
            .map(|v| strength::lean_limit(v.y, v.z - l.ground_z, big_r, rho))
            .fold(f64::INFINITY, f64::min)
    };
    let upper_m = mesh_of(&f.upper, 0.4);
    let lower_m = mesh_of(&f.lower, 0.4);
    let axles_m = mesh_of(&f.axles, 0.4);
    let coupling_m = mesh_of(&f.bolt.clone().union(f.nut.clone()).union(f.bushing.clone()), 0.4);
    let ties_m = f.ties.as_ref().map(|t| mesh_of(t, 0.4));
    let spacers_m = mesh_of(&f.spacers, 0.4);
    r.line(format!(
        "lean before touching the ground (tread radius {rho} mm assumed): axle heads {:.0} deg, lower {:.0} deg, upper {:.0} deg",
        lean(&axles_m),
        lean(&lower_m),
        lean(&upper_m)
    ));

    // ── Files ───────────────────────────────────────────────────────────
    let save = |name: &str, m: &cortenforge::mesh::types::IndexedMesh| save_mesh(m, &out.join(name)).map_err(|e| e.to_string());
    save("upper.stl", &upper_m)?;
    save("lower.stl", &lower_m)?;
    save("axles.stl", &axles_m)?;
    save("coupling.stl", &coupling_m)?;
    let wheels_m = mesh_of(&f.wheels, 1.0);
    let (c_up, c_lo, c_ax, c_cp, c_wh) = ([0.55, 0.6, 0.68], [0.72, 0.74, 0.78], [0.4, 0.4, 0.4], [0.85, 0.3, 0.25], [0.95, 0.75, 0.15]);
    let mut layers = vec![
        Layer::solid(&upper_m, c_up),
        Layer::solid(&lower_m, c_lo),
        Layer::solid(&axles_m, c_ax),
        Layer::solid(&spacers_m, c_ax),
        Layer::solid(&coupling_m, c_cp),
    ];
    if let Some(tm) = &ties_m {
        layers.push(Layer::solid(tm, c_cp));
    }
    render(&layers, &standard_views(), 3.0, &out.join("chassis.png")).map_err(|e| e.to_string())?;
    let mut with_wheels = layers.clone();
    with_wheels.push(Layer::solid(&wheels_m, c_wh));
    render(&with_wheels, &standard_views(), 2.5, &out.join("chassis_wheels.png")).map_err(|e| e.to_string())?;
    // Rocked to both ends, side view.
    let side = [View::looking(Vector3::y(), Vector3::z())];
    for (name, e) in [("rocked_heel.png", -p.travel), ("rocked_toe.png", p.travel)] {
        let up = mesh_of(&rolled(&mov, &l, p.rocker_r, e), 0.4);
        render(
            &[Layer::solid(&up, c_up), Layer::solid(&lower_m, c_lo), Layer::solid(&axles_m, c_ax), Layer::solid(&wheels_m, c_wh)],
            &side,
            3.0,
            &out.join(name),
        )
        .map_err(|e| e.to_string())?;
    }
    std::fs::write(out.join("summary.txt"), &r.0).map_err(|e| e.to_string())?;
    println!("wrote STL + PNG + summary.txt to {} ({:.1} s total)", out.display(), t0.elapsed().as_secs_f64());
    Ok(())
}

/// The hand-formula results for one frame. Stresses in MPa under the stride load (fatigue);
/// the impact case is the same times impact / stride (everything here is linear in the load).
pub struct Checks {
    pub stride: f64,
    pub impact: f64,
    pub allow_static: f64,
    pub allow_fatigue: f64,
    pub heel_ball: (f64, f64),
    /// (stress, x from centre, contact from centre)
    pub upper: (f64, f64, f64),
    pub lower: (f64, f64, f64),
    pub all_down: f64,
    /// (stress, panel length)
    pub foot: (f64, f64),
    /// (local, global, panel length)
    pub chord: (f64, f64, f64),
    pub hertz_stride: (f64, f64),
    pub hertz_impact: (f64, f64),
    /// (case, wheel load N, axle bending MPa, boss bearing MPa), each at its own load.
    pub axles: Vec<(&'static str, f64, f64, f64)>,
    /// (lateral / vertical ratio held by the vertical load, track spacing, track height)
    pub roll: (f64, f64, f64),
}

impl Checks {
    /// The worst ratio of stress to allowable over the vertical cases: fatigue at the stride,
    /// static at the impact. Below 1 passes.
    pub fn worst_ratio(&self) -> f64 {
        let k = self.impact / self.stride;
        let beams = [self.upper.0, self.lower.0, self.foot.0, self.chord.0 + self.chord.1].iter().cloned().fold(0.0, f64::max);
        let mut worst = (beams / self.allow_fatigue).max(beams * k / self.allow_static);
        for (name, _, sa, _) in &self.axles {
            let r = if *name == "stride" { sa / self.allow_fatigue } else { sa / self.allow_static };
            worst = worst.max(r);
        }
        worst
    }
}

pub fn checks(f: &Frame, length: f64) -> Result<Checks, String> {
    let (p, l) = (&f.params, &f.layout);
    let al = AL_7075_T6_STRENGTH;
    let (allow_static, allow_fatigue) = allowables(&al);
    let bw = LOADS.bw();
    let (ps, pi) = (LOADS.stride * bw, LOADS.impact * bw);
    let (xh, xb) = (0.15 * length, 0.72 * length);
    let contacts = [l.xc - p.travel, l.xc, l.xc + p.travel];
    let cell = 0.15;

    // Upper bending: heel and ball loads balanced over the contact.
    let mut upper = (0.0, 0.0, 0.0);
    let mut x = xh.max(l.xu0) + 1.0;
    while x < xb.min(l.xu1) {
        if let Some(s) = section(&f.upper, x, cell, None) {
            for &xcon in &contacts {
                let sig = strength::upper_moment(x, xcon, ps, xh, xb).abs() * s.c() / s.i;
                if sig > upper.0 {
                    upper = (sig, x - l.xc, xcon - l.xc);
                }
            }
        }
        x += 2.0;
    }

    // Lower bending: rigid frame on four wheels.
    let mut lower = (0.0, 0.0, 0.0);
    let mut x = l.axle_x[0] + 1.0;
    while x < l.axle_x[3] {
        if let Some(s) = section(&f.lower, x, cell, None) {
            for &xcon in &contacts {
                let sig = strength::lower_moment(x, &l.axle_x, xcon, ps).abs() * s.c() / s.i;
                if sig > lower.0 {
                    lower = (sig, x - l.xc, xcon - l.xc);
                }
            }
        }
        x += 3.0;
    }

    // Local bending of the members that carry the rolling contact between truss nodes, as
    // fixed-fixed beams with the load mid-panel (P L / 8), both sides together.
    let foot_top = l.z_t + 0.1 + arc_rise(p.rocker_r, 0.5 * p.upper_len) + p.wall_bot_h;
    let panel_u = p.upper_len / p.wall_panels.max(1) as f64;
    let xcon = l.xc + p.travel;
    let foot = section(&f.upper, xcon, 0.1, Some((l.z_t - 1.0, foot_top))).ok_or("wall foot section")?;
    let sig_foot = ps * panel_u / 8.0 * foot.c() / foot.i;
    let chord = section(&f.lower, xcon, 0.1, Some((l.axle_z - l.r_boss, l.z_t + 0.01))).ok_or("bottom chord section")?;
    let panel_l = p.wheel_d + p.wheel_gap;
    let sig_chord = ps * panel_l / 8.0 * chord.c() / chord.i;
    let glob = section(&f.lower, xcon, cell, None).ok_or("lower section")?;
    let sig_glob = strength::lower_moment(xcon, &l.axle_x, xcon, ps).abs() * (glob.zc - glob.z_min) / glob.i;

    // Axles and bosses.
    let support_y = 0.5 * (l.y_in + l.y_out);
    let bearing_y = 0.5 * p.wheel_w - 3.5;
    let w_max = |pp: f64| {
        contacts
            .iter()
            .map(|&c| strength::wheel_reactions(&l.axle_x, c, pp).iter().cloned().fold(0.0, f64::max))
            .fold(0.0, f64::max)
    };
    let mut axles = Vec::new();
    for (name, w) in [("stride", w_max(ps)), ("impact", w_max(pi)), ("stride on one wheel", ps)] {
        let sa = strength::axle_stress(w, support_y, bearing_y, p.axle_d);
        let sb = 0.5 * w / (p.axle_d * p.rail_t);
        axles.push((name, w, sa, sb));
    }

    let spacing = 2.0 * (l.y_wi + 0.5 * p.wall_t);
    let height = l.z_t - l.ground_z;
    Ok(Checks {
        stride: ps,
        impact: pi,
        allow_static,
        allow_fatigue,
        heel_ball: (xh, xb),
        upper,
        lower,
        all_down: strength::all_wheels_down(&l.axle_x),
        foot: (sig_foot, panel_u),
        chord: (sig_chord, sig_glob, panel_l),
        hertz_stride: strength::hertz_line(0.5 * ps, p.rocker_r, p.wall_t, &al),
        hertz_impact: strength::hertz_line(0.5 * pi, p.rocker_r, p.wall_t, &al),
        axles,
        roll: (strength::roll_ratio(spacing, height), spacing, height),
    })
}
