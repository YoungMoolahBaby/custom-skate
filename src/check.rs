//! `check`: every tool this project builds on, tested against known results before use.
//!
//! - The pitch model (`pitch.rs`) against the numbers Daggfeldt & Petré (2025) state in
//!   their own text, and against itself (two routes, limits, slope).
//! - cf-design's `mass_properties` against exact volumes, because this project is a contest
//!   in grams: it has to resolve ~1 g on a ~180 g chassis with 2-3 mm walls.

use std::f64::consts::{FRAC_PI_2, PI};
use std::time::Instant;

use cortenforge::cf_design::Solid;
use cortenforge::cf_design::mechanism::mass::mass_properties;
use nalgebra::{UnitQuaternion, Vector3};

use crate::materials::AL_7075_T6;
use crate::pitch::{Rocker, fixed_alpha, roller_alpha};

/// Pass/fail tally.
#[derive(Default)]
pub struct Tally {
    pub pass: usize,
    pub fail: usize,
}

impl Tally {
    pub fn item(&mut self, name: &str, ok: bool, detail: impl AsRef<str>) {
        if ok {
            self.pass += 1;
        } else {
            self.fail += 1;
        }
        println!("  [{}] {name}: {}", if ok { " ok " } else { "FAIL" }, detail.as_ref());
    }
}

pub fn run() -> Tally {
    let mut t = Tally::default();
    println!("pitch model (Daggfeldt & Petre 2025)");
    pitch_checks(&mut t);
    println!("cf-design mass_properties vs exact volumes ({})", AL_7075_T6.name);
    mass_checks(&mut t);
    println!("scan measuring (mesh-io, mesh-measure, scan-prep-core) on a tilted ellipsoid");
    scan_checks(&mut t);
    t
}

/// Ramanujan's second approximation to an ellipse's perimeter (error ~1e-10 relative here).
fn ellipse_perimeter(a: f64, b: f64) -> f64 {
    let h = ((a - b) / (a + b)).powi(2);
    PI * (a + b) * (1.0 + 3.0 * h / (10.0 + (4.0 - 3.0 * h).sqrt()))
}

/// A foot-sized ellipsoid (260 x 100 x 80 mm), meshed by cf-design, tilted and moved, written to
/// PLY and read back the way a scan is, then measured: extents and girths have exact answers.
fn scan_checks(t: &mut Tally) {
    let (a, b, c) = (130.0, 50.0, 40.0);
    let t0 = Instant::now();
    let mut mesh = Solid::ellipsoid(Vector3::new(a, b, c)).mesh(1.0).geometry;
    let tilt = UnitQuaternion::from_euler_angles(0.3, -0.5, 1.1);
    let shift = Vector3::new(40.0, -25.0, 310.0);
    for v in &mut mesh.vertices {
        *v = tilt * *v + shift;
    }
    let path = std::env::temp_dir().join("custom-skate-check-ellipsoid.ply");
    cortenforge::mesh::io::save_mesh(&mesh, &path).expect("save ply");
    let raw = cortenforge::mesh::io::load_mesh(&path).expect("load ply");
    let (back, scale, merged) = crate::scan::load(&path).expect("load ply");
    let _ = std::fs::remove_file(&path);
    let worst_move = raw
        .vertices
        .iter()
        .zip(&mesh.vertices)
        .map(|(a, b)| (a - b).norm())
        .fold(0.0_f64, f64::max);
    t.item(
        "PLY round trip: same triangles, vertices within 1e-4 mm",
        raw.faces.len() == mesh.faces.len() && raw.vertices.len() == mesh.vertices.len() && worst_move < 1e-4,
        format!(
            "{} triangles, worst vertex move {worst_move:.1e} mm (PLY stores f32); then units x{scale}, \
             welding merged {merged} vertices, {} triangles left",
            raw.faces.len(),
            back.faces.len()
        ),
    );

    let m = crate::scan::measure(&back, 20);
    let err = |x: f64, exact: f64| (x - exact) / exact;
    let (el, ew, eh) = (err(m.length, 2.0 * a), err(m.width, 2.0 * b), err(m.height, 2.0 * c));
    t.item(
        "principal-axis extents within 0.5%",
        el.abs() < 0.005 && ew.abs() < 0.005 && eh.abs() < 0.005,
        format!(
            "{:.2} / {:.2} / {:.2} mm vs 260 / 100 / 80 ({:+.3}% / {:+.3}% / {:+.3}%)",
            m.length,
            m.width,
            m.height,
            el * 100.0,
            ew * 100.0,
            eh * 100.0
        ),
    );
    // Stations at 1/4 and 1/2 of the length: ellipses with semi-axes scaled by sqrt(1 - (x/a)^2).
    let mut worst: f64 = 0.0;
    let mut line = String::new();
    for frac in [0.25, 0.5] {
        let s = m.stations.iter().find(|s| (s.frac - frac).abs() < 1e-9).expect("station");
        let x = (frac - 0.5) * 2.0 * a;
        let k = (1.0 - (x / a).powi(2)).sqrt();
        let exact = ellipse_perimeter(b * k, c * k);
        let e = err(s.perimeter, exact);
        worst = worst.max(if s.contours == 1 { e.abs() } else { f64::INFINITY });
        line += &format!(
            "{frac}: {:.2} vs {:.2} mm ({:+.3}%), {} loop(s), area {:.0} vs {:.0} mm2; ",
            s.perimeter,
            exact,
            e * 100.0,
            s.contours,
            s.area,
            PI * b * c * k * k
        );
    }
    t.item("girths: one loop each, perimeter within 0.5%", worst < 0.005, line);
    t.item(
        "closed mesh has no open edges",
        m.open_loops.is_empty(),
        format!("{} open loops, {:.2} s", m.open_loops.len(), t0.elapsed().as_secs_f64()),
    );

    // A foot with a leg stump: the ellipsoid plus a vertical cylinder over its -x end, cut flat
    // at z = 100 (scan-prep-core's plane clip), then tilted. The cut is the only open edge; it
    // must be found, and must point the heel to -x and up to +z.
    let stump = Solid::cylinder(35.0, 70.0).translate(Vector3::new(-80.0, 0.0, 60.0));
    let solid = Solid::ellipsoid(Vector3::new(a, b, c)).union(stump);
    let mut leg = solid.mesh(1.0).geometry;
    cortenforge::mesh::repair::weld_vertices(&mut leg, 1e-5);
    let mut cut = cortenforge::cf_scan_prep_core::clip_mesh_against_plane(
        &leg,
        nalgebra::Point3::new(0.0, 0.0, 100.0),
        -Vector3::z(),
    );
    let before = crate::scan::measure(&cut, 4).open_loops.len();
    let merged = cortenforge::mesh::repair::weld_vertices(&mut cut, 1e-5);
    for v in &mut cut.vertices {
        *v = tilt * *v + shift;
    }
    let m = crate::scan::measure(&cut, 20);
    let heel_to_toe = tilt * Vector3::x();
    let up = tilt * Vector3::z();
    let (d_along, d_up) = (m.along.dot(&heel_to_toe), m.up.dot(&up));
    let ring = m.open_loops.iter().map(|l| l.length).fold(0.0_f64, f64::max);
    t.item(
        "leg stump: one open edge, heel and up found",
        m.open_loops.len() == 1 && d_along > 0.95 && d_up > 0.95 && (ring - 2.0 * PI * 35.0).abs() < 1.0,
        format!(
            "{} open edge(s) after welding the clip ({before} before, {merged} vertices merged), the cut \
             {ring:.2} mm round (circle 219.91); heel-to-toe . true {d_along:.3}, up . true {d_up:.3}",
            m.open_loops.len()
        ),
    );
}

fn pitch_checks(t: &mut Tally) {
    // The paper: "moving the contact point from the back to the front of the blade, a
    // distance of not more than 30 cm, on a blade with a radius of curvature of 2.7 m
    // results in less than 0.12 rad (~7 deg) of skate rotation".
    let a = fixed_alpha(0.30, 2.7);
    t.item(
        "eq.1: 30 cm on a 2.7 m blade < 0.12 rad",
        a < 0.12,
        format!("{a:.4} rad = {:.2} deg", a.to_degrees()),
    );

    // The paper: "For a beta of 0.12 rad, the linear approximation in equation (9) would
    // result in an error in phi of less than one tenth of a degree (s = 2.7 m, h = 0.03 m,
    // r = 1 m)". beta = y / s (eq. 5).
    let p = Rocker { r: 1.0, s: 2.7, h: 0.03, theta: 0.0 };
    let y = 0.12 * p.s;
    let err = (p.phi_of_y(y) - p.phi_of_y_linear(y)).abs().to_degrees();
    t.item("eq.9 vs eq.7 at beta 0.12 rad: error < 0.1 deg", err < 0.1, format!("{err:.4} deg"));

    // Eq. 4 + eq. 5 (in alpha) and eq. 7 (in y) are the same curve.
    let mut worst: f64 = 0.0;
    for i in -300..=300 {
        let alpha = f64::from(i) * 1e-3;
        let y = p.y_of_alpha(alpha);
        worst = worst.max((p.phi_of_alpha(alpha) - p.phi_of_y(y)).abs());
    }
    t.item("eq.4+5 and eq.7 agree (alpha +-0.3 rad)", worst < 1e-12, format!("max diff {worst:.1e} rad"));

    // Eq. 11 is the slope of eq. 7 at y = 0 over the slope 1/s of a flat rocker.
    let dy = 1e-6;
    let slope = (p.phi_of_y(dy) - p.phi_of_y(-dy)) / (2.0 * dy);
    let a_num = slope * p.s;
    let a_eq = p.amplification();
    t.item(
        "eq.11 amplification = slope of eq.7 at y = 0",
        (a_num - a_eq).abs() < 1e-6,
        format!("{a_num:.6} vs {a_eq:.6} (r 1 m, s 2.7 m, h 3 cm)"),
    );

    // Limits: a flat rocker is a fixed holder (r_eff -> s); a flat blade is a roller chassis
    // (r_eff -> r, the foot pitches x / r).
    let fixed = Rocker { r: 1e9, ..p }.effective_radius();
    let roller = Rocker { s: 1e9, ..p }.effective_radius();
    t.item(
        "eq.10 limits: r -> inf gives s, s -> inf gives r",
        (fixed - p.s).abs() < 1e-6 && (roller - p.r).abs() < 1e-6,
        format!("{fixed:.6} m, {roller:.6} m"),
    );
    let x = 0.05;
    let via_limit = Rocker { s: 1e9, ..p }.phi_of_y_linear(x);
    t.item(
        "roller: pitch x / r is eq.9 with s -> inf",
        (via_limit - roller_alpha(x, p.r)).abs() < 1e-9,
        format!("{:.6} vs {:.6} rad at x 5 cm", via_limit, roller_alpha(x, p.r)),
    );
}

/// Relative error of cf-design's grid mass against an exact volume, at three grid sizes.
/// Returns the error at the finest grid.
fn mass_case(t: &mut Tally, name: &str, solid: &Solid, exact_mm3: f64) -> f64 {
    let rho = AL_7075_T6.density;
    let exact_g = exact_mm3 * 1e-9 * rho * 1000.0;
    let mut line = format!("exact {exact_g:.3} g;");
    let mut last = f64::NAN;
    for cell in [1.0, 0.5, 0.25] {
        let t0 = Instant::now();
        let m = mass_properties(solid, rho, cell).expect("solid has mass");
        let g = m.mass * 1000.0;
        last = (g - exact_g) / exact_g;
        line += &format!(
            " cell {cell} mm: {g:.3} g ({:+.3}%, {:.2} s);",
            last * 100.0,
            t0.elapsed().as_secs_f64()
        );
    }
    t.item(&format!("{name}: within 0.5% at 0.25 mm"), last.abs() < 0.005, line);
    last
}

fn mass_checks(t: &mut Tally) {
    // A rail-like plate: 100 x 3 x 30 mm.
    let plate = Solid::cuboid(Vector3::new(50.0, 1.5, 15.0));
    mass_case(t, "plate 100x3x30", &plate, 100.0 * 3.0 * 30.0);

    // A 2 mm wall with a 20 mm lightening hole through it.
    let wall = Solid::cuboid(Vector3::new(50.0, 1.0, 15.0));
    let x_to_y = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), FRAC_PI_2);
    let hole = Solid::cylinder(10.0, 5.0).rotate(x_to_y);
    let holed = wall.subtract(hole);
    mass_case(t, "2 mm wall, 20 mm hole", &holed, 100.0 * 2.0 * 30.0 - PI * 100.0 * 2.0);

    // An axle-like rod: 8 mm diameter, 60 mm long, off-axis to test the centre of mass too.
    let offset = Vector3::new(12.0, -7.0, 30.0);
    let rod = Solid::cylinder(4.0, 30.0).translate(offset);
    mass_case(t, "rod 8 x 60 mm", &rod, PI * 16.0 * 60.0);
    let m = mass_properties(&rod, AL_7075_T6.density, 0.25).expect("rod has mass");
    let com_err = (m.center_of_mass.coords - offset).norm();
    t.item("rod centre of mass at its offset", com_err < 0.01, format!("off by {com_err:.4} mm"));
}
