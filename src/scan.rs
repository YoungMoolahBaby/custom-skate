//! Foot scans: load, tidy enough to measure, and measure.
//!
//! Round 1 runs on a stand-in scan (Schuster, Cresswell & Kelly 2024, Dryad, CC0) until Jon's own
//! scans exist. It measures along the scan's principal axes (mesh-measure's oriented bounding
//! box): length (heel to toe), width, height; a girth profile (cross-section perimeters
//! perpendicular to the length axis, heel to toe); and the scan's open edges (the cut above the
//! ankle, which also tells which end is the heel).

use std::path::Path;

use cortenforge::cf_scan_prep_core::{detect_boundary_loops, fit_plane_to_points};
use cortenforge::mesh::io::load_mesh;
use cortenforge::mesh::measure::{cross_section, oriented_bounding_box};
use cortenforge::mesh::repair::weld_vertices;
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Vector3};

/// One girth station, `frac` of the length from the heel (0) to the toe (1).
#[derive(Debug, Clone)]
pub struct Station {
    pub frac: f64,
    /// mm
    pub perimeter: f64,
    /// mm^2
    pub area: f64,
    /// Extent across the foot, mm.
    pub width: f64,
    /// Extent up the foot, mm.
    pub height: f64,
    pub contours: usize,
}

/// An open edge of the scan.
#[derive(Debug, Clone)]
pub struct OpenLoop {
    pub vertices: usize,
    /// mm
    pub length: f64,
    pub centroid: Point3<f64>,
    pub points: Vec<Point3<f64>>,
}

#[derive(Debug, Clone)]
pub struct FootMeasure {
    /// Heel to toe, across, and up, in mm, along the principal axes.
    pub length: f64,
    pub width: f64,
    pub height: f64,
    /// Unit axes: heel-to-toe, across, up.
    pub along: Vector3<f64>,
    pub across: Vector3<f64>,
    pub up: Vector3<f64>,
    pub center: Point3<f64>,
    pub stations: Vec<Station>,
    pub open_loops: Vec<OpenLoop>,
    pub patches: Vec<Patch>,
    /// What set "up": the open cut, the flat sole, or the shortest principal axis.
    pub up_from: &'static str,
}

/// Units guess from the longest extent: a foot is ~200-320 mm long.
pub fn scale_to_mm(longest_extent: f64) -> f64 {
    if longest_extent < 2.0 {
        1000.0 // metres
    } else if longest_extent < 50.0 {
        10.0 // centimetres
    } else {
        1.0
    }
}

/// Load a scan (OBJ, STL, PLY), convert to mm, weld if it arrives as a triangle soup.
/// Returns the mesh, the scale applied, and how many vertices welding merged.
pub fn load(path: &Path) -> Result<(IndexedMesh, f64, usize), String> {
    let mut mesh = load_mesh(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let obb = oriented_bounding_box(&mesh);
    let longest = 2.0 * obb.half_extents.max();
    let scale = scale_to_mm(longest);
    if scale != 1.0 {
        for v in &mut mesh.vertices {
            v.coords *= scale;
        }
    }
    // Always weld: near-duplicate vertices break mesh-measure's section chaining (its endpoints
    // must match within 1e-6), and even cf-design's own meshes carry a few (docs/CORTENFORGE.md).
    let merged = weld_vertices(&mut mesh, 1e-5);
    Ok((mesh, scale, merged))
}

/// `mesh_measure::cross_section`, nudged until no vertex lies within 1 um of the plane. A plane
/// through or next to vertices makes tiny segments whose ends fall inside the chaining tolerance
/// (1e-6), so it drops or misjoins segments and returns broken loops with ~2x the perimeter
/// (docs/CORTENFORGE.md). PLY stores f32, so "on the plane" means within ~3e-5 mm, not 0.
/// Each nudge is 3.3 um: nothing at the scale of a foot.
pub fn section(
    mesh: &IndexedMesh,
    point: Point3<f64>,
    normal: Vector3<f64>,
) -> cortenforge::mesh::measure::CrossSection {
    let n = normal.normalize();
    let mut p = point;
    for _ in 0..32 {
        if !mesh.vertices.iter().any(|v| (v - p).dot(&n).abs() < 1e-3) {
            break;
        }
        p += n * 3.3e-3;
    }
    cross_section(mesh, p, n)
}

/// A flat patch of the surface: faces whose normals lie within 5 deg of a common direction.
#[derive(Debug, Clone)]
pub struct Patch {
    /// Outward normal (unit).
    pub normal: Vector3<f64>,
    /// mm^2
    pub area: f64,
    /// Longest in-plane extent of the patch, mm.
    pub extent: f64,
    /// RMS distance of its face centres from their mean plane, mm.
    pub rms: f64,
    pub centroid: Point3<f64>,
}

/// Up to two large flat patches (each over 2% of the surface, planar to 1 mm RMS, at least 60 deg
/// apart): on a capped foot scan, the sole and the cap above the ankle.
/// Every face as (unit normal, area, centre).
fn face_list(mesh: &IndexedMesh) -> Vec<(Vector3<f64>, f64, Point3<f64>)> {
    mesh.faces
        .iter()
        .filter_map(|f| {
            let [a, b, c] = f.map(|i| mesh.vertices[i as usize]);
            let n = (b - a).cross(&(c - a));
            let area = 0.5 * n.norm();
            (area > 0.0).then(|| (n / (2.0 * area), area, Point3::from((a.coords + b.coords + c.coords) / 3.0)))
        })
        .collect()
}

/// The (unsigned) direction with the most surface area facing within 10 deg of it either way:
/// for a standing foot, vertical. A Fibonacci sweep of the hemisphere (~4.5 deg apart), then a
/// local search down to 0.25 deg.
fn horizontal_axis(faces: &[(Vector3<f64>, f64, Point3<f64>)]) -> Vector3<f64> {
    let cone = 10f64.to_radians().cos();
    let score = |d: &Vector3<f64>| faces.iter().filter(|f| f.0.dot(d).abs() > cone).map(|f| f.1).sum::<f64>();
    let m = 2000;
    let golden = std::f64::consts::PI * (3.0 - 5f64.sqrt());
    let mut best = (Vector3::z(), f64::MIN);
    for i in 0..m / 2 {
        let z = 1.0 - (2 * i + 1) as f64 / m as f64;
        let r = (1.0 - z * z).sqrt();
        let phi = i as f64 * golden;
        let d = Vector3::new(r * phi.cos(), r * phi.sin(), z);
        let s = score(&d);
        if s > best.1 {
            best = (d, s);
        }
    }
    for step_deg in [2.0, 1.0, 0.5, 0.25] {
        let (e1, e2) = plane_basis(best.0);
        let t = f64::to_radians(step_deg);
        loop {
            let mut improved = false;
            for k in 0..8 {
                let a = k as f64 * std::f64::consts::FRAC_PI_4;
                let d = (best.0 + (e1 * a.cos() + e2 * a.sin()) * t).normalize();
                let s = score(&d);
                if s > best.1 {
                    best = (d, s);
                    improved = true;
                }
            }
            if !improved {
                break;
            }
        }
    }
    best.0
}

pub fn flat_patches(mesh: &IndexedMesh) -> Vec<Patch> {
    let faces = face_list(mesh);
    let total: f64 = faces.iter().map(|f| f.1).sum();
    let mut bins: std::collections::HashMap<[i32; 3], (Vector3<f64>, f64)> = Default::default();
    for (n, a, _) in &faces {
        let key = [(n.x * 12.0).round() as i32, (n.y * 12.0).round() as i32, (n.z * 12.0).round() as i32];
        let e = bins.entry(key).or_insert((Vector3::zeros(), 0.0));
        e.0 += n * *a;
        e.1 += a;
    }
    let mut seeds: Vec<(Vector3<f64>, f64)> = bins.into_values().map(|(s, a)| (s.normalize(), a)).collect();
    seeds.sort_by(|a, b| b.1.total_cmp(&a.1));

    let cos5 = 5f64.to_radians().cos();
    let mut patches: Vec<Patch> = Vec::new();
    for (seed, _) in seeds.iter().take(8) {
        if patches.iter().any(|p| p.normal.dot(seed) > 60f64.to_radians().cos()) {
            continue;
        }
        let mut dir = *seed;
        let mut members: Vec<&(Vector3<f64>, f64, Point3<f64>)> = Vec::new();
        for _ in 0..3 {
            members = faces.iter().filter(|f| f.0.dot(&dir) > cos5).collect();
            let s: Vector3<f64> = members.iter().map(|f| f.0 * f.1).sum();
            if s.norm() == 0.0 {
                break;
            }
            dir = s.normalize();
        }
        let area: f64 = members.iter().map(|f| f.1).sum();
        if area < 0.02 * total {
            continue;
        }
        let centroid = Point3::from(members.iter().map(|f| f.2.coords * f.1).sum::<Vector3<f64>>() / area);
        let rms = (members.iter().map(|f| (f.2 - centroid).dot(&dir).powi(2) * f.1).sum::<f64>() / area).sqrt();
        if rms > 1.0 {
            continue;
        }
        let (e1, e2) = plane_basis(dir);
        let pts: Vec<(f64, f64)> =
            members.iter().map(|f| ((f.2 - centroid).dot(&e1), (f.2 - centroid).dot(&e2))).collect();
        let major = major_axis_2d(&pts);
        let proj: Vec<f64> = pts.iter().map(|p| p.0 * major.0 + p.1 * major.1).collect();
        let extent = proj.iter().cloned().fold(f64::MIN, f64::max) - proj.iter().cloned().fold(f64::MAX, f64::min);
        patches.push(Patch { normal: dir, area, extent, rms, centroid });
        if patches.len() == 2 {
            break;
        }
    }
    patches
}

/// For a candidate up: the length of the lower half of the scan over the length of the upper half,
/// each the extent along its own major axis. A foot standing upright is long below (the whole
/// foot) and compact above (the leg and the top of the instep), so the true up scores over 1.
/// Halves, not thin bands: a thin band at a few degrees of tilt catches only one end of the sole
/// and flips the answer (ratio 2.7 on the dataset's own z, 0.89 at 7 deg off, scan 002_mBW).
fn band_ratio(mesh: &IndexedMesh, up: Vector3<f64>) -> f64 {
    let h = |v: &Point3<f64>| v.coords.dot(&up);
    let (lo, hi) = mesh.vertices.iter().fold((f64::MAX, f64::MIN), |a, v| (a.0.min(h(v)), a.1.max(h(v))));
    let band = 0.5 * (hi - lo);
    let (e1, e2) = plane_basis(up);
    let extent = |keep: &dyn Fn(f64) -> bool| {
        let pts: Vec<(f64, f64)> = mesh
            .vertices
            .iter()
            .filter(|v| keep(h(v)))
            .map(|v| (v.coords.dot(&e1), v.coords.dot(&e2)))
            .collect();
        if pts.len() < 3 {
            return 0.0;
        }
        let (c, s) = major_axis_2d(&pts);
        let proj = pts.iter().map(|p| p.0 * c + p.1 * s);
        let (a, b) = proj.fold((f64::MAX, f64::MIN), |a, x| (a.0.min(x), a.1.max(x)));
        b - a
    };
    let bottom = extent(&|x| x < lo + band);
    let top = extent(&|x| x > hi - band);
    bottom / top.max(1e-9)
}

/// An orthonormal pair spanning the plane normal to `n`.
fn plane_basis(n: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let helper = if n.x.abs() < 0.9 { Vector3::x() } else { Vector3::y() };
    let e1 = helper.cross(&n).normalize();
    (e1, n.cross(&e1))
}

/// Unit major axis of a 2D point cloud (principal component), as (cos, sin).
fn major_axis_2d(pts: &[(f64, f64)]) -> (f64, f64) {
    let n = pts.len().max(1) as f64;
    let (mx, my) = pts.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0 / n, a.1 + p.1 / n));
    let (mut sxx, mut syy, mut sxy) = (0.0, 0.0, 0.0);
    for p in pts {
        let (x, y) = (p.0 - mx, p.1 - my);
        sxx += x * x;
        syy += y * y;
        sxy += x * y;
    }
    let theta = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    (theta.cos(), theta.sin())
}

/// Measure a foot (or any elongated closed or open mesh) in mm.
///
/// The frame, found from the surface (principal axes don't work: a 125 mm leg stump tilts them
/// ~12 deg):
/// - **Up**: from the biggest open edge (the cut above the ankle), its plane's normal. A closed
///   scan (the cut capped): the direction with the most surface within 10 deg of level either way
///   (sole, cap, heel pad), refined by the mean of those faces' normals; signed so the lower half
///   is the long one (the whole foot) and the upper half compact (the leg); then snapped to the
///   flat sole if one lies within 15 deg.
/// - **Heel to toe**: the major axis of the lower 35% of the scan (heel to toes even when the
///   toes lift off the floor).
/// - **Heel**: the end whose half of the scan stands taller (the leg rises over the heel).
///
/// On the 200 stand-in scans (Schuster et al. 2024, which come pre-aligned): up within 2.7 deg of
/// theirs on all, heel never swapped, length -0.48% to +1.45% of theirs (`batch_scan`).
pub fn measure(mesh: &IndexedMesh, stations: usize) -> FootMeasure {
    let open_loops: Vec<OpenLoop> = detect_boundary_loops(mesh)
        .iter()
        .map(|l| {
            let pts: Vec<Point3<f64>> = l.vertices.iter().map(|&i| mesh.vertices[i as usize]).collect();
            let n = pts.len();
            let length = (0..n).map(|k| (pts[(k + 1) % n] - pts[k]).norm()).sum();
            let centroid = Point3::from(pts.iter().map(|p| p.coords).sum::<Vector3<f64>>() / n as f64);
            OpenLoop { vertices: n, length, centroid, points: pts }
        })
        .collect();

    let centroid = Point3::from(mesh.vertices.iter().map(|v| v.coords).sum::<Vector3<f64>>() / mesh.vertices.len() as f64);
    let patches = flat_patches(mesh);
    let (up, how) = if let Some(cut) = open_loops.iter().max_by(|a, b| a.length.total_cmp(&b.length)) {
        // A cut above the ankle is roughly level: its plane's normal, pointing from the bulk to
        // the cut, is up (scan-prep-core's plane fit).
        let (_, n, _) = fit_plane_to_points(&cut.points);
        (n * n.dot(&(cut.centroid - centroid)).signum(), "the open cut")
    } else {
        // Closed scan (the cut is capped). The axis with the most near-horizontal surface (sole,
        // the capped cut, the top of the forefoot), signed so the floor band is the long one.
        let faces = face_list(mesh);
        let cone = 10f64.to_radians().cos();
        let cos15 = 15f64.to_radians().cos();
        // Refine (still unsigned): the area-weighted mean of the near-level faces' normals.
        let folded = |axis: Vector3<f64>| -> Vector3<f64> {
            faces
                .iter()
                .filter(|f| f.0.dot(&axis).abs() > cone)
                .map(|f| f.0 * (f.1 * f.0.dot(&axis).signum()))
                .sum::<Vector3<f64>>()
                .normalize()
        };
        let axis = folded(horizontal_axis(&faces));
        let (rp, rm) = (band_ratio(mesh, axis), band_ratio(mesh, -axis));
        if std::env::var_os("SCAN_DEBUG").is_some() {
            eprintln!("axis ({:+.4}, {:+.4}, {:+.4}) half ratio +{rp:.3} -{rm:.3}", axis.x, axis.y, axis.z);
        }
        let axis = if rp >= rm { axis } else { -axis };
        if let Some(sole) = patches.iter().filter(|p| -p.normal.dot(&axis) > cos15).max_by(|a, b| a.area.total_cmp(&b.area)) {
            (-sole.normal, "the flat sole")
        } else {
            // No flat sole (a lightly loaded foot): the refined axis, pointing up.
            (folded(axis), "the level surfaces")
        }
    };
    let h = |v: &Point3<f64>| v.coords.dot(&up);
    let (hmin, hmax) = mesh.vertices.iter().fold((f64::MAX, f64::MIN), |a, v| (a.0.min(h(v)), a.1.max(h(v))));
    let band = 0.35 * (hmax - hmin);
    let (e1, e2) = plane_basis(up);
    let foot: Vec<(f64, f64)> = mesh
        .vertices
        .iter()
        .filter(|v| h(v) < hmin + band)
        .map(|v| (v.coords.dot(&e1), v.coords.dot(&e2)))
        .collect();
    let (c, s) = major_axis_2d(&foot);
    let mut along = e1 * c + e2 * s;
    // Heel: the half that stands taller.
    let p = |v: &Point3<f64>| v.coords.dot(&along);
    let (pmin, pmax) = mesh.vertices.iter().fold((f64::MAX, f64::MIN), |a, v| (a.0.min(p(v)), a.1.max(p(v))));
    let mid = 0.5 * (pmin + pmax);
    let (mut lo_h, mut lo_n, mut hi_h, mut hi_n) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    for v in &mesh.vertices {
        if p(v) < mid {
            lo_h += h(v) - hmin;
            lo_n += 1.0;
        } else {
            hi_h += h(v) - hmin;
            hi_n += 1.0;
        }
    }
    if hi_h / hi_n.max(1.0) > lo_h / lo_n.max(1.0) {
        along = -along; // the taller half is the heel; along points heel to toe
    }
    let across = up.cross(&along).normalize();
    let w = |v: &Point3<f64>| v.coords.dot(&across);
    let (wmin, wmax) = mesh.vertices.iter().fold((f64::MAX, f64::MIN), |a, v| (a.0.min(w(v)), a.1.max(w(v))));
    let q = |v: &Point3<f64>| v.coords.dot(&along);
    let (amin, amax) = mesh.vertices.iter().fold((f64::MAX, f64::MIN), |a, v| (a.0.min(q(v)), a.1.max(q(v))));
    let length = amax - amin;
    let width = wmax - wmin;
    let height = hmax - hmin;
    let center = Point3::from(
        along * (0.5 * (amin + amax)) + across * (0.5 * (wmin + wmax)) + up * (0.5 * (hmin + hmax)),
    );
    let heel = center - along * (length / 2.0);

    let stations = (1..stations)
        .map(|k| {
            let frac = k as f64 / stations as f64;
            let p = heel + along * (frac * length);
            let cs = section(mesh, p, along);
            let (mut wmin, mut wmax, mut hmin, mut hmax) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
            for q in &cs.points {
                let w = (q - p).dot(&across);
                let h = (q - p).dot(&up);
                wmin = wmin.min(w);
                wmax = wmax.max(w);
                hmin = hmin.min(h);
                hmax = hmax.max(h);
            }
            let (width, height) = if cs.points.is_empty() { (0.0, 0.0) } else { (wmax - wmin, hmax - hmin) };
            Station { frac, perimeter: cs.perimeter, area: cs.area, width, height, contours: cs.contour_count }
        })
        .collect();

    FootMeasure { length, width, height, along, across, up, center, stations, open_loops, patches, up_from: how }
}

/// `scan <file>`: load and measure a foot scan, print a report.
pub fn run(path: &Path) -> Result<(), String> {
    let (mesh, scale, merged) = load(path)?;
    println!("{}", path.display());
    println!(
        "  {} vertices, {} triangles; units x{scale} to mm; welding merged {merged} vertices",
        mesh.vertices.len(),
        mesh.faces.len()
    );
    let m = measure(&mesh, 20);
    println!(
        "  length {:.1} mm (heel to toe), width {:.1} mm, height {:.1} mm; up from {}",
        m.length, m.width, m.height, m.up_from
    );
    println!(
        "  axes in the file's frame: heel-to-toe ({:+.3}, {:+.3}, {:+.3}), up ({:+.3}, {:+.3}, {:+.3})",
        m.along.x, m.along.y, m.along.z, m.up.x, m.up.y, m.up.z
    );
    for p in &m.patches {
        println!(
            "  flat patch: {:.0} mm2, {:.0} mm long, {:.2} mm RMS, normal ({:+.3}, {:+.3}, {:+.3})",
            p.area, p.extent, p.rms, p.normal.x, p.normal.y, p.normal.z
        );
    }
    println!("  open edges: {}", m.open_loops.len());
    let mut loops = m.open_loops.clone();
    loops.sort_by(|a, b| b.length.total_cmp(&a.length));
    for l in loops.iter().take(5) {
        let c = l.centroid - m.center;
        println!(
            "    {:>6.1} mm around, {:>5} vertices, centre {:+.0} along / {:+.0} up (mm from the box centre)",
            l.length,
            l.vertices,
            c.dot(&m.along),
            c.dot(&m.up)
        );
    }
    println!("  girth profile, heel (0) to toe (1):");
    println!("    {:>5} {:>10} {:>10} {:>9} {:>9} {:>8}", "frac", "girth mm", "area mm2", "width", "height", "loops");
    for s in &m.stations {
        println!(
            "    {:>5.2} {:>10.1} {:>10.0} {:>9.1} {:>9.1} {:>8}",
            s.frac, s.perimeter, s.area, s.width, s.height, s.contours
        );
    }
    if let Some(ball) = m
        .stations
        .iter()
        .filter(|s| (0.55..=0.85).contains(&s.frac))
        .max_by(|a, b| a.width.total_cmp(&b.width))
    {
        println!(
            "  widest forefoot station (about the ball): {:.0}% of length, width {:.1} mm, girth {:.1} mm",
            ball.frac * 100.0,
            ball.width,
            ball.perimeter
        );
    }
    Ok(())
}
