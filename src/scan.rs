//! Foot scans: load, tidy enough to measure, and measure.
//!
//! Round 1 runs on a stand-in scan (Schuster, Cresswell & Kelly 2024, Dryad, CC0) until Jon's own
//! scans exist. It measures along the scan's principal axes (mesh-measure's oriented bounding
//! box): length (heel to toe), width, height; a girth profile (cross-section perimeters
//! perpendicular to the length axis, heel to toe); and the scan's open edges (the cut above the
//! ankle, which also tells which end is the heel).

use std::path::Path;

use cortenforge::cf_scan_prep_core::detect_boundary_loops;
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

/// Measure a foot (or any elongated closed or open mesh) in mm.
pub fn measure(mesh: &IndexedMesh, stations: usize) -> FootMeasure {
    let obb = oriented_bounding_box(mesh);
    let axes = [
        obb.rotation * Vector3::x(),
        obb.rotation * Vector3::y(),
        obb.rotation * Vector3::z(),
    ];
    let half = obb.half_extents;
    let mut order = [0usize, 1, 2];
    order.sort_by(|&a, &b| half[b].total_cmp(&half[a]));
    let mut along = axes[order[0]];
    let (mut a1, mut a2) = (order[1], order[2]);

    let open_loops: Vec<OpenLoop> = detect_boundary_loops(mesh)
        .iter()
        .map(|l| {
            let pts: Vec<Point3<f64>> = l.vertices.iter().map(|&i| mesh.vertices[i as usize]).collect();
            let n = pts.len();
            let length = (0..n).map(|k| (pts[(k + 1) % n] - pts[k]).norm()).sum();
            let centroid = Point3::from(pts.iter().map(|p| p.coords).sum::<Vector3<f64>>() / n as f64);
            OpenLoop { vertices: n, length, centroid }
        })
        .collect();

    // Up and heel from the biggest open edge (the cut above the ankle, over the heel). Without
    // one, up is the shortest axis and the heel is at -along.
    let mut up = axes[a2];
    if let Some(cut) = open_loops.iter().max_by(|a, b| a.length.total_cmp(&b.length)) {
        let off = cut.centroid - obb.center;
        if off.dot(&axes[a1]).abs() > off.dot(&axes[a2]).abs() {
            std::mem::swap(&mut a1, &mut a2);
        }
        up = axes[a2];
        if off.dot(&up) < 0.0 {
            up = -up;
        }
        if off.dot(&along) > 0.0 {
            along = -along; // heel at -along, so along points heel to toe
        }
    }
    let across = up.cross(&along).normalize();

    let length = 2.0 * half[order[0]];
    let width = 2.0 * half[a1];
    let height = 2.0 * half[a2];
    let heel = obb.center - along * (length / 2.0);

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

    FootMeasure { length, width, height, along, across, up, center: obb.center, stations, open_loops }
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
    println!("  length {:.1} mm, width {:.1} mm, height {:.1} mm (principal axes)", m.length, m.width, m.height);
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
