//! The foot in a standard frame, and its signed-distance field.
//!
//! Standard frame: x from the heel (rearmost point at x = 0) toward the toes, y across, z up, the
//! floor (lowest point) at z = 0, y centred on the footprint. Everything downstream (last, boot,
//! chassis) is built in this frame, in mm.

use std::sync::Arc;
use std::time::Instant;

use cortenforge::cf_design::{Aabb, Solid};
use cortenforge::mesh::repair::weld_vertices;
use cortenforge::mesh::sdf::{CachedGridSdf, TriMeshDistance, WALL_THRESHOLD_FACTOR_DEFAULT};
use cortenforge::mesh::types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::scan::FootMeasure;

/// A surface in the standard frame with its signed-distance field: `eval = grid + bias`.
#[derive(Clone)]
pub struct Foot {
    /// The scan, for display and for fit checks (always the real foot).
    pub mesh: IndexedMesh,
    /// Heel to toe, across, floor to the top of the scan, mm.
    pub length: f64,
    pub width: f64,
    pub height: f64,
    pub sdf: Arc<CachedGridSdf>,
    /// Added to the grid's distance: a closed foot is a grown surface shrunk back by `bias`.
    pub bias: f64,
    /// The region the field covers.
    pub bounds: Aabb,
}

/// Move a measured scan into the standard frame.
pub fn canonical(mesh: &IndexedMesh, m: &FootMeasure) -> IndexedMesh {
    let (x, y, z) = (m.along, m.up.cross(&m.along).normalize(), m.up);
    let mut out = mesh.clone();
    for v in &mut out.vertices {
        *v = Point3::new(v.coords.dot(&x), v.coords.dot(&y), v.coords.dot(&z));
    }
    let lo = out.vertices.iter().fold(Vector3::repeat(f64::MAX), |a, v| a.inf(&v.coords));
    // Centre y on the footprint (the lowest 10 mm), not on the whole scan.
    let (fy0, fy1) = out
        .vertices
        .iter()
        .filter(|v| v.z < lo.z + 10.0)
        .fold((f64::MAX, f64::MIN), |a, v| (a.0.min(v.y), a.1.max(v.y)));
    let shift = Vector3::new(lo.x, 0.5 * (fy0 + fy1), lo.z);
    for v in &mut out.vertices {
        v.coords -= shift;
    }
    out
}

fn bounds_of(mesh: &IndexedMesh, margin: f64) -> Aabb {
    let lo = mesh.vertices.iter().fold(Vector3::repeat(f64::MAX), |a, v| a.inf(&v.coords));
    let hi = mesh.vertices.iter().fold(Vector3::repeat(f64::MIN), |a, v| a.sup(&v.coords));
    Aabb::new(Point3::from(lo - Vector3::repeat(margin)), Point3::from(hi + Vector3::repeat(margin)))
}

/// mesh-sdf's cached grid over a closed mesh (flood-filled sign: robust on scans, but only trust
/// it more than a cell from the surface).
fn grid(mesh: &IndexedMesh, bounds: Aabb, cell: f64, what: &str) -> Result<Arc<CachedGridSdf>, String> {
    let t0 = Instant::now();
    let distance = TriMeshDistance::new(mesh.clone()).map_err(|e| format!("distance: {e}"))?;
    let (grid, report) = CachedGridSdf::build(&distance, bounds, cell, WALL_THRESHOLD_FACTOR_DEFAULT)
        .map_err(|e| format!("sdf grid: {e:?}"))?;
    eprintln!(
        "  {what}: distance grid {cell} mm, {:?} nodes, {} inside component(s), {:.1} s",
        report.dims,
        report.inside_components,
        t0.elapsed().as_secs_f64()
    );
    Ok(Arc::new(grid))
}

/// Build the foot: standard frame, then its distance grid at `cell` mm over the scan plus
/// `margin` mm.
pub fn build(mesh: &IndexedMesh, m: &FootMeasure, cell: f64, margin: f64) -> Result<Foot, String> {
    let mesh = canonical(mesh, m);
    let bounds = bounds_of(&mesh, margin);
    let sdf = grid(&mesh, bounds, cell, "foot")?;
    let (lo, hi) = (bounds.min.coords + Vector3::repeat(margin), bounds.max.coords - Vector3::repeat(margin));
    Ok(Foot { length: hi.x - lo.x, width: hi.y - lo.y, height: hi.z - lo.z, mesh, sdf, bias: 0.0, bounds })
}

impl Foot {
    /// Signed distance of the surface this `Foot` stands for (negative inside), mm.
    pub fn eval(&self, p: Point3<f64>) -> f64 {
        self.sdf.signed_distance(p) + self.bias
    }

    /// The surface as a cf-design solid.
    pub fn solid(&self) -> Solid {
        Solid::from_sdf(SharedSdf(self.sdf.clone()), self.bounds).offset(-self.bias)
    }

    /// Morphological closing by `r` mm: grow, re-distance the grown surface, shrink back. Fills
    /// the gaps between toes and hollows narrower than ~2r (the arch), keeps convex shape. A
    /// last's starting surface. (Growing and shrinking one field cancels out, so the grown
    /// surface is meshed and given its own grid.)
    pub fn closed(&self, r: f64, cell: f64) -> Result<Foot, String> {
        let t0 = Instant::now();
        let mut grown = self.solid().offset(r).mesh(cell).geometry;
        weld_vertices(&mut grown, 1e-5);
        eprintln!("  closing r {r} mm: grown surface {} triangles, {:.1} s", grown.faces.len(), t0.elapsed().as_secs_f64());
        let bounds = bounds_of(&grown, 3.0 * cell + 2.0);
        let sdf = grid(&grown, bounds, cell, "grown foot")?;
        Ok(Foot {
            mesh: self.mesh.clone(),
            length: self.length,
            width: self.width,
            height: self.height,
            sdf,
            bias: r,
            bounds,
        })
    }
}

/// The convex hull of the foot's points from `x_from` (mm, heel at 0) forward, as a surface with
/// its own distance grid (`margin` mm around it): a toe box. cf-geometry's Quickhull.
pub fn forefoot_hull(foot: &Foot, x_from: f64, cell: f64, margin: f64) -> Result<Foot, String> {
    let pts: Vec<Point3<f64>> = foot.mesh.vertices.iter().filter(|v| v.x >= x_from).copied().collect();
    let hull = cortenforge::cf_geometry::convex_hull(&pts, None).ok_or("forefoot hull: degenerate")?;
    let mut mesh = IndexedMesh::default();
    mesh.vertices = hull.vertices.clone();
    mesh.faces = hull.faces.clone();
    eprintln!("  forefoot hull from x {x_from:.0} mm: {} points -> {} vertices, {} faces", pts.len(), mesh.vertices.len(), mesh.faces.len());
    let bounds = bounds_of(&mesh, margin);
    let sdf = grid(&mesh, bounds, cell, "forefoot hull")?;
    Ok(Foot { mesh, length: foot.length, width: foot.width, height: foot.height, sdf, bias: 0.0, bounds })
}

/// `Arc<CachedGridSdf>` as an `Sdf`, so several solids share one grid.
#[derive(Clone)]
pub struct SharedSdf(pub Arc<CachedGridSdf>);

impl cortenforge::cf_design::Sdf for SharedSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.0.signed_distance(p)
    }
    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        self.0.gradient(p)
    }
}
