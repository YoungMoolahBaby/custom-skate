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
        self.closed_with_margin(r, cell, 3.0 * cell + 2.0)
    }

    /// `closed`, with the distance grid reaching `margin` mm past the grown surface. mesh-sdf
    /// clamps queries outside its grid to the edge value, so anything offset far from the
    /// surface (a boot's outer layers) needs the wider grid.
    pub fn closed_with_margin(&self, r: f64, cell: f64, margin: f64) -> Result<Foot, String> {
        let t0 = Instant::now();
        let mut grown = self.solid().offset(r).mesh(cell).geometry;
        weld_vertices(&mut grown, 1e-5);
        eprintln!("  closing r {r} mm: grown surface {} triangles, {:.1} s", grown.faces.len(), t0.elapsed().as_secs_f64());
        let bounds = bounds_of(&grown, margin);
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

/// The base of a last (round 1's recipe): the foot closed by `CLOSING` mm (the arch and small
/// hollows filled; a footbed fills the arch in a real skate), blended with the convex hull of the
/// forefoot (a toe box). Closing alone can't make a toe box: toes are ~20 mm cylinders side by
/// side, and a closing of radius r leaves a groove ~a^2/2r deep between them (7 and 18 mm both
/// left visible grooves).
pub struct LastBase {
    /// Negative inside; the last is this offset by the fit allowance.
    pub solid: Solid,
    /// Where the closed foot's distance grid is valid.
    pub bounds: Aabb,
    /// The toe box starts here, mm from the heel.
    pub toe_from: f64,
}

pub const CLOSING: f64 = 12.0;
/// The toe box from this fraction of the foot's length.
pub const TOE_FROM: f64 = 0.62;
/// The toe box fades in: shrunk by up to `FADE` mm over its first `FADE_LEN` mm, so it rises out
/// of the foot instead of starting with a step. (A 4 mm smooth union left a visible step; a 10 mm
/// one hid it but swelled the whole forefoot ~2.5 mm, since the hull hugs the foot there.)
pub const FADE: f64 = 8.0;
pub const FADE_LEN: f64 = 60.0;
pub const BLEND: f64 = 2.0;

/// Build the last's base; `margin`: how far past the grown foot its distance grid reaches.
pub fn last_base(foot: &Foot, margin: f64) -> Result<LastBase, String> {
    let closed = foot.closed_with_margin(CLOSING, 1.5, margin)?;
    let toe_from = TOE_FROM * foot.length;
    let toe = forefoot_hull(foot, toe_from, 1.5, 14.0)?;
    let hull = toe.clone();
    let (lo, hi) = (toe.bounds.min, toe.bounds.max);
    let faded_toe = Solid::user_fn(
        move |q: Point3<f64>| {
            let s = ((toe_from + FADE_LEN - q.x) / FADE_LEN).clamp(0.0, 1.0);
            // Outside its grid the hull's field is the clamped edge value, flat in space; adding
            // the distance to the grid keeps it growing, so it can't undercut the foot's own field
            // far from the toes (it made a flat tab on the tendon guard, ~30 mm out). Inside the
            // grid, and everywhere round 1 meshed, nothing changes.
            let out = Vector3::new(
                (lo.x - q.x).max(q.x - hi.x).max(0.0),
                (lo.y - q.y).max(q.y - hi.y).max(0.0),
                (lo.z - q.z).max(q.z - hi.z).max(0.0),
            )
            .norm();
            hull.eval(q) + FADE * s + out
        },
        toe.bounds,
    );
    let solid = closed.solid().smooth_union(faded_toe, BLEND);
    Ok(LastBase { solid, bounds: closed.bounds, toe_from })
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
