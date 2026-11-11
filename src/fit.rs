//! Fit, round 1: how far each point of the real foot is from the last (the liner's inside).
//!
//! Positive gap = room (the liner foam fills it or it is loose); negative = the last cuts into the
//! foot (the liner is squeezed there). Round 1's last is the foot closed and toe-boxed plus an
//! allowance, so it never cuts in: this is a map of room, not pressure. Round 2 shapes the last
//! tighter where a skate should hold (heel, midfoot) and asks sim-soft for pressures.

use cortenforge::cf_design::Solid;
use cortenforge::mesh::types::IndexedMesh;

/// Per-vertex gaps (mm) of the foot mesh against `last`, for vertices below `top`; `None` above.
pub fn gaps(foot: &IndexedMesh, last: &Solid, top: f64) -> Vec<Option<f64>> {
    foot.vertices.iter().map(|v| (v.z < top).then(|| -last.evaluate(v))).collect()
}

/// Gap statistics for one region of the foot.
pub struct Region {
    pub name: &'static str,
    pub n: usize,
    pub min: f64,
    pub p10: f64,
    pub median: f64,
    pub p90: f64,
}

/// Regions by fraction of foot length (heel at 0) and height: the sole is the lowest 8 mm.
pub fn regions(foot: &IndexedMesh, gaps: &[Option<f64>], length: f64) -> Vec<Region> {
    let parts: [(&'static str, f64, f64, bool); 8] = [
        ("heel, sole", 0.0, 0.25, true),
        ("heel, upper", 0.0, 0.25, false),
        ("midfoot, sole (arch)", 0.25, 0.62, true),
        ("midfoot, upper (instep)", 0.25, 0.62, false),
        ("forefoot, sole (ball)", 0.62, 0.85, true),
        ("forefoot, upper", 0.62, 0.85, false),
        ("toes, sole", 0.85, 1.01, true),
        ("toes, upper", 0.85, 1.01, false),
    ];
    parts
        .iter()
        .map(|&(name, a, b, sole)| {
            let mut g: Vec<f64> = foot
                .vertices
                .iter()
                .zip(gaps)
                .filter_map(|(v, g)| {
                    let f = v.x / length;
                    (f >= a && f < b && (v.z < 8.0) == sole).then_some(*g).flatten()
                })
                .collect();
            g.sort_by(|x, y| x.total_cmp(y));
            let q = |f: f64| if g.is_empty() { f64::NAN } else { g[((g.len() - 1) as f64 * f).round() as usize] };
            Region { name, n: g.len(), min: q(0.0), p10: q(0.1), median: q(0.5), p90: q(0.9) }
        })
        .collect()
}
