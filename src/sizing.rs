//! Round 2, step 2 continued: size the rocker like for like (docs/PLAN.md).
//!
//! A one-piece frame is thickened until it weighs what real ones do; its sideways stress and
//! stiffness under the stride with the sideways push are the target; the rocker's lower rails
//! and upper walls are sized to match. Every model here gets the same loads (`wheel_loads`), the
//! same boot mounts (`MOUNT_*`, clamped) and the same mesh size, so the numbers compare.
//!
//! The rocker is two bodies touching at the rocker contact (the wall feet on the ledges) and at
//! the side bearing (the walls on the rails' outer faces, through a preloaded liner, so both
//! sides bear). It is solved as two substructures:
//! 1. `lower`: the lower with its axles, held rigidly at those contacts. Gives its stresses, the
//!    axles' sideways give relative to the contacts, and the contact forces (sim-soft's
//!    reaction readout).
//! 2. `upper`: the upper with its ties, clamped at the boot mounts like the one-piece and loaded
//!    by those contact forces. Gives its stresses and how far its contact faces move; the lower
//!    rides on them as a rigid body (sideways, yaw and roll), and that adds to the axles' give.
//!
//! One-way coupling: the contact forces don't redistribute as the upper gives. The contacts are
//! two-sided, so a ledge could pull (which would need the coupling bolt, not modelled); at the
//! 0.45 push neither does: 1,438 and 34 N in the runs, the side bearing's spread in height
//! helping the ledges hold the roll. `upper(.., sole)` clamps the whole web top instead of the
//! mounts: the upper bonded to a custom boot's sole.
//!
//! V section (`FrameParams::vee`): no side bearing; the lower is held at the two tilted contact
//! patches alone, in x, y and z (sim-soft can't hold a node along a tilted normal), and each
//! side's force is split into pressing and sliding to check it could be carried. A side that
//! pulls needs the coupling bolt's preload. The upper's leaning walls cross the mesh lattice, so
//! they need a finer mesh than upright ones (`cell_up`; 1.5 mm keeps all but ~5% of the volume).

use std::collections::{BTreeMap, HashMap};
use std::time::Instant;

use cortenforge::cf_design::Solid;
use cortenforge::cf_design::mechanism::mass::mass_properties;
use nalgebra::Vector3;

use crate::fem::{self, Model, nodal_stresses, von_mises};
use crate::frame::{self, FrameParams, Layout};
use crate::materials::{AL_6061_T6, AL_7075_T6, AL_7075_T6_STRENGTH as AL, Material, PU_BUSHING, TI_6AL_4V};
use crate::strength::LOADS;

/// The boot mounts both frames bolt to: centres 165 mm apart, each 36 mm long (the one-piece's
/// mount platforms, 4 mm thick). An assumption until Jon's TF9 rivet pattern is measured.
pub const MOUNT_SPACING: f64 = 165.0;
pub const MOUNT_LEN: f64 = 36.0;
pub const MOUNT_T: f64 = 4.0;

/// Half-length of the rocker contact patch along x, mm: Hertz line contact of the 7.5 ft arc on
/// the flat ledge, both 7075, ~1400 N on a 2 mm wall (the loaded side at the sideways push):
/// b = sqrt(4 q R / (pi E*)) ~ 7 mm.
pub const PATCH_HALF: f64 = 7.0;

/// What one FE model gives.
#[derive(Debug, Clone, Default)]
pub struct Fe {
    /// Highest nodal von Mises stresses (MPa), away from the clamps and the point-like contact
    /// patches, at least 4 mm apart, with where (x from the frame centre, y, z from the axle line).
    pub top: Vec<(f64, Vector3<f64>)>,
    /// Sideways (y) displacement of each axle between the rails, mm.
    pub axle_dy: [f64; 4],
    pub dof: usize,
    /// Meshed volume / the solid's volume - 1.
    pub vol_err: f64,
    pub seconds: f64,
}

impl Fe {
    pub fn peak(&self) -> f64 {
        self.top.first().map_or(0.0, |t| t.0)
    }
}

fn grams(solid: &Solid, mat: &Material) -> f64 {
    mass_properties(solid, mat.density, 0.25).map_or(0.0, |m| m.mass * 1000.0)
}

/// The one-piece from `p`'s members, with axles and spacers (as the shops' weights are).
pub fn onepiece_grams(p: &FrameParams, length: f64) -> f64 {
    let f = frame::build(0.0, length, *p);
    let op = frame::one_piece(0.0, length, p, MOUNT_SPACING, MOUNT_LEN, MOUNT_T);
    grams(&op, &AL_7075_T6) + grams(&f.axles, &AL_7075_T6) + grams(&f.spacers, &AL_6061_T6)
}

/// The rocker with axles, spacers and the coupling (as `chassis` weighs it).
pub fn rocker_grams(p: &FrameParams, length: f64) -> f64 {
    rocker_parts(p, length).iter().map(|(_, g)| g).sum()
}

/// The rocker's grams by part: upper, lower, axles, spacers, coupling (bolt, nut, bushing), ties.
pub fn rocker_parts(p: &FrameParams, length: f64) -> Vec<(&'static str, f64)> {
    let f = frame::build(0.0, length, *p);
    let coupling = grams(&f.bolt, &TI_6AL_4V) + grams(&f.nut, &TI_6AL_4V) + grams(&f.bushing, &PU_BUSHING);
    vec![
        ("upper", grams(&f.upper, &AL_7075_T6)),
        ("lower", grams(&f.lower, &AL_7075_T6)),
        ("axles", grams(&f.axles, &AL_7075_T6)),
        ("spacers", grams(&f.spacers, &AL_6061_T6)),
        ("coupling", coupling),
        ("ties", f.ties.as_ref().map_or(0.0, |t| grams(t, &AL_7075_T6))),
    ]
}

fn mesh(body: &Solid, cell: f64) -> Result<(Model, f64), String> {
    let m = fem::mesh_solid(body, cell, AL.e, AL.nu)?;
    let exact = mass_properties(body, 1000.0, 0.25).map_or(0.0, |q| q.mass * 1e6);
    let err = fem::volume(&m) / exact - 1.0;
    Ok((m, err))
}

/// Mesh the upper (with its ties) alone: (Tet10 elements, nodes in use, volume error), to pick a
/// mesh size that keeps every part of it.
pub fn mesh_upper(p: &FrameParams, length: f64, cell: f64) -> Result<(usize, usize, f64), String> {
    let f = frame::build(0.0, length, *p);
    let body = match &f.ties {
        Some(t) => f.upper.clone().union(t.clone()),
        None => f.upper.clone(),
    };
    let (m, err) = mesh(&body, cell)?;
    Ok((m.elems.len(), m.used.iter().filter(|&&u| u).count(), err))
}

/// The four wheels' ground forces on the axles: the stride load `n` shared equally (contact at
/// the centre), each wheel's sideways force `lam` x its share acting at the ground, 36 mm below
/// the axle, which the axle sees as the same forces plus a roll couple: unequal vertical loads
/// on the wheel's two bearings (608s, 7 mm wide, centred +-(wheel width / 2 - 3.5) mm).
fn wheel_loads(m: &Model, l: &Layout, p: &FrameParams, lam: f64, held: &dyn Fn(u32) -> bool) -> Vec<(u32, Vector3<f64>)> {
    let n = LOADS.stride * LOADS.bw();
    let r_axle = 0.5 * p.axle_d;
    let y_b = 0.5 * p.wheel_w - 3.5;
    let mut loads = Vec::new();
    for &ax in &l.axle_x {
        let w = n / 4.0;
        let side = lam * w;
        let dv = side * 0.5 * p.wheel_d / (2.0 * y_b);
        for (sign, fz) in [(1.0, 0.5 * w + dv), (-1.0, 0.5 * w - dv)] {
            let seat: Vec<u32> = fem::pick(m, |q| {
                ((q.x - ax).hypot(q.z - l.axle_z) - r_axle).abs() < 0.05 && (q.y - sign * y_b).abs() < 3.5
            })
            .into_iter()
            .filter(|&v| !held(v))
            .collect();
            loads.extend(fem::spread(&seat, Vector3::new(0.0, 0.5 * side, fz)));
        }
    }
    loads
}

/// Mean sideways displacement of each axle's nodes between the rails.
fn axle_dy(m: &Model, x: &[f64], l: &Layout, p: &FrameParams) -> [f64; 4] {
    let r = 0.5 * p.axle_d + 0.05;
    std::array::from_fn(|k| {
        let ax = l.axle_x[k];
        let vs = fem::pick(m, |q| (q.x - ax).hypot(q.z - l.axle_z) < r && q.y.abs() < l.y_in);
        vs.iter().map(|&v| fem::disp(m, x, v).y).sum::<f64>() / vs.len().max(1) as f64
    })
}

/// Points within `r` of any of `pts` (a hash grid of cell `r`).
struct Near {
    r: f64,
    grid: HashMap<(i64, i64, i64), Vec<Vector3<f64>>>,
}

impl Near {
    fn new(pts: impl IntoIterator<Item = Vector3<f64>>, r: f64) -> Self {
        let mut grid: HashMap<(i64, i64, i64), Vec<Vector3<f64>>> = HashMap::new();
        for q in pts {
            grid.entry(Self::key(&q, r)).or_default().push(q);
        }
        Self { r, grid }
    }
    fn key(q: &Vector3<f64>, r: f64) -> (i64, i64, i64) {
        ((q.x / r).floor() as i64, (q.y / r).floor() as i64, (q.z / r).floor() as i64)
    }
    fn hit(&self, q: &Vector3<f64>) -> bool {
        let (i, j, k) = Self::key(q, self.r);
        for di in -1..=1 {
            for dj in -1..=1 {
                for dk in -1..=1 {
                    if let Some(v) = self.grid.get(&(i + di, j + dj, k + dk))
                        && v.iter().any(|p| (p - q).norm() < self.r)
                    {
                        return true;
                    }
                }
            }
        }
        false
    }
}

/// The highest nodal von Mises stresses, skipping `skip`, at least 4 mm apart.
fn top_stresses(m: &Model, x: &[f64], l: &Layout, skip: &dyn Fn(&Vector3<f64>) -> bool) -> Vec<(f64, Vector3<f64>)> {
    let ns = nodal_stresses(m, x);
    let mut vm: Vec<(f64, Vector3<f64>)> =
        m.rest.iter().zip(&ns).filter_map(|(q, s)| s.map(|s| (von_mises(&s), *q))).filter(|(_, q)| !skip(q)).collect();
    vm.sort_by(|a, b| b.0.total_cmp(&a.0));
    let mut top: Vec<(f64, Vector3<f64>)> = Vec::new();
    for (s, q) in vm {
        if top.iter().all(|(_, t)| (t - q).norm() > 4.0) {
            top.push((s, q));
            if top.len() == 6 {
                break;
            }
        }
    }
    top.into_iter().map(|(s, q)| (s, Vector3::new(q.x - l.xc, q.y, q.z - l.axle_z))).collect()
}

/// The axle shafts between the rails (loaded at the bearing seats; their stress is the axle's).
fn in_axle(l: &Layout, p: &FrameParams, q: &Vector3<f64>) -> bool {
    l.axle_x.iter().any(|&ax| (q.x - ax).hypot(q.z - l.axle_z) < 0.5 * p.axle_d + 1.5 && q.y.abs() < l.y_in)
}

/// Bottom of the side-bearing band at x: the liner strip runs along the wall foot's inner face,
/// `wall_bot_h` tall, from the foot's bottom edge, which is the rocker arc (rising toward the
/// wall's ends).
fn band_bottom(p: &FrameParams, l: &Layout, x: f64) -> f64 {
    l.z_t + frame::arc_rise(p.rocker_r, x - l.xc)
}

fn in_mount(l: &Layout, x: f64) -> bool {
    ((x - l.xc).abs() - 0.5 * MOUNT_SPACING).abs() < 0.5 * MOUNT_LEN
}

/// The one-piece from `p`'s members (with its axles, bonded), clamped on its two mount tops.
pub fn onepiece(p: &FrameParams, length: f64, cell: f64, lam: f64) -> Result<Fe, String> {
    let t = Instant::now();
    let f = frame::build(0.0, length, *p);
    let l = f.layout;
    let body = frame::one_piece(0.0, length, p, MOUNT_SPACING, MOUNT_LEN, MOUNT_T).union(f.axles.clone());
    let (m, vol_err) = mesh(&body, cell)?;
    let clamp = fem::pick(&m, |q| q.z > l.z_s - 1e-3 && in_mount(&l, q.x));
    let held: std::collections::BTreeSet<u32> = clamp.iter().copied().collect();
    let loads = wheel_loads(&m, &l, p, lam, &|v| held.contains(&v));
    println!("  one-piece: {} clamp nodes, {} load nodes", clamp.len(), loads.len());
    let near_clamp = Near::new(clamp.iter().map(|&v| m.rest[v as usize]), 2.0);
    let x = fem::solve(&m, clamp, Vec::new(), &loads)?;
    let top = top_stresses(&m, &x, &l, &|q| in_axle(&l, p, q) || near_clamp.hit(q));
    Ok(Fe { top, axle_dy: axle_dy(&m, &x, &l, p), dof: 3 * m.rest.len(), vol_err, seconds: t.elapsed().as_secs_f64() })
}

/// Forces the lower puts on the upper at their contacts: (rest position, force), per lower node.
#[derive(Debug, Clone, Default)]
pub struct Contacts {
    pub bearing: Vec<(Vector3<f64>, Vector3<f64>)>,
    pub patch: Vec<(Vector3<f64>, Vector3<f64>)>,
}

impl Contacts {
    pub fn total(&self) -> Vector3<f64> {
        self.bearing.iter().chain(&self.patch).map(|(_, f)| f).sum()
    }
}

/// The rocker's lower with its axles, held at the contact patches (vertical and along x) and
/// on both rails' outer faces along the upper (sideways).
pub fn lower(p: &FrameParams, length: f64, cell: f64, lam: f64) -> Result<(Fe, Contacts), String> {
    if p.vee {
        return lower_vee(p, length, cell, lam);
    }
    let t = Instant::now();
    let f = frame::build(0.0, length, *p);
    let l = f.layout;
    let body = f.lower.clone().union(f.axles.clone());
    let (m, vol_err) = mesh(&body, cell)?;
    let tol = 1e-3;
    let patch = fem::pick(&m, |q| q.z > l.z_t - tol && q.y.abs() > l.y_wi && q.y.abs() < l.y_wo && (q.x - l.xc).abs() < PATCH_HALF);
    let bearing = fem::pick(&m, |q| {
        let dz = q.z - band_bottom(p, &l, q.x);
        (q.y.abs() - l.y_out).abs() < tol && dz > -tol && dz < p.wall_bot_h && q.x > l.xu0 && q.x < l.xu1
    });
    let patch_axes = [true, false, true];
    let mut mask: BTreeMap<u32, [bool; 3]> = BTreeMap::new();
    for &v in &patch {
        *mask.entry(v).or_insert([false; 3]) = patch_axes;
    }
    for &v in &bearing {
        mask.entry(v).or_insert([false; 3])[1] = true;
    }
    let loads = wheel_loads(&m, &l, p, lam, &|v| mask.contains_key(&v));
    println!("  lower: {} contact-patch nodes, {} side-bearing nodes, {} load nodes", patch.len(), bearing.len(), loads.len());
    let near_patch = Near::new(patch.iter().map(|&v| m.rest[v as usize]), 2.0);
    let rollers: Vec<(u32, [bool; 3])> = mask.into_iter().collect();
    let (x, r) = fem::solve_with_reactions(&m, Vec::new(), rollers, &loads)?;
    let force = |v: u32, axes: [bool; 3]| {
        let i = 3 * v as usize;
        Vector3::new(
            if axes[0] { r[i] } else { 0.0 },
            if axes[1] { r[i + 1] } else { 0.0 },
            if axes[2] { r[i + 2] } else { 0.0 },
        )
    };
    let contacts = Contacts {
        bearing: bearing.iter().map(|&v| (m.rest[v as usize], force(v, [false, true, false]))).collect(),
        patch: patch.iter().map(|&v| (m.rest[v as usize], force(v, patch_axes))).collect(),
    };
    let applied: Vector3<f64> = loads.iter().map(|(_, f)| f).sum();
    let got = contacts.total();
    println!(
        "  lower: loads ({:.1}, {:.1}, {:.1}) N; contact forces ({:.1}, {:.1}, {:.1}) N",
        applied.x, applied.y, applied.z, got.x, got.y, got.z
    );
    for side in [1.0, -1.0] {
        let fs: Vector3<f64> = contacts.patch.iter().filter(|(q, _)| q.y * side > 0.0).map(|(_, f)| f).sum();
        // The ledge top's normal (up and out), and the force's part along it (+ = pressing).
        let n = Vector3::new(0.0, side * l.v_sin, l.v_cos);
        let normal = fs.dot(&n);
        println!(
            "  lower: {} contact ({:.0}, {:.0}, {:.0}) N = {normal:.0} N pressing, {:.0} N sliding",
            if side > 0.0 { "+y" } else { "-y" },
            fs.x,
            fs.y,
            fs.z,
            (fs - normal * n).norm()
        );
    }
    let top = top_stresses(&m, &x, &l, &|q| in_axle(&l, p, q) || near_patch.hit(q));
    let fe = Fe { top, axle_dy: axle_dy(&m, &x, &l, p), dof: 3 * m.rest.len(), vol_err, seconds: t.elapsed().as_secs_f64() };
    Ok((fe, contacts))
}

/// Hertz's elliptical weight along a contact patch, at `dx` from its centre.
fn hertz(dx: f64) -> f64 {
    (1.0 - (dx / PATCH_HALF).powi(2)).max(0.0).sqrt()
}

/// A tilted ledge top's normal (up and out) on the side `side` (+1 or -1); V section.
fn ledge_normal(l: &Layout, side: f64) -> Vector3<f64> {
    Vector3::new(0.0, side * l.v_sin, l.v_cos)
}

/// The V-section lower with its axles. A tilted contact presses along its normal and slides
/// against friction, and sim-soft holds nodes only along x, y and z, so: the heavy (+y) side's
/// patch is pinned (it may stick by friction; the split is printed), and the light side presses
/// along its normal alone. Two solves, the wheel loads and then a pressing force on the light
/// side; the light side's force is the multiple of the second that leaves no roll moment on the
/// heavy patch, which a contact can't carry. A light side that comes out pulling needs the
/// coupling bolt's preload (printed).
fn lower_vee(p: &FrameParams, length: f64, cell: f64, lam: f64) -> Result<(Fe, Contacts), String> {
    const UNIT: f64 = 1000.0;
    let t = Instant::now();
    let f = frame::build(0.0, length, *p);
    let l = f.layout;
    let body = f.lower.clone().union(f.axles.clone());
    let (m, vol_err) = mesh(&body, cell)?;
    let on_patch = |q: &Vector3<f64>| {
        l.along(q.y, q.z) > -0.02 && l.across(q.y, q.z).abs() < l.half_t + 0.05 && q.y.abs() > l.y_out && (q.x - l.xc).abs() < PATCH_HALF
    };
    let heavy = fem::pick(&m, |q| q.y > 0.0 && on_patch(q));
    let light = fem::pick(&m, |q| q.y < 0.0 && on_patch(q));
    if heavy.is_empty() || light.is_empty() {
        return Err(format!("lower (V): {} heavy and {} light contact nodes", heavy.len(), light.len()));
    }
    let held: std::collections::BTreeSet<u32> = heavy.iter().copied().collect();
    let loads = wheel_loads(&m, &l, p, lam, &|v| held.contains(&v));
    let n_light = ledge_normal(&l, -1.0);
    let w: Vec<f64> = light.iter().map(|&v| hertz(m.rest[v as usize].x - l.xc)).collect();
    let wsum: f64 = w.iter().sum();
    // The upper pressing on the light ledge pushes the lower against the ledge's normal.
    let unit: Vec<(u32, Vector3<f64>)> = light.iter().zip(&w).map(|(&v, wi)| (v, -n_light * UNIT * *wi / wsum)).collect();
    println!("  lower (V): {} heavy-side patch nodes (held), {} light-side patch nodes, {} load nodes", heavy.len(), light.len(), loads.len());
    let (xa, ra) = fem::solve_with_reactions(&m, heavy.clone(), Vec::new(), &loads)?;
    let (xb, rb) = fem::solve_with_reactions(&m, heavy.clone(), Vec::new(), &unit)?;
    let c: Vector3<f64> = heavy.iter().map(|&v| m.rest[v as usize]).sum::<Vector3<f64>>() / heavy.len() as f64;
    let roll_moment = |r: &[f64]| {
        heavy
            .iter()
            .map(|&v| {
                let (i, q) = (3 * v as usize, m.rest[v as usize] - c);
                q.y * r[i + 2] - q.z * r[i + 1]
            })
            .sum::<f64>()
    };
    let k = -roll_moment(&ra) / roll_moment(&rb);
    let x: Vec<f64> = (0..xa.len()).map(|i| xa[i] + k * (xb[i] - m.rest[i / 3][i % 3])).collect();
    let r: Vec<f64> = ra.iter().zip(&rb).map(|(a, b)| a + k * b).collect();
    let n_minus = k * UNIT;
    let contacts = Contacts {
        bearing: Vec::new(),
        patch: heavy
            .iter()
            .map(|&v| {
                let i = 3 * v as usize;
                (m.rest[v as usize], Vector3::new(r[i], r[i + 1], r[i + 2]))
            })
            .chain(light.iter().zip(&w).map(|(&v, wi)| (m.rest[v as usize], n_light * n_minus * *wi / wsum)))
            .collect(),
    };
    let applied: Vector3<f64> = loads.iter().map(|(_, f)| f).sum();
    let got = contacts.total();
    let fh: Vector3<f64> = contacts.patch.iter().filter(|(q, _)| q.y > 0.0).map(|(_, f)| f).sum();
    let nh = ledge_normal(&l, 1.0);
    let press = fh.dot(&nh);
    let slide = (fh - press * nh).norm();
    println!(
        "  lower (V): loads ({:.1}, {:.1}, {:.1}) N; contact forces ({:.1}, {:.1}, {:.1}) N; roll moment left on the heavy patch {:.1} N mm",
        applied.x,
        applied.y,
        applied.z,
        got.x,
        got.y,
        got.z,
        roll_moment(&r)
    );
    println!(
        "  lower (V): +y contact ({:.0}, {:.0}, {:.0}) N = {press:.0} N pressing, {slide:.0} N sliding (friction {:.2}); -y contact {n_minus:.0} N pressing{}",
        fh.x,
        fh.y,
        fh.z,
        slide / press,
        if n_minus < 0.0 { format!(" (pulls: needs a bolt preload of at least {:.0} N)", -n_minus * 2.0 * l.v_cos) } else { String::new() }
    );
    let near_patch = Near::new(heavy.iter().chain(&light).map(|&v| m.rest[v as usize]), 2.0);
    let top = top_stresses(&m, &x, &l, &|q| in_axle(&l, p, q) || near_patch.hit(q));
    if std::env::var("SIZE_DEBUG").is_ok() {
        // How the lower's section moves at each axle: the rails' outer faces just above the axle
        // boss and just under the rail top, each side. A rigid roll moves the top's u_y by
        // -roll x height; the rest is the section racking (the rails leaning, not turning).
        for &ax in &l.axle_x {
            let band = |side: f64, z0: f64, z1: f64| {
                let vs = fem::pick(&m, |q| (q.x - ax).abs() < 3.0 && (q.y * side - l.y_out).abs() < 0.05 && q.z > z0 && q.z < z1);
                let u = vs.iter().map(|&v| fem::disp(&m, &x, v)).sum::<Vector3<f64>>() / vs.len().max(1) as f64;
                (u, vs.len())
            };
            let zb = (l.axle_z + l.r_boss + 1.0, l.axle_z + l.r_boss + 4.0);
            let zt = (l.rail_top - 4.0, l.rail_top - 1.0);
            let (bp, bm, tp, tm) = (band(1.0, zb.0, zb.1), band(-1.0, zb.0, zb.1), band(1.0, zt.0, zt.1), band(-1.0, zt.0, zt.1));
            let w = 2.0 * l.y_out;
            let (roll_b, roll_t) = ((bp.0.z - bm.0.z) / w, (tp.0.z - tm.0.z) / w);
            let (uy_b, uy_t) = (0.5 * (bp.0.y + bm.0.y), 0.5 * (tp.0.y + tm.0.y));
            let h = 0.5 * (zt.0 + zt.1) - 0.5 * (zb.0 + zb.1);
            println!(
                "  DEBUG lower x {:+.0}: u_y low {uy_b:.3} high {uy_t:.3} mm; roll low {roll_b:.2e} high {roll_t:.2e} rad; racking (u_y high - low + roll x {h:.0} mm) {:.3} mm ({} {} {} {} nodes)",
                ax - l.xc,
                uy_t - uy_b + 0.5 * (roll_b + roll_t) * h,
                bp.1,
                bm.1,
                tp.1,
                tm.1
            );
        }
    }
    let fe = Fe { top, axle_dy: axle_dy(&m, &x, &l, p), dof: 3 * m.rest.len(), vol_err, seconds: t.elapsed().as_secs_f64() };
    Ok((fe, contacts))
}

/// How the upper's contact faces move, as a rigid motion of the lower riding on them.
#[derive(Debug, Clone, Copy, Default)]
pub struct Carry {
    /// Sideways shift (at the frame centre) and yaw (mm per mm of x), from the side bearing.
    pub dy: f64,
    pub yaw: f64,
    /// Roll (rad, + raises the +y side), from the two contact patches' vertical motion.
    pub roll: f64,
    /// Height of the side bearing's load centre, mm (absolute z).
    pub z_bearing: f64,
}

impl Carry {
    /// Sideways shift this carries to a point of the lower at (x - centre, z).
    pub fn at(&self, dx: f64, z: f64) -> f64 {
        self.dy + self.yaw * dx + self.roll * (self.z_bearing - z)
    }
}

/// The rocker's upper with its ties, clamped on the web top at the boot mounts (or, with
/// `sole`, along its whole length: the upper bonded to a custom boot's sole), loaded by the
/// lower's contact forces. Returns its stresses (`axle_dy` = the shift it carries the axles by).
pub fn upper(p: &FrameParams, length: f64, cell: f64, contacts: &Contacts, sole: bool, clamp_hw: f64) -> Result<(Fe, Carry), String> {
    let t = Instant::now();
    let f = frame::build(0.0, length, *p);
    let l = f.layout;
    let mut body = f.upper.clone();
    if let Some(ties) = &f.ties {
        body = body.union(ties.clone());
    }
    let (m, vol_err) = mesh(&body, cell)?;
    let tol = 1e-3;
    // `clamp_hw` > 0: only the web top within that |y| is held (a narrower boot mount).
    let clamp = fem::pick(&m, |q| q.z > l.z_s - tol && (sole || in_mount(&l, q.x)) && (clamp_hw <= 0.0 || q.y.abs() < clamp_hw + tol));
    // The walls' inner faces along their feet, and the arcs' lowest stretch.
    let face = fem::pick(&m, |q| (q.y.abs() - l.y_wi).abs() < tol && q.z - band_bottom(p, &l, q.x) < p.wall_bot_h + 1.0);
    let foot = if p.vee {
        fem::pick(&m, |q| l.across(q.y, q.z).abs() < l.half_t + 0.05 && l.up_wall(q.y, q.z) < l.z_t + 1.1 && (q.x - l.xc).abs() < PATCH_HALF)
    } else {
        fem::pick(&m, |q| {
            q.y.abs() > l.y_wi - tol && q.y.abs() < l.y_wo + tol && q.z < l.z_t + 1.1 && (q.x - l.xc).abs() < PATCH_HALF
        })
    };
    if foot.is_empty() || (face.is_empty() && !contacts.bearing.is_empty()) {
        return Err(format!("upper: {} bearing-face nodes, {} contact nodes", face.len(), foot.len()));
    }
    // Side bearing: the lower's forces summed in 4 x 3 mm bins per side (z from the band's
    // bottom), each spread equally on the upper's face nodes in the same bin (the four nearest
    // face nodes if the bin has none).
    let (bx, bz) = (4.0, 3.0);
    let bin = |q: &Vector3<f64>| {
        (q.y > 0.0, ((q.x - l.xu0) / bx).floor() as i64, ((q.z - band_bottom(p, &l, q.x)) / bz).floor() as i64)
    };
    let mut sums: BTreeMap<(bool, i64, i64), Vector3<f64>> = BTreeMap::new();
    for (q, fv) in &contacts.bearing {
        *sums.entry(bin(q)).or_insert_with(Vector3::zeros) += fv;
    }
    let mut members: BTreeMap<(bool, i64, i64), Vec<u32>> = BTreeMap::new();
    for &v in &face {
        members.entry(bin(&m.rest[v as usize])).or_default().push(v);
    }
    let mut add: BTreeMap<u32, Vector3<f64>> = BTreeMap::new();
    let mut fallback = 0;
    for (key, fv) in &sums {
        let vs = match members.get(key) {
            Some(vs) => vs.clone(),
            None => {
                fallback += 1;
                let cx = l.xu0 + (key.1 as f64 + 0.5) * bx;
                let c = Vector3::new(cx, if key.0 { l.y_wi } else { -l.y_wi }, band_bottom(p, &l, cx) + (key.2 as f64 + 0.5) * bz);
                let mut by: Vec<u32> = face.clone();
                by.sort_by(|a, b| (m.rest[*a as usize] - c).norm().total_cmp(&(m.rest[*b as usize] - c).norm()));
                by.truncate(4);
                by
            }
        };
        for v in &vs {
            *add.entry(*v).or_insert_with(Vector3::zeros) += fv / vs.len() as f64;
        }
    }
    let bearing_nodes: Vec<(u32, Vector3<f64>)> = add.iter().map(|(&v, &f)| (v, f)).collect();
    // Contact patches: each side's total spread over that side's arc nodes (the bottom 1 mm of the
    // foot) with Hertz's elliptical weight along x.
    let mut patch_nodes: Vec<(u32, Vector3<f64>)> = Vec::new();
    for side in [false, true] {
        let total: Vector3<f64> = contacts.patch.iter().filter(|(q, _)| (q.y > 0.0) == side).map(|(_, f)| f).sum();
        let vs: Vec<(u32, f64)> = foot
            .iter()
            .copied()
            .filter(|&v| (m.rest[v as usize].y > 0.0) == side)
            .map(|v| (v, (1.0 - ((m.rest[v as usize].x - l.xc) / PATCH_HALF).powi(2)).max(0.0).sqrt()))
            .collect();
        let wsum: f64 = vs.iter().map(|(_, w)| w).sum();
        patch_nodes.extend(vs.iter().map(|&(v, w)| (v, total * w / wsum)));
    }
    let mut loads: BTreeMap<u32, Vector3<f64>> = BTreeMap::new();
    for (v, f) in bearing_nodes.iter().chain(&patch_nodes) {
        *loads.entry(*v).or_insert_with(Vector3::zeros) += f;
    }
    let clamped: std::collections::BTreeSet<u32> = clamp.iter().copied().collect();
    let loads: Vec<(u32, Vector3<f64>)> = loads.into_iter().filter(|(v, _)| !clamped.contains(v)).collect();
    let applied: Vector3<f64> = loads.iter().map(|(_, f)| f).sum();
    println!(
        "  upper: {} clamp nodes; side bearing on {} face nodes ({} bins by nearest node), contact on {} arc nodes; loads ({:.1}, {:.1}, {:.1}) N",
        clamp.len(),
        bearing_nodes.len(),
        fallback,
        patch_nodes.len(),
        applied.x,
        applied.y,
        applied.z
    );
    let near = Near::new(clamp.iter().chain(patch_nodes.iter().map(|(v, _)| v)).map(|&v| m.rest[v as usize]), 3.0);
    let x = fem::solve(&m, clamp, Vec::new(), &loads)?;
    let top = top_stresses(&m, &x, &l, &|q| near.hit(q));
    if std::env::var("SIZE_DEBUG").is_ok() {
        for side in [true, false] {
            let pf: Vector3<f64> = contacts.patch.iter().filter(|(q, _)| (q.y > 0.0) == side).map(|(_, f)| f).sum();
            println!("  DEBUG side {}: contact patch force ({:.0}, {:.0}, {:.0}) N", if side { "+y" } else { "-y" }, pf.x, pf.y, pf.z);
            let mut row = String::new();
            for x0 in (-100..100).step_by(10) {
                let fy: f64 = contacts
                    .bearing
                    .iter()
                    .filter(|(q, _)| (q.y > 0.0) == side && q.x - l.xc >= x0 as f64 && q.x - l.xc < x0 as f64 + 10.0)
                    .map(|(_, f)| f.y)
                    .sum();
                let _ = std::fmt::Write::write_fmt(&mut row, format_args!(" {fy:.0}"));
            }
            println!("  DEBUG side bearing Fy per 10 mm of x from -100:{row}");
            let mut col = String::new();
            for z0 in 0..4 {
                let fy: f64 = contacts
                    .bearing
                    .iter()
                    .filter(|(q, _)| (q.y > 0.0) == side && ((q.z - l.z_t) / 3.0).floor() as i64 == z0)
                    .map(|(_, f)| f.y)
                    .sum();
                let _ = std::fmt::Write::write_fmt(&mut col, format_args!(" {fy:.0}"));
            }
            println!("  DEBUG side bearing Fy per 3 mm of z from the ledge top:{col}");
        }
        // Where the sideways give builds up, at a few x: the web's top (free between the
        // mounts), the walls' roots (just under the web, inner face) and their feet.
        for xs in [0.0, 30.0, 60.0] {
            let mean_uy = |pred: &dyn Fn(&Vector3<f64>) -> bool| {
                let vs = fem::pick(&m, |q| (q.x - l.xc - xs).abs() < 2.0 && pred(q));
                (vs.iter().map(|&v| fem::disp(&m, &x, v).y).sum::<f64>() / vs.len().max(1) as f64, vs.len())
            };
            let web = mean_uy(&|q| q.z > l.z_s - 0.5 && q.y.abs() < l.wall_out(l.z_s));
            let root = mean_uy(&|q| (l.across(q.y, q.z) + l.half_t).abs() < 0.3 && (q.z - (l.z_wb - p.wall_top_h)).abs() < 1.0);
            let foot = mean_uy(&|q| (l.across(q.y, q.z) + l.half_t).abs() < 0.3 && (l.up_wall(q.y, q.z) - band_bottom(p, &l, q.x) - 2.0).abs() < 1.5);
            println!(
                "  DEBUG x {xs:+.0}: u_y web top {:.3} ({} nodes), wall root {:.3} ({}), wall foot {:.3} ({})",
                web.0, web.1, root.0, root.1, foot.0, foot.1
            );
        }
        let ns = nodal_stresses(&m, &x);
        for (s, q) in top.iter().take(3) {
            let pos = Vector3::new(q.x + l.xc, q.y, q.z + l.axle_z);
            if let Some(v) = (0..m.rest.len()).find(|&v| (m.rest[v] - pos).norm() < 1e-9)
                && let Some(t) = ns[v]
            {
                println!(
                    "  DEBUG upper {s:.0} MPa: sxx {:.0} syy {:.0} szz {:.0} sxy {:.0} syz {:.0} sxz {:.0}; displacement {:?}",
                    t[(0, 0)],
                    t[(1, 1)],
                    t[(2, 2)],
                    t[(0, 1)],
                    t[(1, 2)],
                    t[(0, 2)],
                    fem::disp(&m, &x, v as u32).map(|d| (d * 1000.0).round() / 1000.0).as_slice()
                );
            }
        }
    }

    if bearing_nodes.is_empty() {
        // V section: the lower rides on the two tilted patches alone, as `lower_vee` holds it: it
        // moves with the heavy (+y) foot (stuck by friction) and follows the light foot along
        // the ledge's normal only. Hertz-weighted means over each foot; yaw from the heavy foot.
        let side_mean = |side: f64| {
            let vs: Vec<(Vector3<f64>, Vector3<f64>, f64)> = patch_nodes
                .iter()
                .filter(|(v, _)| m.rest[*v as usize].y * side > 0.0)
                .map(|(v, _)| {
                    let q = m.rest[*v as usize];
                    (q, fem::disp(&m, &x, *v), hertz(q.x - l.xc))
                })
                .collect();
            let ws: f64 = vs.iter().map(|t| t.2).sum();
            let c = vs.iter().map(|t| t.0 * t.2).sum::<Vector3<f64>>() / ws;
            let u = vs.iter().map(|t| t.1 * t.2).sum::<Vector3<f64>>() / ws;
            let yaw = vs.iter().map(|t| t.2 * (t.0.x - c.x) * (t.1.y - u.y)).sum::<f64>()
                / vs.iter().map(|t| t.2 * (t.0.x - c.x).powi(2)).sum::<f64>().max(1e-12);
            (c, u, yaw)
        };
        let ((ch, uh, yaw), (cl, ul, _)) = (side_mean(1.0), side_mean(-1.0));
        let nl = ledge_normal(&l, -1.0);
        // Rigid roll about the heavy foot: u(q) = uh + roll * (0, -(z - ch.z), y - ch.y).
        let v = Vector3::new(0.0, -(cl.z - ch.z), cl.y - ch.y);
        let roll = (ul.dot(&nl) - uh.dot(&nl)) / v.dot(&nl);
        let carry = Carry { dy: uh.y, yaw, roll, z_bearing: ch.z };
        let shift: [f64; 4] = std::array::from_fn(|k| carry.at(l.axle_x[k] - l.xc, l.axle_z));
        let fe = Fe { top, axle_dy: shift, dof: 3 * m.rest.len(), vol_err, seconds: t.elapsed().as_secs_f64() };
        return Ok((fe, carry));
    }
    // The lower rides on the upper's faces: sideways shift and yaw from the side bearing
    // (least squares, weighted by each node's sideways load), roll from the patches.
    let (mut sw, mut sx, mut sxx, mut su, mut sxu, mut sz) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for (v, f) in &bearing_nodes {
        let w = f.y.abs();
        let q = m.rest[*v as usize];
        let dx = q.x - l.xc;
        let u = fem::disp(&m, &x, *v).y;
        sw += w;
        sx += w * dx;
        sxx += w * dx * dx;
        su += w * u;
        sxu += w * dx * u;
        sz += w * q.z;
    }
    let det = sw * sxx - sx * sx;
    let yaw = (sw * sxu - sx * su) / det;
    let dy = (su - yaw * sx) / sw;
    let side_uz = |side: bool| {
        let vs: Vec<&(u32, Vector3<f64>)> = patch_nodes.iter().filter(|(v, _)| (m.rest[*v as usize].y > 0.0) == side).collect();
        let n = vs.len().max(1) as f64;
        let uz = vs.iter().map(|(v, _)| fem::disp(&m, &x, *v).z).sum::<f64>() / n;
        let y = vs.iter().map(|(v, _)| m.rest[*v as usize].y).sum::<f64>() / n;
        (uz, y)
    };
    let ((uz_p, y_p), (uz_m, y_m)) = (side_uz(true), side_uz(false));
    let carry = Carry { dy, yaw, roll: (uz_p - uz_m) / (y_p - y_m), z_bearing: sz / sw };
    let shift: [f64; 4] = std::array::from_fn(|k| carry.at(l.axle_x[k] - l.xc, l.axle_z));
    let fe = Fe { top, axle_dy: shift, dof: 3 * m.rest.len(), vol_err, seconds: t.elapsed().as_secs_f64() };
    Ok((fe, carry))
}

/// The rocker, lower then upper; `axle_dy` of the result is the two added.
pub struct Rocker {
    pub lower: Fe,
    pub upper: Fe,
    pub carry: Carry,
    pub axle_dy: [f64; 4],
}

/// `cell_up`: the upper's mesh size (0 = `cell`); leaning walls need a finer one than upright.
#[allow(clippy::too_many_arguments)]
pub fn rocker(p: &FrameParams, length: f64, cell: f64, cell_up: f64, lam: f64, sole: bool, clamp_hw: f64) -> Result<Rocker, String> {
    let (lo, contacts) = lower(p, length, cell, lam)?;
    let (up, carry) = upper(p, length, if cell_up > 0.0 { cell_up } else { cell }, &contacts, sole, clamp_hw)?;
    let axle_dy = std::array::from_fn(|k| lo.axle_dy[k] + up.axle_dy[k]);
    Ok(Rocker { lower: lo, upper: up, carry, axle_dy })
}

/// Set a frame parameter by name (for sweeps from the command line).
pub fn set_param(p: &mut FrameParams, key: &str, v: f64) -> Result<(), String> {
    match key {
        "rail_t" => p.rail_t = v,
        "wall_t" => p.wall_t = v,
        "web_t" => p.web_t = v,
        "diag_w" => p.diag_w = v,
        "post_w" => p.post_w = v,
        "top_h" => p.top_h = v,
        "boss_w" => p.boss_w = v,
        "ledge_w" => p.ledge_w = v,
        "ledge_t" => p.ledge_t = v,
        "ledge_len" => p.ledge_len = v,
        "bot_h_out" => p.bot_h_out = v,
        "wall_top_h" => p.wall_top_h = v,
        "wall_bot_h" => p.wall_bot_h = v,
        "wall_diag_w" => p.wall_diag_w = v,
        "tie_d" => p.tie_d = v,
        "upper_len" => p.upper_len = v,
        "wall_panels" => p.wall_panels = v.round().max(1.0) as usize,
        "web_windows" => p.web_windows = v.round().max(0.0) as usize,
        "vee" => p.vee = v > 0.5,
        "web_truss" => p.web_truss = v.round().max(0.0) as usize,
        "deck_panels" => p.deck_panels = v.round().max(0.0) as usize,
        "deck_t" => p.deck_t = v,
        "bulk_t" => p.bulk_t = v,
        "rib_d" => p.rib_d = v,
        "rib_t" => p.rib_t = v,
        "rib_taper" => p.rib_taper = v,
        "rib_chords" => p.rib_chords = v.round().clamp(0.0, 2.0) as usize,
        "side_gap" => p.side_gap = v,
        _ => return Err(format!("unknown parameter {key}")),
    }
    Ok(())
}
