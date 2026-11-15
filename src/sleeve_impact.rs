//! U2, the padded sleeve, step 2 (2026-11-14): impacts. Jon: "just as much as falling, it needs to
//! handle taking hard shots from ice hockey pucks. ice hockey elbow pads will need to be more heavy
//! duty vs roller hockey which has a lighter puck". On D3's solver (`impact`: CortenForge's
//! explicit dynamics, the puck a free body), a section of the sleeve's pad on a rigid support:
//!
//! - **The elbow's point** on EN 1621-1's anvil, a 50 mm radius dome. EN 1621-1 (motorcyclists'
//!   limb protectors) drops a 5 kg striker with a flat 40 x 30 mm face at 50 J on the protector
//!   and reads the force through it in a load cell under the anvil: level 1 a mean of nine under
//!   35 kN with none over 50, level 2 a mean under 20 kN (secondary sources: SATRA, PVA,
//!   Wikipedia, RAD; RESEARCH.md 30). Its drop stands for a fall on the elbow; the same anvil
//!   takes the pucks.
//! - **The slash guard's plate** on the forearm, a cylinder.
//!
//! Hits: the drop; the ice puck, cold (`impact::ice_puck`), and the roller puck (125 g, no data on
//! its stiffness: as stiff as the cold ice puck, an assumption), rim first at a hard slap shot's
//! speed. The force through the pad is the held nodes' (the load cell's). As D3's: the support is
//! rigid and the foam elastic (no loss), so absolute forces read high; the EN 1621-1 test's anvil is
//! rigid too.
//!
//! The foam: a rate-dependent one, as G-Form's, with Poron XRD's moduli at 244 kg/m3 (Bhagavathula
//! et al. 2022, RESEARCH.md 20): 1.73 MPa at 1 /s, 9.3 at ~210 /s, 24 at ~3950 /s, read at each
//! hit's rate (its speed over the foam's thickness), log-linear between. The insert: PA 12 (MJF),
//! outside the foam (D3: without a hard layer a puck's rim crushed through the foam).

use sim_soft_explicit::cpu::f64::CpuExecutor;
use sim_soft_explicit::f64::Material;
use sim_soft_explicit::stepping::{Stepper, StepperConfig};
use sim_soft_explicit::ExplicitModel;

use crate::impact::{Layer, Puck, box_mesh, ice_puck, material, obstacle, puck_obstacle, run_free};

/// Poron XRD at 244 kg/m3 (Bhagavathula et al. 2022): (strain rate /s, modulus Pa).
const PORON: [(f64, f64); 3] = [(1.0, 1.73e6), (210.0, 9.3e6), (3950.0, 24.0e6)];
const PORON_DENSITY: f64 = 244.0;

/// Poron XRD's modulus at a strain rate, log-linear between the measured ones (held beyond).
pub fn poron_e(rate: f64) -> f64 {
    let r = rate.clamp(PORON[0].0, PORON[2].0);
    let k = if r <= PORON[1].0 { 0 } else { 1 };
    let (a, b) = (PORON[k], PORON[k + 1]);
    let f = (r / a.0).ln() / (b.0 / a.0).ln();
    a.1 * (b.1 / a.1).powf(f)
}

/// EN 1621-1's drop: 5 kg, a flat 40 x 30 mm face, 50 J.
pub const DROP_MASS: f64 = 5.0;
pub const DROP_ENERGY: f64 = 50.0;
/// EN 1621-1's anvil, a dome of 50 mm radius.
pub const ANVIL_R: f64 = 0.050;
/// The forearm under the plate: a cylinder of this radius (mid-forearm, the stand-in's girth
/// ~250 mm round: an assumption).
pub const FOREARM_R: f64 = 0.040;
/// A hard slap shot: varsity players, 30.0 m/s (Pearsall et al. 2001; RESEARCH.md 19).
pub const SHOT: f64 = 30.0;
/// A pro's slap shot: 125.2 km/h, pro and semi-pro players on ice, radar (Bežák & Přidal 2017;
/// RESEARCH.md 19).
pub const PRO_SHOT: f64 = 125.2 / 3.6;

/// The roller puck: 125 g (Inline Hockey Australia's 120-130 g; RESEARCH.md 19), as stiff as the
/// cold ice puck (fitted at its mass to the same contact time and restitution).
pub fn roller_puck() -> Puck {
    Puck::calibrated(0.125, 0.15, 24.6, 0.41e-3, 0.267)
}

/// A pad section over a rigid dome (a sphere of radius `r_in`, m): a square patch (a cube-sphere
/// face round +x) of `layers`, `half_angle` deg either way, cells about `cell` m. Positions,
/// elements, a material per element, and the held nodes (on the dome).
pub fn dome_coupon(r_in: f64, layers: &[Layer], half_angle: f64, cell: f64) -> (Vec<[f64; 3]>, Vec<[u32; 4]>, Vec<Material>, Vec<bool>) {
    let mut radii = vec![r_in];
    let mut layer_of_cell = Vec::new();
    for (li, l) in layers.iter().enumerate() {
        let r0 = *radii.last().unwrap_or(&r_in);
        for c in 1..=l.cells {
            radii.push(r0 + l.t * c as f64 / l.cells as f64);
            layer_of_cell.push(li);
        }
    }
    let r_out = *radii.last().unwrap_or(&r_in);
    let n = ((2.0 * half_angle.to_radians() * r_out) / cell).round().max(2.0) as usize;
    let nr = radii.len() - 1;
    let (unit, tets) = box_mesh([n, n, nr], [1.0, 1.0, 1.0]);
    let pos: Vec<[f64; 3]> = unit
        .iter()
        .map(|u| {
            let a = (-half_angle + 2.0 * half_angle * u[0]).to_radians();
            let b = (-half_angle + 2.0 * half_angle * u[1]).to_radians();
            let (dx, dy, dz) = (1.0, a.tan(), b.tan());
            let len = (dx * dx + dy * dy + dz * dz).sqrt();
            let k = (u[2] * nr as f64).round() as usize;
            let r = radii[k.min(nr)];
            [r * dx / len, r * dy / len, r * dz / len]
        })
        .collect();
    let mut fixed = Vec::with_capacity(tets.len());
    let mut mats = Vec::with_capacity(tets.len());
    for t in tets {
        let p = t.map(|i| pos[i as usize]);
        let e = |a: usize| [p[a][0] - p[0][0], p[a][1] - p[0][1], p[a][2] - p[0][2]];
        let (a, b, c) = (e(1), e(2), e(3));
        let vol = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
        fixed.push(if vol > 0.0 { t } else { [t[0], t[2], t[1], t[3]] });
        let rc = p.iter().map(|q| (q[0] * q[0] + q[1] * q[1] + q[2] * q[2]).sqrt()).sum::<f64>() / 4.0;
        let cl = radii.windows(2).position(|w| rc >= w[0] && rc <= w[1]).unwrap_or(0);
        mats.push(layers[layer_of_cell[cl.min(layer_of_cell.len() - 1)]].mat);
    }
    let held = pos.iter().map(|q| ((q[0] * q[0] + q[1] * q[1] + q[2] * q[2]).sqrt() - r_in).abs() < 1e-9).collect();
    (pos, fixed, mats, held)
}

/// EN 1621-1's striker as a rigid obstacle in its body frame: a box 30 mm (y) by 40 mm (z)
/// across, its flat face at x = 0 toward -x, its edges rounded 2 mm; its distance baked over the
/// box it can reach the pad from, at `at`.
pub fn striker_obstacle(cell: f64, at: [f64; 3]) -> Result<sim_soft_explicit::executor::Obstacle, String> {
    let (hy, hz, rr, depth) = (0.015, 0.020, 0.002, 0.030);
    let sdf = move |p: [f64; 3]| {
        let q = [(p[0] - 0.5 * depth).abs() - (0.5 * depth - rr), p[1].abs() - (hy - rr), p[2].abs() - (hz - rr)];
        let o = (q[0].max(0.0).powi(2) + q[1].max(0.0).powi(2) + q[2].max(0.0).powi(2)).sqrt();
        o + q[0].max(q[1]).max(q[2]).min(0.0) - rr
    };
    let lo = [-0.045, -0.045, -0.05];
    let hi = [0.003, 0.045, 0.05];
    let snap = |v: f64| (v / cell).round() * cell;
    obstacle(lo.map(snap), hi.map(snap), cell, at, sdf)
}

/// What hits.
#[derive(Clone, Copy, Debug)]
pub enum Hit {
    /// EN 1621-1's drop.
    Drop,
    /// A puck (its model, its speed m/s), rim first.
    Puck(Puck, f64),
}

impl Hit {
    pub fn speed(&self) -> f64 {
        match self {
            Hit::Drop => (2.0 * DROP_ENERGY / DROP_MASS).sqrt(),
            Hit::Puck(_, v) => *v,
        }
    }
    pub fn energy(&self) -> f64 {
        match self {
            Hit::Drop => DROP_ENERGY,
            Hit::Puck(p, v) => 0.5 * p.mass * v * v,
        }
    }
}

/// Where: the elbow's point on the anvil (a dome), or the forearm (a cylinder, ±60 deg, 100 mm).
#[derive(Clone, Copy, Debug)]
pub enum Support {
    Anvil,
    Forearm,
}

/// One case: its name, where, what hits, the foam's thickness (m), the insert's (m, 0 none).
#[derive(Clone, Copy, Debug)]
pub struct Case {
    pub name: &'static str,
    pub support: Support,
    pub hit: Hit,
    pub foam: f64,
    pub insert: f64,
}

/// What a run gives.
#[derive(Clone, Debug)]
pub struct Outcome {
    pub foam_e: f64,
    pub nodes: usize,
    pub elems: usize,
    pub dt: f64,
    /// Peak contact force on the impactor, and through the pad (on the support), N.
    pub contact: f64,
    pub through: f64,
    /// The impactor's face's deepest travel into the pad, m; its speed at the end, m/s.
    pub deepest: f64,
    pub end_speed: f64,
    /// Element-steps with an element turned inside out: crushed past what the mesh takes.
    pub inverted: u64,
    /// The run ended early (a blow-up) at this time, s.
    pub stopped: Option<f64>,
    pub t_end: f64,
    pub seconds: f64,
}

/// Run one case.
pub fn run_case(c: &Case) -> Result<Outcome, String> {
    let t0 = std::time::Instant::now();
    let rate = c.hit.speed() / c.foam;
    let foam_e = poron_e(rate);
    let foam = material(foam_e, 0.15, PORON_DENSITY);
    let pa12 = material(1.8e9, 0.38, 1010.0);
    let mut layers = vec![Layer { t: c.foam, cells: ((c.foam / 0.002).round() as usize).max(2), mat: foam }];
    if c.insert > 0.0 {
        layers.push(Layer { t: c.insert, cells: 3, mat: pa12 });
    }
    let r_in = match c.support {
        Support::Anvil => ANVIL_R,
        Support::Forearm => FOREARM_R,
    };
    let (pos, tets, mats, held) = match c.support {
        Support::Anvil => dome_coupon(r_in, &layers, 40.0, 0.0015),
        Support::Forearm => crate::impact::cyl_coupon(r_in, &layers, 60.0, 0.10, 0.0015),
    };
    let held_idx: Vec<usize> = (0..held.len()).filter(|&i| held[i]).collect();
    let r_out = r_in + layers.iter().map(|l| l.t).sum::<f64>();
    let (nodes, elems) = (pos.len(), tets.len());
    let model = ExplicitModel::new(pos, tets, mats, held).map_err(|e| format!("model: {e:?}"))?;
    let (body, at, t_end) = match c.hit {
        Hit::Drop => {
            let at = [r_out + 0.0002, 0.0, 0.0];
            (Puck::rigid(DROP_MASS), at, 0.020)
        }
        Hit::Puck(p, _) => {
            let at = [r_out + 0.0381 + 0.0002, 0.0, 0.0];
            (p, at, 0.004)
        }
    };
    let ob = match c.hit {
        Hit::Drop => striker_obstacle(0.0005, at)?,
        Hit::Puck(..) => puck_obstacle(0.0381, 0.0127, 0.0005, at)?,
    };
    let exec = CpuExecutor::new(&model, &ob).map_err(|e| format!("executor: {e:?}"))?;
    let chunk = 2;
    let mut cfg = StepperConfig::new(0.0);
    cfg.monitor_every = chunk as u64;
    let mut st = Stepper::new(exec, cfg, 0.0);
    let dt = st.dt();
    let run = run_free(&mut st, body, at, [-1.0, 0.0, 0.0], c.hit.speed(), t_end, chunk, 0.0005, Some((&held_idx, 5)))?;
    let last = run.last().copied();
    let stopped = last.filter(|s| s.t < t_end && s.speed > 0.0).map(|s| s.t);
    Ok(Outcome {
        foam_e,
        nodes,
        elems,
        dt,
        contact: run.iter().map(|s| s.force).fold(0.0, f64::max),
        through: run.iter().filter(|s| s.leg.is_finite()).map(|s| s.leg).fold(0.0, f64::max),
        deepest: run.iter().map(|s| s.travel).fold(0.0, f64::max),
        end_speed: last.map_or(0.0, |s| s.speed),
        inverted: last.map_or(0, |s| s.inverted),
        stopped,
        t_end: last.map_or(0.0, |s| s.t),
        seconds: t0.elapsed().as_secs_f64(),
    })
}

/// The cases: the elbow's point (ice and roller versions, the drop and the pucks), the plate.
pub fn cases() -> Vec<Case> {
    let ice = Hit::Puck(ice_puck(true), SHOT);
    let roller = Hit::Puck(roller_puck(), SHOT);
    let mm = |x: f64| x * 1e-3;
    vec![
        Case { name: "drop-10", support: Support::Anvil, hit: Hit::Drop, foam: mm(10.0), insert: 0.0 },
        Case { name: "drop-15", support: Support::Anvil, hit: Hit::Drop, foam: mm(15.0), insert: 0.0 },
        Case { name: "drop-15i", support: Support::Anvil, hit: Hit::Drop, foam: mm(15.0), insert: mm(1.5) },
        Case { name: "roller-10", support: Support::Anvil, hit: roller, foam: mm(10.0), insert: 0.0 },
        Case { name: "roller-15", support: Support::Anvil, hit: roller, foam: mm(15.0), insert: 0.0 },
        Case { name: "ice-15", support: Support::Anvil, hit: ice, foam: mm(15.0), insert: 0.0 },
        Case { name: "ice-15i", support: Support::Anvil, hit: ice, foam: mm(15.0), insert: mm(1.5) },
        Case { name: "ice-20i", support: Support::Anvil, hit: ice, foam: mm(20.0), insert: mm(1.5) },
        // Round 2 of the cases: the ice version's pad under the drop and a pro's slap shot (35
        // m/s: Bežák & Přidal 2017's pro and semi-pro mean, 125.2 km/h; RESEARCH.md 19).
        Case { name: "drop-20i", support: Support::Anvil, hit: Hit::Drop, foam: mm(20.0), insert: mm(1.5) },
        Case { name: "ice-20i-35", support: Support::Anvil, hit: Hit::Puck(ice_puck(true), PRO_SHOT), foam: mm(20.0), insert: mm(1.5) },
        Case { name: "plate-roller", support: Support::Forearm, hit: roller, foam: mm(8.0), insert: 0.0 },
        Case { name: "plate-roller-i", support: Support::Forearm, hit: roller, foam: mm(8.0), insert: mm(1.5) },
        Case { name: "plate-ice-i", support: Support::Forearm, hit: ice, foam: mm(8.0), insert: mm(1.5) },
        // The plate thicker: 12 mm with no insert (its half nearer the wrist, where an insert
        // fits poorly: the sleeve's step 1), with one, and the ice version at 15.
        Case { name: "plate-roller-12", support: Support::Forearm, hit: roller, foam: mm(12.0), insert: 0.0 },
        Case { name: "plate-roller-12i", support: Support::Forearm, hit: roller, foam: mm(12.0), insert: mm(1.5) },
        Case { name: "plate-ice-12i", support: Support::Forearm, hit: ice, foam: mm(12.0), insert: mm(1.5) },
        Case { name: "plate-ice-15i", support: Support::Forearm, hit: ice, foam: mm(15.0), insert: mm(1.5) },
    ]
}

fn line(c: &Case, o: &Outcome) -> String {
    let what = match c.hit {
        Hit::Drop => format!("EN 1621-1 drop, {:.0} kg at {:.2} m/s ({:.0} J)", DROP_MASS, c.hit.speed(), c.hit.energy()),
        Hit::Puck(p, v) => format!("{:.0} g puck at {v:.0} m/s ({:.0} J)", p.mass * 1e3, c.hit.energy()),
    };
    let on = match c.support {
        Support::Anvil => "the elbow's point (EN 1621-1's 50 mm dome)",
        Support::Forearm => "the forearm (40 mm cylinder)",
    };
    format!(
        "{}: {what} on {on}; foam {:.0} mm at {:.1} MPa{}: through the pad {:.2} kN (contact {:.2} kN), the face {:.1} mm in, {}{} ({} nodes, {} elements, {:.0} s)",
        c.name,
        c.foam * 1e3,
        o.foam_e / 1e6,
        if c.insert > 0.0 { format!(", a {:.1} mm PA 12 insert", c.insert * 1e3) } else { String::new() },
        o.through / 1e3,
        o.contact / 1e3,
        o.deepest * 1e3,
        if o.inverted > 0 { format!("elements crushed inside out ({} element-steps)", o.inverted) } else { "no element crushed".to_string() },
        o.stopped.map_or(String::new(), |t| format!(", the run blew up at {:.2} ms", t * 1e3)),
        o.nodes,
        o.elems,
        o.seconds
    )
}

/// `cargo run --release -- sleeveimpact [cases|all|report]`: the cases named (comma-separated),
/// each its line to out/sleeve_impact/, or all of them in turn; `report` gathers every case's
/// line from there, in order, into docs/elbow/sleeve/impact.txt (`all` writes it too). With
/// none, lists them. The runs are deterministic, so a case's line stands until its code changes.
pub fn run(which: Option<&str>) -> Result<(), String> {
    let all = cases();
    let Some(w) = which else {
        for c in &all {
            println!("{}", c.name);
        }
        return Ok(());
    };
    let out = std::path::Path::new("out/sleeve_impact");
    let docs = std::path::Path::new("docs/elbow/sleeve");
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    std::fs::create_dir_all(docs).map_err(|e| e.to_string())?;
    let report = || -> Result<(), String> {
        let mut lines = Vec::new();
        for c in &all {
            let l = std::fs::read_to_string(out.join(format!("{}.txt", c.name))).map_err(|_| format!("{}: not run yet", c.name))?;
            lines.push(l.trim_end().to_string());
        }
        std::fs::write(docs.join("impact.txt"), lines.join("\n") + "\n").map_err(|e| e.to_string())
    };
    if w == "report" {
        return report();
    }
    let names: Vec<&str> = w.split(',').map(str::trim).collect();
    let pick: Vec<Case> = if w == "all" { all.clone() } else { all.iter().copied().filter(|c| names.contains(&c.name)).collect() };
    if pick.is_empty() || (w != "all" && pick.len() != names.len()) {
        return Err(format!("no case among {w:?}"));
    }
    for c in &pick {
        let o = run_case(c)?;
        let l = line(c, &o);
        println!("{l}");
        std::fs::write(out.join(format!("{}.txt", c.name)), format!("{l}\n")).map_err(|e| e.to_string())?;
    }
    if w == "all" {
        report()?;
    }
    Ok(())
}
