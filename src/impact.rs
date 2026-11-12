//! Track D3: impacts, on CortenForge's explicit solver (`sim-soft-explicit`).
//!
//! The solver pushes a rigid obstacle along a pose track into a soft body. A puck isn't pushed:
//! it flies in and is slowed by what it hits. So the obstacle here is a free rigid body: every
//! `chunk` steps the contact force (the solver's monitor, averaged over the chunk) changes its
//! velocity, and the next chunk's pose track is set from the new velocity (`set_poses` keeps
//! the soft body's state). The coupling lags by one chunk, a few microseconds against
//! milliseconds of contact.
//!
//! Units: SI (m, kg, s, N, Pa), as the solver's materials are.
//!
//! Checked first (`check`), before any pad:
//! - a heavy flat impactor on a soft block (a spring): peak force v sqrt(k m), contact time
//!   pi sqrt(m / k), rebound at the impact speed;
//! - a thin PA 12 strip clamped at one end, set moving: its first natural frequency against
//!   Euler-Bernoulli, which shows how stiff the solver's four-node tetrahedra are in bending
//!   with 2, 3 and 4 elements through the thickness (thin shells are the pad's hard parts).

use sim_soft_explicit::cpu::f64::CpuExecutor;
use sim_soft_explicit::executor::{Executor, Obstacle};
use sim_soft_explicit::f64::{Material, Pose};
use sim_soft_explicit::fixtures::grid::bake;
use sim_soft_explicit::stepping::{Stepper, StepperConfig};
use sim_soft_explicit::ExplicitModel;

/// A material from Young's modulus and Poisson's ratio (Pa, -, kg/m3), elastic.
pub fn material(e: f64, nu: f64, density: f64) -> Material {
    Material {
        mu: e / (2.0 * (1.0 + nu)),
        lambda: e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu)),
        c2: 0.0,
        viscosity: 0.0,
        density,
    }
}

/// A box of `n` cells a side each way (sizes `size`, m, from the origin), each cube cut into
/// six tetrahedra round its main diagonal: positions and elements (positive volumes).
pub fn box_mesh(n: [usize; 3], size: [f64; 3]) -> (Vec<[f64; 3]>, Vec<[u32; 4]>) {
    let id = |i: usize, j: usize, k: usize| (k * (n[1] + 1) * (n[0] + 1) + j * (n[0] + 1) + i) as u32;
    let mut pos = Vec::new();
    for k in 0..=n[2] {
        for j in 0..=n[1] {
            for i in 0..=n[0] {
                pos.push([
                    size[0] * i as f64 / n[0] as f64,
                    size[1] * j as f64 / n[1] as f64,
                    size[2] * k as f64 / n[2] as f64,
                ]);
            }
        }
    }
    let mut tets = Vec::new();
    for k in 0..n[2] {
        for j in 0..n[1] {
            for i in 0..n[0] {
                let v = |a: usize, b: usize, c: usize| id(i + a, j + b, k + c);
                let (o, d) = (v(0, 0, 0), v(1, 1, 1));
                for t in [
                    [o, v(1, 0, 0), v(1, 1, 0), d],
                    [o, v(1, 0, 0), v(1, 0, 1), d],
                    [o, v(0, 1, 0), v(1, 1, 0), d],
                    [o, v(0, 1, 0), v(0, 1, 1), d],
                    [o, v(0, 0, 1), v(1, 0, 1), d],
                    [o, v(0, 0, 1), v(0, 1, 1), d],
                ] {
                    let p = t.map(|i| pos[i as usize]);
                    let e = |a: usize| [p[a][0] - p[0][0], p[a][1] - p[0][1], p[a][2] - p[0][2]];
                    let (a, b, c) = (e(1), e(2), e(3));
                    let vol = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
                    tets.push(if vol > 0.0 { t } else { [t[0], t[2], t[1], t[3]] });
                }
            }
        }
    }
    (pos, tets)
}

pub fn pose_at(t: [f64; 3]) -> Pose {
    Pose { qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0, tx: t[0], ty: t[1], tz: t[2] }
}

/// A rigid obstacle from a distance function in its body frame, baked over a box (m), posed at
/// `at`, frictionless.
pub fn obstacle(low: [f64; 3], high: [f64; 3], cell: f64, at: [f64; 3], distance: impl Fn([f64; 3]) -> f64) -> Result<Obstacle, String> {
    let (grid, values) = bake(low, high, cell, distance).map_err(|e| format!("bake: {e:?}"))?;
    Ok(Obstacle { grid, values, fine: None, start: 0.0, interval: 1.0, poses: vec![pose_at(at)], friction: 0.0 })
}

/// One read of a free impact.
#[derive(Clone, Copy, Debug)]
pub struct ImpactSample {
    pub t: f64,
    /// The contact force on the impactor along its direction of travel, N (positive slows it).
    pub force: f64,
    /// The impactor's travel along its direction since the start, and its speed, m and m/s.
    pub travel: f64,
    pub speed: f64,
    /// The force on the held nodes (the leg) along the direction of travel, N; NaN where not read.
    pub leg: f64,
    /// Element-steps so far with an element turned inside out (cumulative): crushing past what
    /// the mesh can take.
    pub inverted: u64,
}

/// Run a free impact: an obstacle of `mass` kg starts at `at` moving along `dir` (unit) at
/// `speed` m/s; every `chunk` steps the averaged contact force changes its speed. Stops at
/// `t_end` s, or once it has bounced clear (moving back, no contact) for `after` s.
#[allow(clippy::too_many_arguments)]
pub fn run_free(
    stepper: &mut Stepper<CpuExecutor>,
    puck: Puck,
    at: [f64; 3],
    dir: [f64; 3],
    speed: f64,
    t_end: f64,
    chunk: usize,
    after: f64,
    held: Option<(&[usize], usize)>,
) -> Result<Vec<ImpactSample>, String> {
    // The face (the obstacle) and, for a two-mass puck, the rest of it behind the rubber.
    let (mut x, mut v) = (0.0, speed);
    let (mut xb, mut vb) = (0.0, speed);
    let (mf, mb) = (puck.mass * puck.face, puck.mass * (1.0 - puck.face));
    let mut chunks = 0usize;
    let mut out = Vec::new();
    let mut clear_since: Option<f64> = None;
    let place = |s: f64| [at[0] + dir[0] * s, at[1] + dir[1] * s, at[2] + dir[2] * s];
    loop {
        let t0 = stepper.time();
        let span = chunk as f64 * stepper.dt();
        stepper
            .executor_mut()
            .set_poses(t0, span, &[pose_at(place(x)), pose_at(place(x + v * span))])
            .map_err(|e| format!("poses: {e:?}"))?;
        // A blow-up ends the run with what it recorded (the last sample says when), rather than
        // losing it: elements turned inside out under a crushing hit are what blow up.
        let mut failed = false;
        for _ in 0..chunk {
            if let Err(e) = stepper.step() {
                eprintln!("  run stopped at t {:.3} ms: {e}", stepper.time() * 1e3);
                failed = true;
                break;
            }
        }
        if failed {
            break;
        }
        let dt = stepper.time() - t0;
        let m = stepper.samples().last().ok_or("no monitor read")?.monitors;
        // The contact force on the soft body; on the impactor it is the negative.
        let f_on_soft = m.contact_force[0] * dir[0] + m.contact_force[1] * dir[1] + m.contact_force[2] * dir[2];
        if mb <= 0.0 {
            x += v * dt;
            v -= f_on_soft / puck.mass * dt;
        } else {
            // The contact force held over the chunk; the rubber between face and body in 20 steps.
            let h = dt / 20.0;
            for _ in 0..20 {
                let fs = puck.spring(xb - x, vb - v);
                v += (fs - f_on_soft) / mf * h;
                vb -= fs / mb * h;
                x += v * h;
                xb += vb * h;
            }
        }
        let v_cm = (mf * v + mb * vb) / puck.mass;
        let t = stepper.time();
        chunks += 1;
        let leg = match held {
            Some((nodes, every)) if chunks % every.max(1) == 0 => {
                let f = held_force(stepper, nodes);
                f[0] * dir[0] + f[1] * dir[1] + f[2] * dir[2]
            }
            _ => f64::NAN,
        };
        out.push(ImpactSample { t, force: f_on_soft, travel: x, speed: v_cm, leg, inverted: m.inverted_element_steps });
        if v_cm < 0.0 && f_on_soft.abs() < 1e-9 {
            let since = *clear_since.get_or_insert(t);
            if t - since > after {
                break;
            }
        } else {
            clear_since = None;
        }
        if t > t_end {
            break;
        }
    }
    Ok(out)
}

/// The spring-mass check: a 20 mm cube of a soft material (E 1 MPa, Poisson's ratio 0, so it
/// compresses one-dimensionally: k = E A / L), held at its bottom, struck on top by a flat
/// impactor of 0.5 kg at 0.2 m/s (its block's own mass is 8 g; its wave crosses it in 0.63 ms
/// against ~16 ms of contact). Returns (peak force, contact time, rebound speed) measured and
/// expected.
pub fn spring_mass_check() -> Result<([f64; 6], [f64; 6]), String> {
    let (l, e, rho) = (0.02, 1.0e6, 1000.0);
    let (m, v0) = (0.5, 0.2);
    let n = 8;
    let (pos, tets) = box_mesh([n, n, n], [l, l, l]);
    let held: Vec<bool> = pos.iter().map(|p| p[2] < 1e-9).collect();
    let base: Vec<usize> = (0..held.len()).filter(|&i| held[i]).collect();
    let mats = vec![material(e, 0.0, rho); tets.len()];
    let model = ExplicitModel::new(pos, tets, mats, held).map_err(|e| format!("model: {e:?}"))?;
    // The impactor: everything above z = 0 in its body frame (a half-space), baked over a slab
    // wider than the block.
    let ob = obstacle([-0.01, -0.01, -0.004], [0.03, 0.03, 0.004], 0.001, [0.0, 0.0, l], |p| -p[2])?;
    let exec = CpuExecutor::new(&model, &ob).map_err(|e| format!("executor: {e:?}"))?;
    let mut cfg = StepperConfig::new(0.0);
    let chunk = 2;
    cfg.monitor_every = chunk as u64;
    let mut st = Stepper::new(exec, cfg, 0.0);
    let k = e * l * l / l;
    let t_c = std::f64::consts::PI * (m / k).sqrt();
    let run = run_free(&mut st, Puck::rigid(m), [0.0, 0.0, l], [0.0, 0.0, -1.0], v0, 3.0 * t_c, chunk, 0.002, Some((&base, 1)))?;
    let peak = run.iter().map(|s| s.force).fold(0.0, f64::max);
    // The block's own stress waves ring on the spring's smooth force (amplitude ~ rho c v A,
    // 2.5 N here): averaged over one of its periods, 4 L / c, the ringing goes.
    let t_b = 4.0 * l / (e / rho).sqrt();
    let smooth = run
        .iter()
        .map(|s| {
            let w: Vec<f64> = run.iter().filter(|r| (r.t - s.t).abs() <= 0.5 * t_b).map(|r| r.force).collect();
            w.iter().sum::<f64>() / w.len().max(1) as f64
        })
        .fold(0.0, f64::max);
    let smooth_of = |get: &dyn Fn(&ImpactSample) -> f64| {
        run.iter()
            .map(|s| {
                let w: Vec<f64> = run.iter().filter(|r| (r.t - s.t).abs() <= 0.5 * t_b).map(get).collect();
                w.iter().sum::<f64>() / w.len().max(1) as f64
            })
            .fold(0.0, f64::max)
    };
    // The force on the block's held base, which must match the contact force once the ringing
    // is averaged out: the read of the force on the leg.
    let base_force = smooth_of(&|s: &ImpactSample| s.leg);
    let deepest = run.iter().map(|s| s.travel).fold(0.0, f64::max);
    let touching: Vec<&ImpactSample> = run.iter().filter(|s| s.force > 1e-6).collect();
    let contact = touching.last().map_or(0.0, |s| s.t) - touching.first().map_or(0.0, |s| s.t);
    let rebound = -run.last().map_or(0.0, |s| s.speed);
    let f = v0 * (k * m).sqrt();
    Ok(([peak, smooth, deepest, contact, rebound, base_force], [f, f, v0 * (m / k).sqrt(), t_c, v0, f]))
}

/// The bending check: a PA 12 strip (E 1.8 GPa, Poisson 0.38, 1010 kg/m3), 40 x 8 x 2 mm,
/// clamped at x = 0, every other node given 0.05 m/s in z, then free. Its tip's motion has the
/// first mode's period; Euler-Bernoulli's first frequency is (1.8751^2 / 2 pi L^2) sqrt(EI /
/// rho A). Returns (measured, expected) Hz with `through` elements through the thickness.
pub fn cantilever_check(through: usize) -> Result<(f64, f64), String> {
    let (l, w, t) = (0.04, 0.008, 0.002);
    let (e, nu, rho) = (1.8e9, 0.38, 1010.0);
    let h = t / through as f64;
    let n = [(l / h).round() as usize, (w / h).round() as usize, through];
    let (pos, tets) = box_mesh(n, [l, w, t]);
    let held: Vec<bool> = pos.iter().map(|p| p[0] < 1e-9).collect();
    let tip: Vec<usize> = (0..pos.len()).filter(|&i| (pos[i][0] - l).abs() < 1e-9).collect();
    let mats = vec![material(e, nu, rho); tets.len()];
    let model = ExplicitModel::new(pos.clone(), tets, mats, held.clone()).map_err(|e| format!("model: {e:?}"))?;
    // No contact: a small obstacle far away.
    let ob = obstacle([-0.002; 3], [0.002; 3], 0.001, [1.0, 1.0, 1.0], |p| (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt() - 0.001)?;
    let exec = CpuExecutor::new(&model, &ob).map_err(|e| format!("executor: {e:?}"))?;
    let mut st = Stepper::new(exec, StepperConfig::new(0.0), 0.0);
    let vel: Vec<[f64; 3]> = held.iter().map(|&h| if h { [0.0; 3] } else { [0.0, 0.0, 0.05] }).collect();
    st.executor_mut().set_state(0.0, &vec![[0.0; 3]; pos.len()], &vel, None);
    let i = w * t * t * t / 12.0;
    let f_expected = 1.8751_f64.powi(2) / (2.0 * std::f64::consts::PI * l * l) * (e * i / (rho * w * t)).sqrt();
    // Three and a half periods; the tip's mean z every few steps, and its upward zero crossings.
    let t_end = 3.5 / f_expected;
    let mut trace: Vec<(f64, f64)> = Vec::new();
    while st.time() < t_end {
        for _ in 0..5 {
            st.step().map_err(|e| format!("step: {e}"))?;
        }
        let snap = st.executor_mut().snapshot();
        let z = tip.iter().map(|&k| snap.displacements[k][2]).sum::<f64>() / tip.len() as f64;
        trace.push((st.time(), z));
    }
    // Upward crossings of zero, with a dead band of 30% of the largest swing so the higher modes'
    // ripples can't count as crossings (counting every local maximum did: 12-20x too high, and
    // worse with refinement, 2026-11-12). Interpolated between reads; a period apart.
    let amp = trace.iter().map(|s| s.1.abs()).fold(0.0, f64::max);
    let band = 0.3 * amp;
    let mut armed = false;
    let mut ups: Vec<f64> = Vec::new();
    for k in 1..trace.len() {
        let (a, b) = (trace[k - 1], trace[k]);
        if b.1 < -band {
            armed = true;
        }
        if armed && a.1 < 0.0 && b.1 >= 0.0 {
            ups.push(a.0 + (b.0 - a.0) * (-a.1) / (b.1 - a.1));
            armed = false;
        }
    }
    if ups.len() < 2 {
        return Err(format!("cantilever: {} upward crossings, need 2", ups.len()));
    }
    let period = (ups[ups.len() - 1] - ups[0]) / (ups.len() - 1) as f64;
    Ok((1.0 / period, f_expected))
}

/// `cargo run --release -- impact`: the checks above, printed with their timings.
pub fn run_checks() -> Result<(), String> {
    let t = std::time::Instant::now();
    let (got, want) = spring_mass_check()?;
    let pct = |k: usize| 100.0 * (got[k] / want[k] - 1.0);
    println!(
        "spring-mass impact (0.5 kg at 0.2 m/s on a 20 mm cube, k = E A / L), expected v sqrt(k m), v sqrt(m/k), pi sqrt(m/k): peak force {:.2} N raw ({:.2}, {:+.1}%), {:.2} N averaged over the block's wave period ({:+.1}%); deepest {:.3} mm ({:.3}, {:+.1}%); contact {:.2} ms ({:.2}, {:+.1}%); rebound {:.3} m/s ({:.3}, {:+.1}%) ({:.1} s)",
        got[0],
        want[0],
        pct(0),
        got[1],
        pct(1),
        got[2] * 1e3,
        want[2] * 1e3,
        pct(2),
        got[3] * 1e3,
        want[3] * 1e3,
        pct(3),
        got[4],
        want[4],
        pct(4),
        t.elapsed().as_secs_f64()
    );
    for through in [2, 3, 4] {
        let t = std::time::Instant::now();
        let (f, f0) = cantilever_check(through)?;
        println!(
            "PA 12 strip 40 x 8 x 2 mm, clamped, {through} elements through the thickness: first mode {f:.0} Hz (Euler-Bernoulli {f0:.0}, {:+.1}%, so the stiffness {:+.0}%) ({:.1} s)",
            100.0 * (f / f0 - 1.0),
            100.0 * ((f / f0).powi(2) - 1.0),
            t.elapsed().as_secs_f64()
        );
    }
    Ok(())
}

/// The force the body puts on its held nodes (its supports: the leg), N: the elastic and
/// viscous forces the material exerts on them, summed. A debug read of every node's forces
/// (`phase_outputs`), so taken every few chunks, not every step.
pub fn held_force(stepper: &mut Stepper<CpuExecutor>, held: &[usize]) -> [f64; 3] {
    let out = stepper.executor_mut().phase_outputs();
    let mut f = [0.0; 3];
    for &i in held {
        for k in 0..3 {
            f[k] += out.elastic_forces[i][k] + out.viscous_forces[i][k];
        }
    }
    f
}

/// A layer of a pad section, from the leg out: its thickness (m), its elements through, its
/// material.
#[derive(Clone, Copy, Debug)]
pub struct Layer {
    pub t: f64,
    pub cells: usize,
    pub mat: Material,
}

/// A section of a pad wrapped round a rigid shin: a cylindrical shell of `layers` from radius
/// `r_in` out, `half_angle` degrees either side of the front (+x) and `height` m tall, cells
/// about `cell` m round and up it. Positions, elements, a material per element, and the held
/// nodes (on the shin, r = r_in).
pub fn cyl_coupon(r_in: f64, layers: &[Layer], half_angle: f64, height: f64, cell: f64) -> (Vec<[f64; 3]>, Vec<[u32; 4]>, Vec<Material>, Vec<bool>) {
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
    let arc = 2.0 * half_angle.to_radians() * r_out;
    let nt = (arc / cell).round().max(2.0) as usize;
    let nz = (height / cell).round().max(2.0) as usize;
    let nr = radii.len() - 1;
    // A unit box mesh mapped onto the cylinder: x -> angle, y -> height, z -> radius.
    let (unit, tets) = box_mesh([nt, nz, nr], [1.0, 1.0, 1.0]);
    let pos: Vec<[f64; 3]> = unit
        .iter()
        .map(|u| {
            let th = (-half_angle + 2.0 * half_angle * u[0]).to_radians();
            let k = (u[2] * nr as f64).round() as usize;
            let r = radii[k.min(nr)];
            [r * th.cos(), r * th.sin(), (u[1] - 0.5) * height]
        })
        .collect();
    // Mapping can turn elements inside out (it mirrors): fix their order, and give each its
    // layer's material from its centre's radius.
    let mut fixed = Vec::with_capacity(tets.len());
    let mut mats = Vec::with_capacity(tets.len());
    for t in tets {
        let p = t.map(|i| pos[i as usize]);
        let e = |a: usize| [p[a][0] - p[0][0], p[a][1] - p[0][1], p[a][2] - p[0][2]];
        let (a, b, c) = (e(1), e(2), e(3));
        let vol = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
        fixed.push(if vol > 0.0 { t } else { [t[0], t[2], t[1], t[3]] });
        let rc = p.iter().map(|q| (q[0] * q[0] + q[1] * q[1]).sqrt()).sum::<f64>() / 4.0;
        let cell = radii.windows(2).position(|w| rc >= w[0] && rc <= w[1]).unwrap_or(0);
        mats.push(layers[layer_of_cell[cell.min(layer_of_cell.len() - 1)]].mat);
    }
    let held = pos.iter().map(|q| ((q[0] * q[0] + q[1] * q[1]).sqrt() - r_in).abs() < 1e-9).collect();
    (pos, fixed, mats, held)
}

/// A puck (radius `r`, half-thickness `h`, axis along z) as a rigid obstacle in its body frame,
/// its distance baked over the box it can reach the pad from (the face toward -x), at `at`.
pub fn puck_obstacle(r: f64, h: f64, cell: f64, at: [f64; 3]) -> Result<Obstacle, String> {
    let sdf = move |p: [f64; 3]| {
        let dr = (p[0] * p[0] + p[1] * p[1]).sqrt() - r;
        let dz = p[2].abs() - h;
        let (a, b) = (dr.max(0.0), dz.max(0.0));
        (a * a + b * b).sqrt() + dr.max(dz).min(0.0)
    };
    // The grid covers the puck's leading half and well past it: the pad stays within it.
    let lo = [-(r + 0.045), -0.07, -0.06];
    let hi = [0.0 + 0.003, 0.07, 0.06];
    let snap = |v: f64| (v / cell).round() * cell;
    obstacle(lo.map(snap), hi.map(snap), cell, at, sdf)
}

/// One puck on a pad section (`cyl_coupon`): a rigid puck of `mass` kg at `speed` m/s, rim
/// first, straight at the front of the shin. Returns the run and the section's size.
#[allow(clippy::too_many_arguments)]
pub fn pad_impact(r_in: f64, layers: &[Layer], half_angle: f64, height: f64, cell: f64, puck: Puck, speed: f64, t_end: f64) -> Result<(Vec<ImpactSample>, usize, usize, f64), String> {
    let (pos, tets, mats, held) = cyl_coupon(r_in, layers, half_angle, height, cell);
    let held_idx: Vec<usize> = (0..held.len()).filter(|&i| held[i]).collect();
    let r_out = r_in + layers.iter().map(|l| l.t).sum::<f64>();
    let (nodes, elems) = (pos.len(), tets.len());
    let model = ExplicitModel::new(pos, tets, mats, held).map_err(|e| format!("model: {e:?}"))?;
    let (pr, ph) = (0.0381, 0.0127);
    // The puck's centre starts just clear of the pad's front, moving toward -x.
    let at = [r_out + pr + 0.0002, 0.0, 0.0];
    let ob = puck_obstacle(pr, ph, 0.0005, at)?;
    let exec = CpuExecutor::new(&model, &ob).map_err(|e| format!("executor: {e:?}"))?;
    let chunk = 2;
    let mut cfg = StepperConfig::new(0.0);
    cfg.monitor_every = chunk as u64;
    let mut st = Stepper::new(exec, cfg, 0.0);
    let dt0 = st.dt();
    let run = run_free(&mut st, puck, at, [-1.0, 0.0, 0.0], speed, t_end, chunk, 0.0005, Some((&held_idx, 5)))?;
    Ok((run, nodes, elems, dt0))
}

/// A puck as two masses: its face (the rubber that meets the target; the obstacle) and the rest
/// of it, joined by the rubber between, a Hunt-Crossley spring: force k d (1 + alpha d'),
/// compression only, no jump at first touch. Linear in d (n = 1): Anderson 2008 found a rim
/// impact's contact time the same from 24.6 to 38 m/s, which a linear spring gives.
#[derive(Clone, Copy, Debug)]
pub struct Puck {
    pub mass: f64,
    /// The face's share of the mass; 1 is a rigid puck.
    pub face: f64,
    pub k: f64,
    pub alpha: f64,
}

impl Puck {
    pub fn rigid(mass: f64) -> Self {
        Puck { mass, face: 1.0, k: 0.0, alpha: 0.0 }
    }

    pub fn spring(&self, d: f64, dd: f64) -> f64 {
        if d <= 0.0 { 0.0 } else { (self.k * d * (1.0 + self.alpha * dd)).max(0.0) }
    }

    /// Rim first into a rigid wall at `v`: the face stops at the wall, the rest compresses the
    /// rubber and comes back. (peak force, contact time, restitution of the whole puck: the
    /// face's momentum is lost to the wall). RK4, 4000 steps over a generous window.
    pub fn on_wall(&self, v: f64) -> (f64, f64, f64) {
        let mb = self.mass * (1.0 - self.face);
        let (mut d, mut dd) = (0.0f64, v);
        let w = (self.k / mb).sqrt();
        let h = 4.0 * std::f64::consts::PI / w / 4000.0;
        let acc = |d: f64, dd: f64| -self.spring(d, dd) / mb;
        let (mut t, mut peak) = (0.0, 0.0f64);
        loop {
            let (k1d, k1v) = (dd, acc(d, dd));
            let (k2d, k2v) = (dd + 0.5 * h * k1v, acc(d + 0.5 * h * k1d, dd + 0.5 * h * k1v));
            let (k3d, k3v) = (dd + 0.5 * h * k2v, acc(d + 0.5 * h * k2d, dd + 0.5 * h * k2v));
            let (k4d, k4v) = (dd + h * k3v, acc(d + h * k3d, dd + h * k3v));
            d += h / 6.0 * (k1d + 2.0 * k2d + 2.0 * k3d + k4d);
            dd += h / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
            t += h;
            peak = peak.max(self.spring(d, dd));
            if (dd < 0.0 && self.spring(d, dd) <= 0.0) || t > 1.0 {
                break;
            }
        }
        (peak, t, -dd * mb / (self.mass * v))
    }

    /// Fitted to a measured rigid-wall rim impact: speed, contact time, restitution. The
    /// restitution fixes alpha v (bisection), the contact time then k.
    pub fn calibrated(mass: f64, face: f64, v: f64, t_contact: f64, e: f64) -> Self {
        let mb = mass * (1.0 - face);
        let unit = |alpha: f64| Puck { mass, face, k: mb * 1.0e6, alpha };
        let (mut lo, mut hi) = (0.0f64, 50.0 / v);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if unit(mid).on_wall(v).2 > e { lo = mid } else { hi = mid }
        }
        let alpha = 0.5 * (lo + hi);
        // Contact time scales as 1/sqrt(k) at fixed alpha v.
        let t1 = unit(alpha).on_wall(v).1;
        Puck { mass, face, k: mb * 1.0e6 * (t1 / t_contact).powi(2), alpha }
    }
}

/// The ice puck, fitted to Anderson 2008 (WSU MS thesis; brand "PA", 164 g, rim first at 24.6
/// m/s into a rigid plate): at 22 C contact 0.81 ms, restitution 0.383; at -4 C 0.41 ms, 0.267.
/// The face is 15% of the mass (an assumption: the rubber near the rim). The measured peak forces,
/// 14.2 and 37 kN, are left out of the fit to check it.
pub fn ice_puck(cold: bool) -> Puck {
    if cold { Puck::calibrated(0.164, 0.15, 24.6, 0.41e-3, 0.267) } else { Puck::calibrated(0.164, 0.15, 24.6, 0.81e-3, 0.383) }
}
