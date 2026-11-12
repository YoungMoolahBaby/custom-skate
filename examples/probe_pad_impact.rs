//! A puck on a pad section (`impact::pad_impact`): the cold ice puck (fitted to Anderson 2008) at
//! 23 m/s, a recreational slap shot, rim first at the front of a rigid shin, through 8 mm of foam
//! (10 MPa, mid-range of an EVA blend's modulus at impact rates, Nicotra 2014) under a 2 mm PA 12
//! shell. `cargo run --release --example probe_pad_impact [foam MPa] [shell mm, 0 for none] [speed m/s] [cold|warm] [foam mm]`

use custom_skate::impact::{Layer, ice_puck, material, pad_impact};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let arg = |i: usize, d: f64| a.get(i).and_then(|s| s.parse().ok()).unwrap_or(d);
    let (foam_e, shell, speed) = (arg(1, 10.0) * 1e6, arg(2, 2.0) * 1e-3, arg(3, 23.0));
    let foam_t = arg(5, 8.0) * 1e-3;
    let cold = a.get(4).map_or(true, |s| s != "warm");
    let t = std::time::Instant::now();
    let foam = material(foam_e, 0.15, 150.0);
    let pa12 = material(1.8e9, 0.38, 1010.0);
    // Foam in ~2 mm elements; the shell 4 through (the bending check); shell 0 leaves it off.
    let mut layers = vec![Layer { t: foam_t, cells: ((foam_t / 0.002).round() as usize).max(2), mat: foam }];
    if shell > 0.0 {
        layers.push(Layer { t: shell, cells: 4, mat: pa12 });
    }
    let puck = ice_puck(cold);
    println!(
        "puck {} (k {:.2} MN/m, alpha {:.3} s/m, face 15%), {speed} m/s; foam {:.0} MPa {:.0} mm; shell {:.1} mm PA 12",
        if cold { "cold, -4 C" } else { "warm, 22 C" },
        puck.k / 1e6,
        puck.alpha,
        foam_e / 1e6,
        foam_t * 1e3,
        shell * 1e3
    );
    match pad_impact(0.045, &layers, 60.0, 0.10, 0.001, puck, speed, 0.003) {
        Ok((run, nodes, elems, dt)) => {
            let peak = run.iter().map(|s| s.force).fold(0.0, f64::max);
            let leg = run.iter().filter(|s| s.leg.is_finite()).map(|s| s.leg).fold(0.0, f64::max);
            let deepest = run.iter().map(|s| s.travel).fold(0.0, f64::max);
            println!("{nodes} nodes, {elems} elements, first step {:.3} us", dt * 1e6);
            println!(
                "peak contact force {:.2} kN, peak force on the leg {:.2} kN, face's deepest travel {:.1} mm, final speed {:.1} m/s, inverted element-steps {}",
                peak / 1e3,
                leg / 1e3,
                deepest * 1e3,
                run.last().map_or(0.0, |s| s.speed),
                run.last().map_or(0, |s| s.inverted)
            );
            for s in run.iter().step_by((run.len() / 30).max(1)) {
                println!("  t {:6.3} ms  contact {:8.0} N  leg {:8.0} N  travel {:5.2} mm  speed {:6.2} m/s  inverted {}", s.t * 1e3, s.force, s.leg, s.travel * 1e3, s.speed, s.inverted);
            }
            println!("{:.0} s", t.elapsed().as_secs_f64());
        }
        Err(e) => eprintln!("{e}"),
    }
}
