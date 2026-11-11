//! One parameter at a time from the round-2 defaults, then combinations: grams (0.5 mm cells, reads
//! ~0.5% heavy on thin walls) against the worst stress / allowable of the vertical cases and the
//! roll ratio. `cargo run --release --example sweep_frame`
use cortenforge::cf_design::mechanism::mass::mass_properties;
use custom_skate::frame::{self, FrameParams};
use custom_skate::materials::{AL_6061_T6, AL_7075_T6, PU_BUSHING, TI_6AL_4V};
use custom_skate::round2::{STAND_IN_LENGTH, checks};

fn g(s: &cortenforge::cf_design::Solid, rho: f64) -> f64 {
    mass_properties(s, rho, 0.5).map_or(0.0, |m| m.mass * 1000.0)
}

fn main() {
    let d = FrameParams::default();
    // The first sizing (2.5 mm rails and walls, 5 mm members, 2 mm web): 221 g.
    let first = FrameParams {
        rail_t: 2.5,
        top_h: 6.0,
        diag_w: 5.0,
        post_w: 5.0,
        web_t: 2.0,
        wall_t: 2.5,
        wall_diag_w: 5.0,
        wall_panels: 6,
        ..d
    };
    let variants: Vec<(&str, FrameParams)> = vec![
        ("first sizing (2.5 mm)", first),
        ("round 2 (defaults)", d),
        ("rail_t 1.8", FrameParams { rail_t: 1.8, ..d }),
        ("wall_t 1.8", FrameParams { wall_t: 1.8, ..d }),
        ("wall_panels 3", FrameParams { wall_panels: 3, ..d }),
        ("wall_bot_h 8", FrameParams { wall_bot_h: 8.0, ..d }),
        ("top_h 0 (no top chord)", FrameParams { top_h: 0.0, ..d }),
        ("no ties", FrameParams { tie_d: 0.0, ..d }),
        ("ledge_len 120", FrameParams { ledge_len: 120.0, ..d }),
        ("upper_len 160", FrameParams { upper_len: 160.0, ..d }),
        ("web_clear 5 (lower stack)", FrameParams { web_clear: 5.0, ..d }),
    ];
    println!("{:<24} {:>7} {:>7} {:>6} {:>7} {:>7}  {:>6} {:>5}", "variant", "upper", "lower", "ties", "rest", "total", "worst", "roll");
    for (name, p) in variants {
        let f = frame::build(0.0, STAND_IN_LENGTH, p);
        let up = g(&f.upper, AL_7075_T6.density);
        let lo = g(&f.lower, AL_7075_T6.density);
        let ti = f.ties.as_ref().map_or(0.0, |t| g(t, AL_7075_T6.density));
        let rest = g(&f.axles, AL_7075_T6.density)
            + g(&f.spacers, AL_6061_T6.density)
            + g(&f.bolt, TI_6AL_4V.density)
            + g(&f.nut, TI_6AL_4V.density)
            + g(&f.bushing, PU_BUSHING.density);
        let c = checks(&f, STAND_IN_LENGTH).expect("checks");
        println!(
            "{name:<24} {up:7.1} {lo:7.1} {ti:6.1} {rest:7.1} {:7.1}  {:6.2} {:5.2}   (upper {:.0}, lower {:.0}, foot {:.0}, chord {:.0} MPa)",
            up + lo + ti + rest,
            c.worst_ratio(),
            c.roll.0,
            c.upper.0,
            c.lower.0,
            c.foot.0,
            c.chord.0 + c.chord.1
        );
    }
}
