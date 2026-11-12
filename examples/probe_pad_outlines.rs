//! Where the pad's outlines meet, on the leg laid flat: for each position round the leg, the
//! front shell's and the side panel's tops and the knee shield's bottom (mm above the floor;
//! the knee joint at 466), and how far each tucks under the shield (negative: a gap, skin shows).
//! `cargo run --release --example probe_pad_outlines`

use custom_skate::pad::Outlines;

fn main() {
    let kz = 466.0;
    let o = Outlines::new(kz);
    let top = |f: &dyn Fn(f64) -> f64| (0..5000).map(|k| 100.0 + 0.1 * k as f64).filter(|&z| f(z) < 0.0).fold(f64::NAN, f64::max);
    println!("  s (mm of arc)  deg   front top   side top   shield bottom   front under   side under");
    for s in [0.0, 12.0, 24.0, 32.0, 40.0, 50.0, 60.0, 66.0, 72.0, 80.0, 86.0] {
        let deg = s / 45.0 * 180.0 / std::f64::consts::PI;
        let ft = top(&|z| o.front.eval(s, z));
        let st = top(&|z| o.side.eval(s, z));
        let bot = (0..4000).map(|k| 300.0 + 0.1 * k as f64).filter(|&z| o.knee.eval(s, z) < 0.0).fold(f64::NAN, f64::min);
        println!("  {s:>6.0} {deg:>12.0} {ft:>11.1} {st:>10.1} {bot:>15.1} {:>13.1} {:>12.1}", ft - bot, st - bot);
    }
}
