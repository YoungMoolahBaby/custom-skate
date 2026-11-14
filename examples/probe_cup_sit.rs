//! How round 3's hinged knee cup (G3: the shield split by a flex hinge 45 mm above the knee
//! joint, cut 90 deg round) sits on the body with its kneecap, with its hinges from free to firm:
//! per pose and hinge stiffness, how far the skin presses into each piece's liner, and how each
//! hinge's tied middle turns and gives (opens, closes, lifts, slides sideways).
//! `cargo run --release --example probe_cup_sit -- scans/stand-in/022_fBW_R.ply`

use custom_skate::pad::{self, Outlines};
use custom_skate::{body, cup};

fn main() {
    let path = std::env::args().nth(1).expect("scan path");
    let b = body::build(std::path::Path::new(&path), 1250.0).unwrap();
    let ll = pad::leaned_leg(std::path::Path::new(&path)).unwrap();
    // Round 3's cup (`cup::cup_params` is round 4's sliding shells).
    let p = custom_skate::pad::PadParams { knee_hinge: Some(45.0), knee_wrap: Some(cup::CUP_WRAP), ..Default::default() };
    let (pm, _, mv) = cup::posed(&b, cup::build_pose()).unwrap();
    let fs = cup::front_of_shin(&b);
    let hd = cup::hold(&b, &ll, &Outlines::new(ll.knee.z), &p, &pm, &cup::normals(&pm, fs));
    let shin = b.bone("lowerleg01.R").unwrap();
    println!("tied: lower {} points, upper {}", hd.lower.len(), hd.upper.len());
    for knee in [0.0, 30.0, 90.0, 135.0] {
        let q = custom_skate::pose::LegPose { hip: cup::HIP_BUILD, knee, ankle: 0.0 };
        let (qm, _, qv) = cup::posed(&b, q).unwrap();
        let n = cup::normals(&qm, fs);
        let shell = qv[shin].after(&mv[shin].inverse());
        println!("knee {knee}:");
        for hold in [0.0, 1.0, 5.0, 20.0, 100.0, 500.0, 5000.0] {
            let ms = cup::place_with(&hd, &pm, &qm, &n, &shell, [hold; 2]);
            let (ci, co, _) = cup::sit(&hd.anchors[0], &pm, &qm, &ms[0], &n);
            let (ti, to, _) = cup::sit(&hd.anchors[1], &pm, &qm, &ms[1], &n);
            let (lt, _) = cup::hinge(&hd.lower, &ms[0], &shell);
            let (ut, _) = cup::hinge(&hd.upper, &ms[1], &ms[0]);
            let lg = cup::give(&hd.lower, &hd.lower_dirs, &ms[0], &shell);
            let ug = cup::give(&hd.upper, &hd.upper_dirs, &ms[1], &ms[0]);
            println!(
                "  hold {hold:>6}: cap {ci:4.1} in /{co:5.1} off, top {ti:4.1} in /{to:5.1} off | lower {lt:4.1} deg, open {:4.1} close {:4.1} lift {:4.1} side {:4.1} | upper {ut:4.1} deg, open {:4.1} close {:4.1} lift {:4.1} side {:4.1}",
                lg.open, lg.close, lg.lift, lg.side, ug.open, ug.close, ug.lift, ug.side
            );
        }
    }
}
