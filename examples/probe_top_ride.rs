//! How far round 3's cup top (with the thigh guard, its top 115 mm above the knee joint) rides up
//! the thigh as the knee straightens, hinged to the cap or resting on the thigh by itself: the
//! skin under the guard's top edge at the front, standing height, against the skin it sits over
//! once placed in a pose. What led to round 4's sliding shells.
//! `cargo run --release --example probe_top_ride -- scans/stand-in/022_fBW_R.ply`

use custom_skate::pad::{self, Outlines};
use custom_skate::pose::LegPose;
use custom_skate::{body, cup};

fn main() {
    let path = std::env::args().nth(1).expect("scan path");
    let b = body::build(std::path::Path::new(&path), 1250.0).unwrap();
    let ll = pad::leaned_leg(std::path::Path::new(&path)).unwrap();
    let kz = ll.knee.z;
    // Round 3's cup (`cup::cup_params` is round 4's sliding shells).
    let p = custom_skate::pad::PadParams { knee_hinge: Some(45.0), knee_wrap: Some(cup::CUP_WRAP), ..Default::default() };
    let (pm, _, mv) = cup::posed(&b, cup::build_pose()).unwrap();
    let fs = cup::front_of_shin(&b);
    let o = Outlines::new(kz);
    let hd = cup::hold(&b, &ll, &o, &p, &pm, &cup::normals(&pm, fs));
    let shin = b.bone("lowerleg01.R").unwrap();
    let mid = b.shift.1;
    // Standing (s, z) of every right-leg skin vertex.
    let sz: Vec<Option<(f64, f64)>> = b
        .mesh
        .vertices
        .iter()
        .map(|v| {
            (v.y < mid).then(|| {
                let q = ll.lean(*v);
                (ll.f.theta(q).to_radians() * pad::R_REF, q.z)
            })
        })
        .collect();
    let top = o.thigh.z_max;
    let edge: Vec<usize> = (0..sz.len()).filter(|&i| sz[i].is_some_and(|(s, z)| s.abs() < 20.0 && (z - top).abs() < 2.0)).collect();
    println!("the guard's top edge at {:.0} mm above the knee joint (standing), {} skin points under its middle", top - kz, edge.len());
    for (label, hold) in [("upper hinge firm", [cup::HINGE_HOLD; 2]), ("top free on the thigh", [cup::HINGE_HOLD, 0.0])] {
    println!("{label}:");
    for hip in [-20.0, 0.0, 60.0, 110.0] {
        let mut row = format!("hip {hip:>4}:");
        for knee in [0.0, 15.0, 30.0, 45.0, 70.0, 90.0, 135.0] {
            let q = LegPose { hip, knee, ankle: 0.0 };
            let (qm, _, qv) = cup::posed(&b, q).unwrap();
            let n = cup::normals(&qm, fs);
            let ms = cup::place_with(&hd, &pm, &qm, &n, &qv[shin].after(&mv[shin].inverse()), hold);
            let ug = cup::give(&hd.upper, &hd.upper_dirs, &ms[1], &ms[0]);
            let mut rides = Vec::new();
            for &i in &edge {
                let y = ms[1].apply(pm.vertices[i].coords);
                let j = (0..qm.vertices.len())
                    .filter(|&j| sz[j].is_some())
                    .min_by(|&a, &c| (qm.vertices[a].coords - y).norm_squared().total_cmp(&(qm.vertices[c].coords - y).norm_squared()))
                    .unwrap();
                rides.push(sz[j].unwrap().1 - sz[i].unwrap().1);
            }
            let mean = rides.iter().sum::<f64>() / rides.len() as f64;
            row += &format!("  knee {knee:>3}: {mean:+5.1} (cap {:+5.1})", if ug.close > ug.open { ug.close } else { -ug.open });
        }
        println!("{row}  (mm up the thigh, mean over the edge's middle)");
    }
    }
}
