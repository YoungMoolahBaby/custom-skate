//! Render what goes under the boot (the ice holder, the roller chassis) on its own, quickly:
//! `cargo run --release --example view_under -- out.png [ice|roller]`. The sole height and the
//! mount pads are the stand-in boot's (sole 10.5 mm below the foot, pads 165 mm apart).
use custom_skate::render::{render_styled, Layer, Style, View};
use custom_skate::round1::mesh_of;
use custom_skate::skate;
use nalgebra::Vector3;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let length = custom_skate::round2::STAND_IN_LENGTH;
    let xc = 0.47 * length;
    let mounts = [(xc - 82.5, 4.0), (xc + 82.5, 4.0)];
    let under = if args.get(1).map(String::as_str) == Some("roller") {
        skate::roller(-10.5, mounts, length)
    } else {
        skate::ice(-10.5, mounts, skate::HolderParams::default())
    };
    println!("{}", under.note);
    let meshes: Vec<_> = under.parts.iter().map(|u| mesh_of(&u.solid, u.cell)).collect();
    for (u, m) in under.parts.iter().zip(&meshes) {
        println!("  {:<15} {} triangles, {:.1} g", u.name, m.faces.len(), custom_skate::hockey_boot::mesh_volume(m) * u.material.density * 1e-6);
    }
    let layers: Vec<Layer> = under.parts.iter().zip(&meshes).map(|(u, m)| Layer::solid(m, u.alt).with_spec(0.4)).collect();
    let views = [
        View::looking(Vector3::new(0.0, 1.0, 0.0), Vector3::z()),
        View::looking(Vector3::new(-1.0, 0.0, -0.1), Vector3::z()),
        View::looking(Vector3::new(-0.6, 0.6, -0.45), Vector3::z()),
        View::looking(Vector3::new(0.0, 0.0, 1.0), Vector3::x()),
    ];
    render_styled(&layers, &views, 2.5, std::path::Path::new(&args[0]), Style::pretty()).expect("render");
}
