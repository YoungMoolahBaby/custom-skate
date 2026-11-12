//! Render STL files together from five directions, for looking at parts without a full run.
//! `cargo run --release --example view_stl -- out.png part.stl[:r,g,b] ...`
use custom_skate::render::{render_styled, Layer, Style, View};
use nalgebra::Vector3;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let out = &args[0];
    let mut meshes = Vec::new();
    for a in &args[1..] {
        let (path, col) = match a.split_once(':') {
            Some((p, c)) => {
                let v: Vec<f64> = c.split(',').filter_map(|x| x.parse().ok()).collect();
                (p, [v[0], v[1], v[2]])
            }
            None => (a.as_str(), [0.6, 0.6, 0.62]),
        };
        let m = cortenforge::mesh::io::load_stl(path).expect("load stl");
        meshes.push((m, col));
    }
    let layers: Vec<Layer> = meshes.iter().map(|(m, c)| Layer::solid(m, *c).with_spec(0.3)).collect();
    let views = [
        View::looking(Vector3::new(-1.0, 0.0, -0.15), Vector3::z()),
        View::looking(Vector3::new(0.0, -1.0, -0.1), Vector3::z()),
        View::looking(Vector3::new(0.0, 0.0, -1.0), Vector3::x()),
        View::looking(Vector3::new(1.0, 0.0, -0.3), Vector3::z()),
        View::looking(Vector3::new(-0.6, 0.6, -0.5), Vector3::z()),
    ];
    render_styled(&layers, &views, 2.0, std::path::Path::new(out), Style::pretty()).expect("render");
}
