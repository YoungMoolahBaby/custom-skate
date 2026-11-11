//! `round1 <scan> [out]`: the whole skate on one foot, end to end, round-1 quality.
//! Foot -> last -> boot shell + liner -> fit map -> roller chassis (rocker, lower frame,
//! coupling, axles) -> ice holder placeholder; meshed, weighed, checked for clashes, written as
//! STL, rendered, summarised in `summary.txt`.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::time::Instant;

use cortenforge::cf_design::Solid;
use cortenforge::cf_design::mechanism::mass::mass_properties;
use cortenforge::mesh::io::save_mesh;
use cortenforge::mesh::repair::weld_vertices;
use cortenforge::mesh::types::IndexedMesh;

use crate::boot::{self, BootParams};
use crate::chassis::{self, ChassisParams};
use crate::materials::{AL_7075_T6, CFRP, LINER_FOAM, Material, PA12_MJF, PU_BUSHING, TI_6AL_4V};
use crate::render::{Layer, diverging, render, standard_views};
use crate::{fit, foot, scan};

pub fn mesh_of(solid: &Solid, cell: f64) -> IndexedMesh {
    let mut m = solid.mesh(cell).geometry;
    weld_vertices(&mut m, 1e-5);
    m
}

fn grams(solid: &Solid, mat: &Material, cell: f64) -> f64 {
    mass_properties(solid, mat.density, cell).map_or(0.0, |m| m.mass * 1000.0)
}

pub fn run(scan_path: &Path, out: &Path) -> Result<(), String> {
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let t0 = Instant::now();
    let (raw, _, _) = scan::load(scan_path)?;
    let m = scan::measure(&raw, 20);
    println!("foot: {} ({:.1} x {:.1} mm, up from {})", scan_path.display(), m.length, m.width, m.up_from);
    // The grid must reach past the largest offset taken from it (the closing): mesh-sdf clamps
    // queries outside the grid to the edge value, silently (docs/CORTENFORGE.md).
    let foot = foot::build(&raw, &m, 1.5, 24.0)?;
    println!("  standard frame: length {:.1}, width {:.1}, height {:.1} mm", foot.length, foot.width, foot.height);

    // The base of the last (foot::last_base): the foot closed by 12 mm + a convex toe box.
    let lb = foot::last_base(&foot, 3.0 * 1.5 + 2.0)?;
    let base = lb.solid;
    if std::env::var_os("ROUND1_DEBUG").is_some() {
        let base_mesh = mesh_of(&base, 1.5);
        render(&[Layer::solid(&base_mesh, [0.6, 0.6, 0.6])], &standard_views(), 1.5, &out.join("debug_base.png"))
            .map_err(|e| e.to_string())?;
    }

    let p = BootParams::default();
    let b = boot::build(base, lb.bounds, foot.length, p);
    println!(
        "  last base: foot closed by {} mm + convex toe box from {:.0}% of length, faded in over {} mm, blended {} mm",
        foot::CLOSING,
        lb.toe_from / foot.length * 100.0,
        foot::FADE_LEN,
        foot::BLEND
    );
    println!(
        "boot (round 1): ease {} + liner {} + wall {} mm, trim at {} mm, sole plate {} mm, sole bottom z {:.1}",
        p.ease, p.liner, p.wall, p.top, p.sole, b.sole_bottom
    );

    let t = Instant::now();
    let last = mesh_of(&b.last.clone().intersect(Solid::plane(nalgebra::Vector3::z(), p.top)), 1.0);
    let shell = mesh_of(&b.shell, 1.0);
    let liner = mesh_of(&b.liner, 1.0);
    println!(
        "  meshed: last {} / shell {} / liner {} triangles ({:.1} s)",
        last.faces.len(),
        shell.faces.len(),
        liner.faces.len(),
        t.elapsed().as_secs_f64()
    );

    let t = Instant::now();
    let shell_pa = grams(&b.shell, &PA12_MJF, 0.5);
    let shell_cf = shell_pa * CFRP.density / PA12_MJF.density;
    let liner_g = grams(&b.liner, &LINER_FOAM, 0.75);
    println!(
        "  mass: shell {shell_pa:.0} g in {} / {shell_cf:.0} g in {}; liner {liner_g:.0} g ({}) ({:.1} s)",
        PA12_MJF.name,
        CFRP.name,
        LINER_FOAM.name,
        t.elapsed().as_secs_f64()
    );

    let save = |name: &str, mesh: &IndexedMesh| -> Result<PathBuf, String> {
        let path = out.join(name);
        save_mesh(mesh, &path).map_err(|e| format!("{}: {e}", path.display()))?;
        Ok(path)
    };
    save("foot.stl", &foot.mesh)?;
    save("last.stl", &last)?;
    save("boot_shell.stl", &shell)?;
    save("liner.stl", &liner)?;

    let skin = [0.86, 0.68, 0.58];
    let shell_col = [0.25, 0.32, 0.42];
    let views = standard_views();
    render(&[Layer::solid(&shell, shell_col)], &views, 2.0, &out.join("boot_shell.png"))
        .map_err(|e| e.to_string())?;
    render(
        &[Layer::solid(&shell, shell_col), Layer::solid(&foot.mesh, skin)],
        &views,
        2.0,
        &out.join("boot_on_foot.png"),
    )
    .map_err(|e| e.to_string())?;

    let mut summary = String::new();
    let _ = writeln!(summary, "round 1, {}", scan_path.display());
    let _ = writeln!(summary, "foot {:.1} x {:.1} mm (length x width), up from {}", foot.length, foot.width, m.up_from);
    let _ = writeln!(
        summary,
        "boot shell {shell_pa:.0} g in {} or {shell_cf:.0} g in {}; liner {liner_g:.0} g (foam density assumed)",
        PA12_MJF.name, CFRP.name
    );

    // Fit: the real foot against the last.
    let gaps = fit::gaps(&foot.mesh, &b.last, p.top);
    let regions = fit::regions(&foot.mesh, &gaps, foot.length);
    println!("fit (gap from the foot to the last, mm; round 1's last never cuts in):");
    let _ = writeln!(summary, "fit, gap from the foot to the last (mm): region, n, min / p10 / median / p90");
    for r in &regions {
        let line = format!(
            "  {:<26} {:>5}  {:5.1} / {:5.1} / {:5.1} / {:5.1}",
            r.name, r.n, r.min, r.p10, r.median, r.p90
        );
        println!("{line}");
        let _ = writeln!(summary, "{line}");
    }
    let colors: Vec<[f64; 3]> =
        gaps.iter().map(|g| g.map_or([0.75, 0.75, 0.75], |g| diverging((g - p.ease) / 10.0))).collect();
    render(
        &[Layer { mesh: &foot.mesh, color: skin, vertex_colors: Some(&colors) }],
        &views,
        2.0,
        &out.join("fit_map.png"),
    )
    .map_err(|e| e.to_string())?;

    // Roller: chassis under the boot.
    let cp = ChassisParams::default();
    let c = chassis::build(b.sole_bottom, foot.length, cp);
    let t = Instant::now();
    let parts: [(&str, &Solid, &Material); 5] = [
        ("rocker (upper)", &c.rocker, &AL_7075_T6),
        ("lower frame", &c.lower, &AL_7075_T6),
        ("axles x4", &c.axles, &AL_7075_T6),
        ("bolt", &c.bolt, &TI_6AL_4V),
        ("bushing", &c.bushing, &PU_BUSHING),
    ];
    println!(
        "roller chassis (round-1 layout): 4 x {} mm wheels, pitch {} mm, rocker radius {:.0} mm ({:.1} ft), stack: ground {:.1} mm below the boot sole",
        cp.wheel_d,
        cp.wheel_d + cp.wheel_gap,
        cp.rocker_r,
        cp.rocker_r / 304.8,
        b.sole_bottom - c.ground_z
    );
    let _ = writeln!(summary, "roller chassis, round-1 layout (weights at 0.25 mm cells):");
    let mut total = 0.0;
    for (name, solid, mat) in parts {
        let g = grams(solid, mat, 0.25);
        total += g;
        let line = format!("  {name:<16} {g:6.1} g  ({})", mat.name);
        println!("{line}");
        let _ = writeln!(summary, "{line}");
    }
    let line = format!(
        "  total {total:.1} g (one-piece 7075 frames: 170-184 g with axles and spacers; Marsblade R1 282-302 g) ({:.1} s)",
        t.elapsed().as_secs_f64()
    );
    println!("{line}");
    let _ = writeln!(summary, "{line}");
    // Clashes: wheels against the frame, the bushing and the bolt. (The rocker rests on the bridge
    // by design, a line contact, so it is not in this list.)
    for (name, a, bb) in [
        ("wheels x lower frame", &c.wheels, &c.lower),
        ("wheels x bushing", &c.wheels, &c.bushing),
        ("wheels x bolt", &c.wheels, &c.bolt),
    ] {
        let v = mass_properties(&a.clone().intersect(bb.clone()), 1000.0, 0.25).map_or(0.0, |m| m.mass * 1e6);
        let line = format!("  clash {name}: {v:.1} mm3");
        println!("{line}");
        let _ = writeln!(summary, "{line}");
    }

    let rocker = mesh_of(&c.rocker, 0.5);
    let lower = mesh_of(&c.lower, 0.5);
    let axles = mesh_of(&c.axles, 0.5);
    let bolt = mesh_of(&c.bolt.clone().union(c.bushing.clone()), 0.5);
    let wheels = mesh_of(&c.wheels, 1.0);
    save("rocker.stl", &rocker)?;
    save("lower_frame.stl", &lower)?;
    save("axles.stl", &axles)?;
    save("coupling.stl", &bolt)?;
    let alu = [0.72, 0.74, 0.78];
    let wheel_col = [0.95, 0.75, 0.15];
    render(
        &[
            Layer::solid(&shell, shell_col),
            Layer::solid(&rocker, [0.55, 0.6, 0.68]),
            Layer::solid(&lower, alu),
            Layer::solid(&axles, [0.4, 0.4, 0.4]),
            Layer::solid(&bolt, [0.85, 0.3, 0.25]),
            Layer::solid(&wheels, wheel_col),
        ],
        &views,
        2.0,
        &out.join("roller_skate.png"),
    )
    .map_err(|e| e.to_string())?;
    render(
        &[
            Layer::solid(&rocker, [0.55, 0.6, 0.68]),
            Layer::solid(&lower, alu),
            Layer::solid(&axles, [0.4, 0.4, 0.4]),
            Layer::solid(&bolt, [0.85, 0.3, 0.25]),
        ],
        &views,
        3.0,
        &out.join("chassis.png"),
    )
    .map_err(|e| e.to_string())?;

    // Ice: the same boot on a holder placeholder.
    let h = chassis::holder(b.sole_bottom, foot.length, 254.0);
    let holder = mesh_of(&h.holder, 0.75);
    let runner = mesh_of(&h.runner, 0.5);
    save("ice_holder_placeholder.stl", &holder)?;
    render(
        &[
            Layer::solid(&shell, shell_col),
            Layer::solid(&holder, [0.92, 0.92, 0.92]),
            Layer::solid(&runner, [0.55, 0.57, 0.6]),
        ],
        &views,
        2.0,
        &out.join("ice_skate.png"),
    )
    .map_err(|e| e.to_string())?;
    let _ = writeln!(
        summary,
        "ice skate: the same boot on a 254 mm holder PLACEHOLDER (the i2's mounting pattern and weight not measured yet)"
    );

    std::fs::write(out.join("summary.txt"), &summary).map_err(|e| e.to_string())?;
    println!("  wrote STL + PNG + summary.txt to {} ({:.1} s total)", out.display(), t0.elapsed().as_secs_f64());
    Ok(())
}
