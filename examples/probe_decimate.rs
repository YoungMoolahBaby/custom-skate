//! Probe: where does cluster_decimate leave vertices off the surface?
use cortenforge::cf_design::Solid;
use custom_skate::hockey_boot::cluster_decimate;
fn main() {
    let shell = Solid::sphere(20.0).subtract(Solid::sphere(18.0));
    let fine = custom_skate::round1::mesh_of(&shell, 0.4);
    let raw = cluster_decimate(&fine, 2.5, None);
    let proj = cluster_decimate(&fine, 2.5, Some(&|q| shell.evaluate(&q)));
    let worst = |m: &cortenforge::mesh::types::IndexedMesh| {
        let mut v: Vec<(f64, nalgebra::Point3<f64>)> = m.vertices.iter().map(|p| (shell.evaluate(p), *p)).collect();
        v.sort_by(|a, b| b.0.abs().total_cmp(&a.0.abs()));
        v.truncate(5);
        v
    };
    let used = |m: &cortenforge::mesh::types::IndexedMesh| {
        let mut u = vec![false; m.vertices.len()];
        for f in &m.faces { for i in f { u[*i as usize] = true; } }
        u.iter().filter(|x| !**x).count()
    };
    println!("means only: {:?}", worst(&raw).iter().map(|(d, p)| format!("{d:.3} at r {:.2}", p.coords.norm())).collect::<Vec<_>>());
    println!("projected:  {:?}", worst(&proj).iter().map(|(d, p)| format!("{d:.3} at r {:.2}", p.coords.norm())).collect::<Vec<_>>());
    println!("unused vertices: {} of {}", used(&proj), proj.vertices.len());
    let off: Vec<f64> = fine.vertices.iter().map(|p| shell.evaluate(p)).filter(|d| d.abs() > 0.05).collect();
    println!("fine mesh: {} of {} vertices more than 0.05 mm off; worst {:?}", off.len(), fine.vertices.len(), off.iter().fold(0.0f64, |a, b| a.max(b.abs())));
    let mut n = vec![nalgebra::Vector3::<f64>::zeros(); fine.vertices.len()];
    for f in &fine.faces { let [a, b, c] = f.map(|i| fine.vertices[i as usize]); let x = (b - a).cross(&(c - a)); for i in f { n[*i as usize] += x; } }
    let wrong = fine.vertices.iter().zip(&n).filter(|(p, n)| { let r = p.coords.norm(); let radial = p.coords / r; (r > 19.0) != (n.dot(&radial) > 0.0) }).count();
    println!("fine mesh: {wrong} vertices whose normal points the wrong way for their sheet");
    let zero = n.iter().filter(|v| v.norm() == 0.0).count();
    let raw0 = shell.mesh(0.4).geometry;
    let mut refd = vec![false; raw0.vertices.len()]; for f in &raw0.faces { for i in f { refd[*i as usize] = true; } }
    println!("fine mesh: {zero} vertices with no triangle (zero normal); before welding: {} of {} unreferenced", refd.iter().filter(|x| !**x).count(), raw0.vertices.len());
}
