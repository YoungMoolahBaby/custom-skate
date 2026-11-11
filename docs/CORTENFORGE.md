# CortenForge findings

End-user notes on the published `cortenforge` 0.9.0 crates, logged while building. Each entry:
type (bug, docs, API, perf, setup, works), date, file:line under
`~/.cargo/registry/src/*/cortenforge-*-0.9.0/`.

## cf-design

- **works** (2026-11-10): `mechanism::mass::mass_properties` (mechanism/mass.rs:78) is accurate
  enough to weigh a chassis. Against exact volumes in 7075-T6, at 1 / 0.5 / 0.25 mm cells:
  3 mm plate +1.48 / +0.37 / +0.09%; 2 mm wall with a 20 mm hole +2.78 / +0.70 / +0.17%;
  8 x 60 mm rod +0.55 / -0.13 / +0.02%; centre of mass exact. 0.02 s per part at 0.25 mm.
  Coarse cells read heavy on thin flat walls, so use 0.25 mm for walls of 2-3 mm.
- **API** (2026-11-10): `MassProperties` is re-exported at the crate root (lib.rs:51-56) but the
  function that makes one, `mass_properties`, is not; it is only at
  `cf_design::mechanism::mass::mass_properties`.
- **API, by design** (2026-11-10): `Bushing` (mechanism/bushing.rs:1-12) gives torsional and axial
  rates (pure shear, closed form) and leaves out the radial rate on purpose. A Marsblade-style
  coupling bushing is squeezed as the frame rocks, which is the case it leaves out. Planned:
  sim-soft (phase 4).
- **bug, minor** (2026-11-10): `Solid::ellipsoid(130, 50, 40).mesh(1.0)` (solid/query.rs:82)
  gives a closed 2-manifold (every edge in 2 faces) that still has 24 vertices doubled within
  1e-6 mm and 48 sliver faces between them; `weld_vertices(1e-6)` removes them. Harmless for
  rendering, but it breaks mesh-measure's section chaining (below). Probe:
  `examples/probe_section.rs`.
- **works** (2026-11-10, round 1): scan-to-boot in cf-design works as hoped. A scan's mesh-sdf
  grid goes into `Solid::from_sdf` (solid/ops.rs:576); `offset`, `subtract`, `intersect`,
  `smooth_union`, `Solid::plane` and `Solid::user_fn` (a sole plate extruded from the foot's
  outline, a toe box faded in along x, both evaluating other `Solid`s inside the closure) compose
  into a last, liner and trimmed shell; `mesh(1.0)` gives a 438k-triangle shell in ~10 s;
  `mass_properties` weighs it. The roller chassis is plain primitives and booleans.
- **note** (2026-11-10): `Solid::offset` and `shell` "require exact SDF input" (ops.rs:351-358,
  306-312), so a scan's offsets must come from a real distance field (mesh-sdf), and growing then
  shrinking one field is a no-op: a morphological closing needs the grown surface meshed and
  re-distanced (`foot::Foot::closed`).

- **works** (2026-11-10, round 2): `Solid::evaluate` sampled on a plane gives exact section
  properties for beam checks (`strength::section`, with `mass_properties`' partial-cell weight in
  2D): a 30 x 6 bar, an offset I-beam and a 20/14 tube come out at their closed-form area, second
  moment and centroid to the printed digits at 0.1 mm cells (`check`). A truss drawn as a 2D
  `user_fn` profile (members as capsules joined by a smooth minimum, bosses, holes) and extruded
  composes with primitives and booleans into a whole chassis: 8 parts weighed at 0.25 mm in ~21 s.
- **note** (2026-11-10, round 2): `mass_properties` gives cells within half a cell of the
  surface a partial weight (mechanism/mass.rs:234, `clamp(0.5 - d / cell)`), which is right for
  mass. But the intersection of two parts 0.1 mm apart (a rocker on its track) then weighs as a
  few mm3 of "overlap" at 0.25 mm cells (2 mm3 at rest, 11 mm3 rocked; 0 at 0.1 mm). For clash
  checks, count only cells whose centres are inside both (`round2::overlap_mm3`).

- **API** (2026-11-11, the boot): the QEM decimator (simplify.rs:681, `simplify_mesh`) is not
  exported (lib.rs has no `pub mod simplify`), so an existing mesh can't be decimated. The public
  route is `Solid::mesh_simplified` (query.rs:277): re-mesh with the adaptive octree, then
  decimate. The octree prunes by `evaluate_interval` (adaptive_dc.rs:253), and a plain `user_fn`
  answers (-inf, +inf), so nothing is pruned without a hand-written interval.
- **bug** (2026-11-11, the boot): `mesh_simplified` threw vertices off the surface. On the boot's
  parts (a Lipschitz interval given, ~18 min for ten parts on 12 threads), vertices sat up to
  ~330 mm off the toe cap and outsole and 20 mm off the liner, and triangles spanned 150-390 mm:
  spikes and dark slivers in the 3D viewer (Jon saw them). Dual contouring clamps its vertices
  to their cells (adaptive_dc.rs:460), so it's the decimation. `optimal_point`
  (simplify.rs:59, used at :281) takes the quadric's minimum whenever the determinant is above
  an absolute 1e-15; on flat patches the quadric is near-singular but in mm its determinant
  clears that easily, so the minimum lands far along the flat direction. And no collapse is
  checked for flipping triangles. Instead: `hockey_boot::cluster_decimate` on the uniform mesh
  (seconds, every vertex snapped back onto the part's field).
- **works** (2026-11-11, the boot): ten parts of a hockey boot as layered offsets of one scan
  field (`user_fn`s sharing an `Arc`), plus primitives (grommets as cylinders, laces as a
  `pipe`), meshed in parallel threads (the uniform mesher; weights converge, shell +0.0% from
  0.75 to 0.5 mm cells).

## sim-opt

- **API** (2026-11-10, round 2): the optimisers (`Sa`, `RicherSa`, `Pt`; algorithm.rs:83-110)
  implement sim-ml-chassis's `Algorithm` and train a `Policy` on a `VecEnv` built from a sim-core
  `Model` (sim-ml-chassis vec_env.rs:91-118). There is no plain black-box minimise
  (`f(x) -> cost` over bounded parameters), so a static design search (chassis mass under stress
  limits) would have to pose as a policy acting in a dummy physics model. Instead: a parameter
  sweep (`examples/sweep_frame.rs`); with the stresses well under their limits only a few
  parameters are active.

## cf-geometry

- **works** (2026-11-10): `convex_hull` (convex_hull.rs:167, Quickhull) takes 10,074 forefoot
  points to a 1,287-vertex, 2,570-face hull instantly; it makes the toe box of a last.

## mesh-sdf

- **perf** (2026-11-10): `CachedGridSdf::build` (flood_fill.rs:592) spends ~11 us a node on a 33k-
  vertex foot: 8.4M nodes at 1 mm took 89 s, 2.3M at 1.5 mm ~26 s. It is the slowest step of
  round 1 (three grids: the foot, the grown foot, the toe box).
- **note** (2026-11-10): out-of-bounds queries return the clamped edge value (documented at
  flood_fill.rs:625), so an offset surface beyond the grid's margin comes out silently wrong, not
  as an error. Keep the margin larger than the largest offset taken from the grid.
- **note** (2026-11-11, the boot): the clamping also bites when a small grid is blended with a
  large one. The toe box's grid (14 mm margin) smooth-unioned into the closed foot's (35 mm)
  returned its flat edge value ~30 mm behind the leg, which undercut the foot's own distance
  there: the tendon guard, offset ~30 mm from the leg at its top, grew a flat horizontal tab.
  Fix: outside its box, add the distance to the box (foot.rs `last_base`).

## mesh (measure, io, repair)

- **API** (2026-11-11, the boot): no rigid registration (ICP or similar) for scans. The
  stand-in set has each foot loaded and unloaded (full and minimal body weight), and checking
  that a foot can get into and out of a near-rigid boot needs the unloaded foot in the loaded
  foot's frame; Jon's own scans will need the same (and left against right, and a re-scan
  against the first). Each scan's standard frame (`foot::canonical`: heel at x 0, floor at z 0,
  centred on the footprint) moves with the load, since the sole spreads. Instead:
  `hockey_boot::align_to`, point-to-surface ICP on the target's distance grid (each point moved
  to its projection, then Kabsch's SVD), over a band of the foot that load and posture change
  least (20-70 mm up): stand-in 022 unloaded onto loaded, RMS 1.3 mm.
- **note** (2026-11-11, the boot): `weld_vertices` (repair.rs:310) merges duplicates by
  remapping the faces but leaves the merged-away copies in the vertex list, unreferenced (1,284
  of 85,104 on a meshed 2 mm shell). Harmless for STL and volumes; but anything that walks the
  vertex list (per-vertex normals, clustering) sees zero-normal strays. Follow it with
  `remove_unreferenced_vertices` (repair.rs:424), as `round1::mesh_of` now does.
- **bug** (2026-11-10): `mesh_measure::cross_section` (cross_section.rs:156) returns broken
  loops when the plane passes through or within ~1e-5 mm of vertices. A vertex on the plane
  gives `d_a * d_b == 0`, which counts as a crossing (line 459), so a triangle can yield the same
  point twice or three hits, and only exactly-two-hit triangles make a segment (line 180).
  Near-vertex planes give segments shorter than the chaining tolerance, an absolute `1e-6`
  (line 493), so `chain_segments` (line 472) joins or reverses them wrongly. Then
  `contour_perimeter` (line 316) closes each open chain back to its start, so the perimeter
  comes out near 2x and the area near 0. Measured on a cf-design ellipsoid section through its
  centre (grid vertices on the plane): 199 loops, perimeter 1021.47 vs 283.62 mm, area 0.
  Through a PLY round trip (f32 coordinates scatter on-plane vertices by ~1e-5 mm) at 1/4 and
  1/2 length: 8 and 26 loops, perimeters +95% and +99%. Off-vertex, welded: 1 loop, -0.007%.
  Workaround in `scan::section`: weld on load, and nudge the plane 3.3 um until no vertex is
  within 1 um of it. Then 1 loop, perimeter -0.002% / -0.003%, area exact (`check`).
- **works** (2026-11-10): `mesh_measure::oriented_bounding_box` gives a tilted ellipsoid's
  extents exactly (260.00 / 100.00 / 80.00 mm). `cf_scan_prep_core::detect_boundary_loops` finds a
  scan's cut edge exactly (219.90 mm vs a 219.91 mm circle).
- **works** (2026-11-10): `mesh_io::load_mesh` reads the stand-in scans (ASCII PLY from Meshmixer,
  with normals and colours, ~4.5 MB) without fuss: 200 files load and measure in ~100 s, about
  0.5 s each. `cf_scan_prep_core::fit_plane_to_points` gives the cut's plane on the leg-stump case
  exactly (alignment 1.000).
- **works, note** (2026-11-10): `mesh_io` PLY stores f32: a 300 mm-scale mesh moves up to
  1.6e-5 mm on a round trip. Fine for scans; it matters only for exact-plane tests like the bug
  above.
- **note** (2026-11-10): `cf_scan_prep_core::clip_mesh_against_plane` (clip.rs:132) leaves the
  new cut vertices unshared between neighbouring triangles: the clipped leg stump had 147 open
  edges until `weld_vertices(1e-5)` merged 1,096 vertices, then 1 (the cut). Weld after clipping.

## sim-soft

- Hyperelastic only (`NeoHookean`, `Yeoh`, lib.rs:1-14), backward Euler, SDF → tet via
  isosurface stuffing. Used since round 2 step 2 as the chassis's linear-elastic solver (below);
  phase 4 uses it for the rubber bushing.
- **note** (2026-11-10, round 2): promising for that: its own tests check bending against beam
  theory (tests/bonded_bilayer_beam.rs: a cantilever vs Euler-Bernoulli, converging with
  refinement; tests/slender_bending_matches_analytic.rs; tests/tet10_bending_locking.rs), and
  run static problems as backward Euler with a long time step (dt = 1 s). Next step of round 2.
- **works** (2026-11-10, round 2): **sim-soft is a usable linear-elastic solver for aluminium**
  (`src/fem.rs`). A cf-design `Solid` goes straight into `SdfMeshedTetMesh::from_sdf` (it
  implements the shared `Sdf` trait), then `Tet10Mesh::from_tet4`, then `CpuTet10NHSolver` with
  `density = 0` (static). A 100 x 10 x 10 mm 7075 cantilever: tip deflection 0.991 / 1.035 /
  1.012 / 0.992 of beam theory at 2.5 / 2 / 1.5 / 1.25 mm cells (the 1.035 where the mesher lost
  0.4% of the volume at the edges); mid-span surface bending stress 1.000 / 1.000 / 0.995 /
  1.000 of M c / I (nodal-averaged). A 60 x 20 x 2 mm strip (a thin wall bent sideways): 0.966
  and 0.994 of beam theory at 1.0 and 0.8 mm cells, stress 1.000 and 1.012. In `check`.
- **note, important** (2026-11-10, round 2): the Newton solve **stalls on slender stiff parts at
  their real load**. sim-soft solves finite strain with an Armijo line search on the residual
  norm (solver/backward_euler/newton.rs:1130-1170). Its first step from rest is the linear
  solution, which moves points along straight lines, so a slender part also stretches by
  ~theta^2 / 2; times aluminium's in-plane stiffness that makes a residual several times the
  load, full steps are rejected and Newton crawls: a 100 x 10 x 10 bar took 5 iterations (the
  first four reducing the residual 1.66 -> 1.06 N), a 60 x 20 x 2 strip did not converge in 50,
  and the lower frame without its axles not in 12. That residual grows with the load squared
  while the answer is linear in it, so **solve at 1e-3 of the load and scale the displacement
  back** (`fem::LOAD_SCALE`): 2 iterations everywhere, the same answers. The residual has an
  absolute floor near 1.3e-7 N (mm, N, MPa units), so the tolerance needs a floor too.
- **API, stub** (2026-11-10): `Observable::stress_field` is `unimplemented!("skeleton phase 2")`
  (observable/basic.rs:90). Instead: `fem::corner_stresses` (F from `Tet10::shape_gradients` at
  the corners, P from `NeoHookean::first_piola`, Cauchy = P F^T / det F) and `fem::nodal_stresses`
  (averaged over the elements sharing a node; raw corner maxima read 3-14% high and got worse
  with refinement).
- **perf** (2026-11-10): the round-2 lower frame with its axles at 2 mm cells: 1.4M DOF (with
  the lattice's unused points, which sim-soft pins) in ~115 s, 2 Newton iterations, 32 GB
  machine. Thin members lose volume to the BCC + stuffing mesher: 2 mm rails -9.5% at 2 mm
  cells, -3.9% at 1.5 mm (so stresses read high). Mesh quality stays good (worst dihedral
  ~14 deg, no slivers).
- **note, unexplained** (2026-11-10): a half model cut on a symmetry plane (the solid
  intersected with a box, a roller u_x = 0 on the cut) gave a 1,060 MPa spot on the cut face,
  ~6x the 181 MPa of the full model at the same place, with a node shared by only 4 elements
  and a ~3 um displacement jump to its neighbours. Cause not found (our boundary conditions or
  the mesher at the cut); full models only for now.
- **works** (2026-11-11, round 2 step 2): **the reaction readout**, `nodal_reaction_forces`
  (solver/backward_euler/assembly.rs:415, -f_int after a static solve). On the rocker's lower,
  held at its contacts, the forces at the held nodes sum to the applied loads within 0.1 N
  (662.2 N sideways, 1471.5 N vertical; `fem::solve_with_reactions`, scaled back like the
  displacements). That is what lets the two-piece frame be solved as two substructures: the
  lower's contact forces become the upper's loads (`src/sizing.rs`).
- **API, limit** (2026-11-11, round 2 step 2, V section): **rollers hold only global x, y, z**
  (`BoundaryConditions::roller_vertices: Vec<(VertexId, [bool; 3])>`, readout/scene.rs:913).
  A contact on a tilted face (the V section's ledges, 20 deg) needs a node held along its normal
  only. Instead: the heavy side pinned (it sticks by friction; the friction it needs is printed,
  0.01) and the light side loaded along its normal by a force found from two solves and
  superposition (`sizing::lower_vee`).
- **note** (2026-11-11, round 2 step 2): **a thin wall leaning across the lattice is lost at a
  cell size equal to its thickness**. The V section's 2 mm walls, leaning 20 deg, at 2 mm cells:
  the stuffing broke them into pieces, `largest_component` kept the web, and the upper lost 70%
  of its volume (upright 2 mm walls at 2 mm cells lose 11%). At 1.5 mm cells: -5.3% (upright
  -4.0%). Nothing warns; we check the meshed volume against `mass_properties` on every run.

## Setup

- **works** (2026-11-10): the facade with default features (sim, mesh, fabrication) builds in
  2 min 40 s from a warm registry, release, on the Ryzen 5 5600.

## Suggestions

Nice-to-haves for CortenForge, with what this project built instead.

- **Linear-elastic structures** (2026-11-10): a small-strain solver for metal parts (isotropic E
  and nu, static solve, natural modes, von Mises and safety factor) on cf-design solids. Wanted
  here for the chassis rails, rocker and axle bosses under skating loads; also wanted by
  cortenforge-cfsem (coil supports, the mothership's structure). Instead: hand formulas, then
  sim-soft at small strain if it passes a cantilever check.
- **A bushing compression rate** (2026-11-10): `Bushing` covers shear. A radial or compression
  rate for a bonded rubber block (shape factor corrections) would cover rocker couplings and
  suspension mounts. Instead: sim-soft on the bushing's tet mesh.
- **Re-export `mass_properties`** at the cf-design root next to `MassProperties`.
- **Robust plane sections** (2026-11-10): classify vertices against the plane with a tolerance
  (on-plane vertices shared, not duplicated), chain segments by mesh edge identity instead of by
  position, scale the tolerance to the mesh, and report open chains as open (with their length)
  instead of closing them. Foot girths are the core measurement of a boot last. Instead:
  `scan::section` (weld + nudge).
- **Faster scan distance grids** (2026-11-10): a narrow-band or fast-marching build for
  `CachedGridSdf` (exact distances only near the surface, propagated outward) would cut round 1's
  ~80 s of grid building. Or an error, not a clamped value, for queries outside the grid.
- **A morphological closing** (2026-11-10): `Solid::close(r)` (grow, re-distance, shrink) for
  scan-derived solids. Instead: `foot::Foot::closed` meshes and re-grids the grown surface.
- **Weld in the clip** (2026-11-10): `clip_mesh_against_plane` could share the cut vertices, so
  the cut is one boundary loop straight away.
- **A black-box optimiser** (2026-11-10, round 2): `minimize(f, x0, bounds)` in sim-opt for
  design searches that have no dynamics (mass under stress limits). Instead: a parameter sweep.
- **A linear-elastic static mode for sim-soft** (2026-11-10, round 2): one factorisation of the
  small-strain stiffness, no line search; plus the stress field. Wanted for metal parts.
  Instead: solve at 1e-3 of the load and scale back; our own stress recovery. (This replaces
  the "linear-elastic structures" wish above: the solver is there, the static mode isn't.)
- **Skew rollers and a lost-volume warning** (2026-11-11, round 2 step 2): a roller along any
  unit vector (or a local frame per node), for contacts on tilted faces; and a warning from
  `SdfMeshedTetMesh::largest_component` (sdf_bridge/sdf_meshed_tet_mesh.rs:278) when the parts
  it drops hold more than a few percent of the volume, which is how a too-coarse mesh shows.
- **Tied or contacting meshes** (2026-11-11, round 2 step 2): two meshed bodies in one solve,
  bonded on a named interface (shared or constrained nodes) or in contact (penalty or mortar),
  so an assembly like the rocker's upper on its lower is one model and the contact forces
  redistribute as the parts give. Instead: two substructures, the upper loaded by the lower's
  reaction forces (one way).
- **Decimate a mesh, safely** (2026-11-11, the boot): export `simplify_mesh` so an existing mesh
  (a scan, a uniform mesh) can be decimated; in it, a relative singularity test for the quadric
  (or keep the new vertex near the collapsed edge, falling back to its ends or midpoint) and a
  flip check per collapse. And `user_fn_lipschitz(f, l, bounds)`, whose interval is the centre
  value +- l x half-diagonal, so adaptive meshing prunes scan-derived parts. Instead:
  `hockey_boot::cluster_decimate`.
- **`weld_vertices` that compacts** (2026-11-11), or a flag for it.
- **Rigid registration of scans** (2026-11-11, the boot): `align(source, target, region) ->
  (Isometry3, rms)` in mesh-measure or scan-prep-core: ICP (point-to-plane), with a region
  mask, a trimmed-outlier option for scans whose shapes differ in places (a loaded and an
  unloaded foot), and the residual per point to see where they differ. Instead:
  `hockey_boot::align_to`.
- **Section properties and interference** (2026-11-10, round 2): `Solid::section(plane)` (area,
  centroid, second moments) for beam checks, and `Solid::overlap_volume(&other)` that ignores the
  partial-cell band, for clash checks. Instead: `strength::section`, `round2::overlap_mm3`.
