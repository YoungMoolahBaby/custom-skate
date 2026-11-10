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

## mesh (measure, io, repair)

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
- **works, note** (2026-11-10): `mesh_io` PLY stores f32: a 300 mm-scale mesh moves up to
  1.6e-5 mm on a round trip. Fine for scans; it matters only for exact-plane tests like the bug
  above.
- **note** (2026-11-10): `cf_scan_prep_core::clip_mesh_against_plane` (clip.rs:132) leaves the
  new cut vertices unshared between neighbouring triangles: the clipped leg stump had 147 open
  edges until `weld_vertices(1e-5)` merged 1,096 vertices, then 1 (the cut). Weld after clipping.

## sim-soft

- Not used yet. Hyperelastic only (`NeoHookean`, `Yeoh`, lib.rs:1-14), backward Euler, SDF → tet
  via isosurface stuffing. Phase 3 tries it on aluminium at small strain against a cantilever
  closed form; phase 4 uses it for the rubber bushing.

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
- **Weld in the clip** (2026-11-10): `clip_mesh_against_plane` could share the cut vertices, so
  the cut is one boundary loop straight away.
