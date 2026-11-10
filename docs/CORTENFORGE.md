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
