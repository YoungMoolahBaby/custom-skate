# Plan

**Scope (Jon, 2026-11-10): the full skate.** A custom boot made from a scan of Jon's feet, on an
aluminium rocking chassis, designed together. Two tracks that meet in the sole:

- **Track A, the chassis**: an aluminium rocking roller chassis (rounded upper chassis on a flat
  lower chassis, sprung coupling) in the 170-185 g class of premium one-piece 7075 chassis, strong
  and stiff enough to play on. The Marsblade R1 is 282-302 g. Phases 0-7 below.
- **Track B, the boot**: scan → last → shell → liner → fit simulation → test shells → a real boot.
  Phases B1-B7 below.
- **Track C, the skate**: the boot's sole becomes the chassis's upper (the part that rocks), so a
  whole part, the rivets and the mount plate go away. That is the biggest weight lever we have.

Everything is modelled in CortenForge and checked against known results first (CLAUDE.md).

What CortenForge brings:
- **cf-design**: SDF solids, `mass_properties` (grams), the `mechanism` module (parts, joints,
  MJCF out to sim-core, an elastomer `Bushing`), meshing, STL.
- **sim-core**: rigid-body dynamics with contact, for the rocking frame on its wheels.
- **sim-soft**: hyperelastic FEM (Neo-Hookean, Yeoh) on tet meshes built from SDFs. A natural fit
  for the rubber bushing, and possibly for aluminium at small strain (untested).
- **sim-opt**: optimisers (simulated annealing, CEM) for the mass search.
- **mesh**: printability checks, repair, STL out.

## Phase 0: the science and the tools (DONE 2026-11-10)

- Sources found and read (docs/RESEARCH.md): the Brock / George Brown poster (2023), the GIH model
  (Daggfeldt & Petré 2025), the GIH sprint study (Psilander et al. 2025), the R1 specs, patent
  US 9,782,665 B2, one-piece chassis weights.
- `src/pitch.rs`: the GIH model, eqs. 1-11, plus the roller limit (s → ∞) and passive rocking
  stiffness.
- `check` 10/10: the model against the paper's own numbers (30 cm on 2.7 m = 6.37° < 0.12 rad;
  linearisation error 0.0886° < 0.1° at β 0.12 rad; two routes agree to 1e-16; eq. 11 = slope of
  eq. 7; both limits). cf-design `mass_properties` against exact volumes: at 0.25 mm cells
  +0.09% (3 mm plate), +0.17% (2 mm wall with a hole), +0.02% (rod), centre of mass exact. That is
  ~0.3 g on a 180 g chassis, in 0.02 s per part.
- `pitch`: the R1's 5-15 ft rocker inserts give 0.125-0.376° per cm of weight shift with all wheels
  down, bracketing 9-13 ft ice blades (0.145-0.209°/cm). A one-piece chassis gives 0° until it tips.
  Passive rocking stiffness for a rigid 75 kg skater: 386-606 N m/rad on a 5 ft rocker,
  2628-2849 N m/rad on 15 ft (CoM 0.7-1.0 m above the contact, an assumption).

## Jon's fit (2026-11-10)

True size 6, i2 ice holder 254. He has both Marsblade roller frames in Medium, on different
skates: **the O1 Medium feels good, the R1 Medium a bit big**. They are different sizes: the O1 M
fits 5.5-7.5 on 72 mm wheels, the R1 M fits 6.0-8.5 on 76/80 and shares its lower chassis with the
Large (up to 11.5), so it is ~24-28 mm longer and overhangs a size 6 boot (a shop reports the same
on a 6.0). **The design takes the O1 Medium's footprint (length, wheel size, stack) and the R1's
idea (metal lower, rocker plates).** A frame cut for his foot is also shorter than the R1 M, so it
is lighter: sizing is one of the weight levers.

**Design target (2026-11-10): his True TF9 size 6 with 4x72 wheels**, the setup that fits (his O1
Medium). The TF9 is an ice boot, so the chassis rivets on at the holder's hole pattern (front and
rear rivet groups), which we measure. His R1 sits on a size 7 Tour boot, so its "too big" is
partly the boot. Weights must be compared at this size: the one-piece listings (170-184 g) are
mostly for bigger wheels and frames, so a 4x72 frame for a size 6 should come in under them. Requirements from the critique (RESEARCH.md 8): no play, a
clear snap-back to centre, metal where pucks and asphalt hit.

## Phase 1: the baselines, measured

Three references, all at his size:
- **The fit**: his O1 Medium, measured (`docs/MEASURE.md`): axle and mount spacing, wheel
  envelope, overhang, stack height. The mount spacing is a hard constraint: our chassis must bolt
  to his True boot.
- **The rocker to beat**: his R1 Medium, weighed, with its rocker plate and travel.
- **The weight class to match**: one-piece 7075 chassis, 170-184 g (README).

Then a parametric one-piece Hi-Lo chassis in cf-design (two side rails, axle bosses, the bridge,
two mount cups) at his size, with his measured mount spacing and wheel layout. Output: the mass
breakdown (rails, bridge, cups, bosses, hardware) and the wall thickness that lands in the
170-184 g class. This tells us where a one-piece chassis puts its grams, at his size.

## Phase 2: the rocker chassis, first geometry

Upper (boot mounts, rocker surface, side guides), lower (wheel rails, flat top, axle bosses),
coupling (bolt, bushing, nut), torsion element (the patent's stiffening rod). A mass budget by
part against the baseline. Hypotheses to test, not results yet:
- the R1's extra grams come from two full structures stacked on each other, plus a nylon upper
  that needs thick sections to be stiff enough;
- an aluminium upper can be a single central web (an I-section rocker) riding in a lower U-channel,
  sharing walls instead of doubling them;
- the hardware (bolt, torsion rod) is a few grams in titanium or aluminium.

## Phase 3: loads and strength

Load cases from the sources: in-skate peak forces 900-1200 N on ice (Brock / George Brown, Table 2)
as the starting point, plus the components the insoles can't see (the lateral push at the edge,
torsion about the frame's long axis, a fall or board hit). Hand formulas first (beam bending of the
rails, torsion, contact stress at the rocker), then FEM. CortenForge has no linear-elastic solver.
sim-soft's Neo-Hookean model reduces to linear elasticity at small strain, so try it on aluminium
against a cantilever closed form first, and log the result. Then stress, deflection and fatigue
(7075-T6 endurance limit) for each part. Stiffness target: no softer than the one-piece in the
side push and in torsion.

## Phase 4: the coupling, engineered, and the feel

The spring element sets how the rocker feels. Jon's experience (2026-11-10): **no complaints about
his O1s**. His **Sprungs** (on TF9s) are "a good idea", but the bushings break, his have, and once
they're old "it gets sketchy": "they didnt do enough engineering work before manufacturing". So
the spring element gets the engineering up front:

1. **Fail-safe.** If the spring element breaks or wears out, the skate stays safe: hard stops
   limit the travel, the boot stays attached, and the worst case is a stiffer or looser ride,
   never a loose frame.
2. **Sized for life, not for day one.** Count the load cycles (strides a session x sessions a
   season x seasons) and the peak loads, then size the spring element for fatigue, permanent set
   and ageing (heat, ozone, UV, sweat, water). Compare elastomers with metal springs (steel or
   titanium leaf, coil or flexure), which have a fatigue limit and don't age. Aluminium has no
   true fatigue limit, so the chassis parts get the same life check.
3. **Inspectable and replaceable.** Wear shows before it fails, and a spare is cheap and swaps
   with a hex key.
4. **No play.** Preloaded, so the frame doesn't clunk or wobble.
5. **Light.** Jon (2026-11-10): "we can definetely explore a bushing with the rocker. my concern is
   weight". Weigh each option by the energy it must store (stiffness x travel squared / 2) over the
   energy per gram it can store for its fatigue life. Elastomers store the most elastic energy per
   unit mass (Ashby, *Materials Selection in Mechanical Design*, springs), coil springs are close,
   leaf springs and flexures are several times heavier because bending leaves most of the
   material lightly stressed. Make the stops shoulders on the aluminium parts, not added parts.
   Give the coupling (spring, bolt, nut, stops) its own line in the mass budget.

The rate: cf-design's `Bushing` gives shear rates only (torsion, axial). A Marsblade-style bushing
is squeezed as the frame rocks, which is compression, so compute that rate in sim-soft (Neo-Hookean
or Yeoh, durometer from Shore A); a metal spring has closed forms. Compare the rocking stiffness
with the passive stiffness from `pitch` (386-2849 N m/rad), so the spring centres the frame
without fighting the skater.

## Phase 5: dynamics

cf-design mechanism → MJCF → sim-core: the rocking frame on four wheels under a stride's force and
pitch profile. Check rolling contact on a known case first (sim-core 0.9 has known capsule and
cylinder contact bugs; see the cortenforge-cfsem log). Outputs: wheels-down time, load per wheel
through the stride, rocker vs one-piece.

## Phase 6: optimise

sim-opt over the geometry parameters: minimise mass subject to stress, fatigue and stiffness no
worse than the one-piece. Target ≤ 185 g, same size and what's included as the baseline.

## Phase 7: make it

Manufacturing route (CNC 7075 billet vs printed aluminium with lattice), printability checks, STL,
a drawing for a machine shop. Patent check before anything is sold (README).

## Track B: the boot

What CortenForge brings here: `scan-prep-core` (clean a scan mesh), `cap-planes` (find and cap a
scan's open loops, like the ankle opening), `mesh-measure` (lengths, girths), `mesh-offset` and
cf-design `offset`/`shell` (fit allowance, boot walls), `mesh-shell` (printable shells),
`mesh-lattice` (padding: gyroid, octet), `sim-soft` (soft tissue and padding pressed against a
rigid shell), `mesh-printability`, `mesh-repair`, STL out.

- **B1. The scan.** Both feet, weight-bearing (as True scans), maybe also in a skating crouch
  (ankle bent forward), which changes the heel and instep. Methods: phone photogrammetry of the
  bare foot (the foot moves during capture, so less accurate), or a foam impression → plaster cast
  → photogrammetry (casts measured to ~0.9 mm, Ellis et al. 2026), or a depth-camera phone app.
  `check` first: scan an object of known size the same way and measure the error. Then tape and
  caliper measurements of the feet (length, ball width, ball / instep / heel girths) as a check
  on the scan.
- **B2. Scan prep.** Clean, cap the ankle, align, measure; compare left and right.
- **B3. The last.** The foot plus fit allowances: toe room, a locked heel pocket, the ankle. True
  and Bauer (patent US 12,135,869 B2) both go scan → custom last → boot.
- **B4. The shell.** Around the last: wall thickness by region, toe cap, eyelet rows, tendon guard,
  and the sole, which carries the chassis (track C). Weight by part.
- **B5. The fit simulation.** sim-soft: the foot's soft tissue (Neo-Hookean, moduli from the
  literature) and the liner in the shell, under standing and push-off loads. Output: a pressure
  map, compared with pressure-pain thresholds and with the Brock poster's insole pressures. Check
  sim-soft on a contact case with a closed form (Hertz) first.
- **B6. Test shells.** Cheap printed shells (PETG, TPU) to try on standing, no skating. Iterate the
  last until it fits.
- **B7. A real boot.** Either a printed shell (MJF nylon, carbon-filled nylon), which has not been
  done for a hockey boot as far as we found, or a carbon layup over a printed last, as True does.
  A heat-mouldable liner, tongue, laces. Safety: it must stop a puck and hold at speed, so it is
  tested standing, then skating slowly, before it is played in.

## Track C: the skate

The sole is the rocker's upper chassis: one part instead of boot sole + mount plate + rivets +
upper chassis. Weigh the whole skate, like for like: a TF9 7W + R1 with 80/76 wheels is 1396 g a
skate; Mako 8EE + Sprung 1231-1334 g (Coast Hockey Shop). Jon's two skates, weighed, are the real
baselines.

## Open questions for Jon

- Answered 2026-11-10: the name (rocker-chassis); size (True 6, Marsblade M a bit big, S too
  small, i2 254).
- Answered 2026-11-10: he has both R1 and O1 in Medium; the O1 M fits, the R1 M is a bit big.
  O1 on a True TF9 size 6 with 72s; R1 on a Tour size 7.
- The measurements in `docs/MEASURE.md`.
- Manufacturing route: CNC 7075 or metal printing.
- GitHub: public or private.
