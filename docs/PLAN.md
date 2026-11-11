# Plan

**Scope (Jon, 2026-11-10): the full skates, "one ice skate, one roller skate".** Custom boots made
from a 3D scan of Jon's feet; the roller skate on an aluminium rocking chassis designed with its
boot, the ice skate on a dynamic (rocking) blade holder. Three tracks that meet in the sole:

- **Track A, the chassis**: an aluminium rocking roller chassis (rounded upper chassis on a flat
  lower chassis, sprung coupling) in the 170-185 g class of premium one-piece 7075 chassis, strong
  and stiff enough to play on. The Marsblade R1 is 282-302 g. Phases 0-7 below.
- **Track B, the boot**: scan → last → shell → liner → fit simulation → test shells → a real boot.
  Phases B1-B7 below.
- **Track C, the skates**: roller, the boot's sole becomes the chassis's upper (the part that
  rocks), so a whole part, the rivets and the mount plate go away; that is the biggest weight
  lever we have. Ice, the boot takes a dynamic holder (his Marsblade i2 254 to start).

Everything is modelled in CortenForge and checked against known results first (CLAUDE.md).

What CortenForge brings:
- **cf-design**: SDF solids, `mass_properties` (grams), the `mechanism` module (parts, joints,
  MJCF out to sim-core, an elastomer `Bushing`), meshing, STL.
- **sim-core**: rigid-body dynamics with contact, for the rocking frame on its wheels.
- **sim-soft**: hyperelastic FEM (Neo-Hookean, Yeoh) on tet meshes built from SDFs. A natural fit
  for the rubber bushing, and possibly for aluminium at small strain (untested).
- **sim-opt**: optimisers (simulated annealing, CEM) for the mass search.
- **mesh**: printability checks, repair, STL out.

## Round 1: the whole skate on an example foot (DONE 2026-11-10)

Jon: "before we do anything on my side i think we should just do the full thing on an example
foot". `cargo run --release -- round1 scans/stand-in/022_fBW_R.ply` (~2.5 min) runs every stage
at round-1 quality on stand-in 022 (man, 175 cm, loaded foot 252 x 104 mm). Renders and the
summary are in `docs/round1/`; STLs go to `out/round1/` (not in git).

- **Foot** (`foot.rs`): the scan in a standard frame (heel at x = 0, toes +x, floor z = 0), with a
  mesh-sdf distance grid (1.5 mm, ~26 s).
- **Last** (`round1.rs`, `foot.rs`): the foot closed by 12 mm (arch and small hollows filled) +
  the convex hull of the forefoot from 62% of the length (cf-geometry Quickhull) as the toe box,
  faded in over 60 mm, + 1 mm ease. Closing alone can't make a toe box (toes are ~20 mm cylinders
  side by side; closing by r leaves a groove ~a^2/2r deep; 7 and 18 mm both left visible grooves).
- **Boot** (`boot.rs`): liner 4 mm, shell wall 2.5 mm, trimmed at 110 mm, throat 40 mm wide from
  32% to 70% of the length, flat 3 mm sole plate. **Shell 277 g in printed PA 12, 425 g in
  carbon/epoxy; liner 44 g** (foam density assumed).
- **Fit** (`fit.rs`): gap from the real foot to the last. Heel, midfoot, instep and ball sit at
  ~1.0 mm (the ease); room only between and over the toes (p90 4-8 mm). Round 1's last never
  cuts in, so this is a room map, not pressure.
- **Roller chassis** (`chassis.rs`), a first layout, not a design: 4 x 72 mm wheels on a 76 mm
  pitch, a 7075 rocker under the sole (7.5 ft radius, 6 mm, pocketed), two 2.5 mm 7075 rails with
  scallops and a 4 mm bridge it rolls on, a Ti M6 bolt through an 18 x 10 mm PU bushing hung in the
  V between the middle wheels; ground 85 mm below the boot sole. **281 g: rocker 48, lower frame
  208, axles 20, bolt 3, bushing 3.** No clashes between the wheels and the frame, bolt or
  bushing. So the naive layout weighs what the R1 does (282-302 g); the job is ~100 g out, and
  the lower frame (208 g) is where it is.
- **Ice skate**: the same boot on a 254 mm holder **placeholder** (i2 not measured yet).
- Not in round 1: strength (phase 3), the spring rate (phase 4), dynamics (phase 5), heel lock,
  tendon guard, eyelets, tongue, variable wall thickness, the i2's real mounting pattern.

**Round 2** (next): the chassis to the weight target (pocket the rails, lighten the bridge and
rocker, then sim-opt), a first strength check of the rails and rocker, the last shaped tighter at
the heel and midfoot with sim-soft pressures, then Jon's own foot (ruler first, scans later).

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

- **B1. The scan.** Jon's scanner: a **Creality CR-Scan Otter** (handheld, infrared structured
  light, up to 0.02 mm at 60 mm per the spec sheet, 0.05-2 mm resolution, exports OBJ / STL / PLY,
  all of which CortenForge's `mesh-io` reads). Both feet, weight-bearing (as True scans), maybe
  also in a skating crouch (ankle bent forward), which changes the heel and instep. The sole can't
  be scanned while standing on it, so: scan the top standing, and the sole from a foam impression
  box (or a plaster cast of it). A cast also holds still, unlike a foot. `check` first: scan an
  object of known size (a gauge block, a machined part) the same way and measure the error. Then
  tape and caliper measurements of the feet (length, ball width, ball / instep / heel girths) as
  a check on the scan. Scans stay in `scans/` (gitignored).
  **Round 1 runs on a stand-in scan** (Jon, 2026-11-10: he can't scan his own feet alone yet):
  Schuster, Cresswell & Kelly (2024), Dryad, **CC0**, 100 people, dominant foot at minimal and at
  full body weight, PLY (RESEARCH.md 12). Scaled to Jon's foot length and width (ruler, MEASURE.md)
  until his scans exist. Dryad blocks scripted downloads, so Jon fetched the whole dataset in a
  browser (2026-11-10). It lives on D: (`D:\custom-skate-data\dryad-g4f4qrfwt`, 1.2 GB) and
  `scans/stand-in` is a junction to it (gitignored).
- **B2. Scan prep.** Clean, cap the ankle, align, measure; compare left and right.
  **`scan <file>` (2026-11-10)** loads OBJ / STL / PLY, converts to mm, welds, finds the foot's
  frame, and measures length, width, height, the open edges, and a girth profile heel to toe.
  - The frame comes from the surface, not principal axes (a 125 mm leg stump tilts those ~12°):
    up from the open cut's plane, else from the most near-level surface (10° cone) signed so the
    lower half is the long one, snapped to the flat sole; heel to toe from the lower 35% of the
    scan; the heel is the taller end (`scan::measure` docs). Three earlier rules failed on real
    scans and are recorded there: principal axes (8 of 200 wrong), a 30° cone (tilted 14°), thin
    bands for the sign (flipped at 7° of tilt).
  - **On all 200 stand-in scans** (`examples/batch_scan.rs`), against the dataset's own
    alignment: up within 2.7° on every scan, heel never swapped, length -0.48% to +1.45%
    (median -0.03%). 192 found up from a flat sole, 8 from the level surfaces.
  - `check` 18/18: the synthetic cases (ellipsoid extents exact, girths -0.002% / -0.003%,
    leg stump cut and frame exact) plus three real scans that broke earlier versions (skipped
    where the data isn't present). Found and worked around a mesh-measure section bug
    (docs/CORTENFORGE.md).
  - Stand-in choice: the participant table (read with mat-io; RESEARCH.md 12) joined to the
    measurements. Men within 5 cm of Jon's 173 cm: loaded foot 243-286 mm, ball width 91-113 mm,
    so the stand-in is picked by Jon's foot length and width (MEASURE.md F1-F2), not his height.
  - **Next**: pick and scale the stand-in; ball and instep girths on their anatomical planes, not
    just perpendicular ones; then B3, the last.
- **B3. The last.** The foot plus fit allowances: toe room, a locked heel pocket, the ankle. True
  and Bauer (patent US 12,135,869 B2) both go scan → custom last → boot.
- **B4. The shell.** Around the last: wall thickness by region, toe cap, eyelet rows, tendon guard,
  and the sole, which carries the chassis or the holder (track C). Weight by part. One last, two
  boots: the ice boot needs more puck protection and stands in cold and wet; the roller boot gets
  the rocking sole.
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

## Track C: the skates

**Roller.** The sole is the rocker's upper chassis: one part instead of boot sole + mount plate +
rivets + upper chassis. Weigh the whole skate, like for like: a TF9 7W + R1 with 80/76 wheels is
1396 g a skate; Mako 8EE + Sprung 1231-1334 g (Coast Hockey Shop). Jon's skates, weighed, are the
real baselines.

**Ice.** The boot takes his Marsblade i2 (254) to start, so the ice skate keeps a rocker it already
has and the work is the boot. The sole carries the i2's mounting pattern (measure it). Later,
possibly our own dynamic holder, with the GIH model (`src/pitch.rs`) setting the rocker radius.

## Open questions for Jon

- Answered 2026-11-10: the name (rocker-chassis, renamed custom-skate for the full skate); size (True 6, Marsblade M a bit big, S too
  small, i2 254).
- Answered 2026-11-10: he has both R1 and O1 in Medium; the O1 M fits, the R1 M is a bit big.
  O1 on a True TF9 size 6 with 72s; R1 on a Tour size 7.
- Answered 2026-11-10: the full skates, one ice and one roller; scanner = Creality CR-Scan Otter;
  the repo is public (MIT OR Apache-2.0); renamed custom-skate.
- The measurements in `docs/MEASURE.md`, and the first scans.
- Manufacturing route: CNC 7075 or metal printing (chassis); printed shell or carbon over a
  printed last (boots).
