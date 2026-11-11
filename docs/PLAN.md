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
  lever we have. Ice, a one-piece skate: the boot runs down into our own rocking holder, with
  our own runner and rocker insert (nothing proprietary, Jon 2026-11-11).

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

## Round 2: the chassis to weight (step 1 DONE 2026-11-10)

Jon: "start round 2". Plan: (1) the chassis to the weight target, (2) strength, (3) the last
tighter at the heel and midfoot, (4) Jon's own foot. `cargo run --release -- chassis` (~2 min,
`src/frame.rs`, `src/strength.rs`, `src/round2.rs`) builds the chassis alone, weighs it, checks
clashes at rest and rocked to both ends, runs the hand-formula checks, renders `docs/round2/`.

**What following the loads by hand showed** (`strength.rs`; loads in RESEARCH.md 13):
- Vertical loads barely size a frame: round 1's rails were ~5x stronger than they needed to be.
- Round 1's 6 mm rocker was the weak part: it bridges from heel to ball over the contact, ~100 N m
  at 3 BW on a 30 x 6 mm section, past yield.
- **The real driver is roll.** The lateral push at the wheels (0.3-0.6 BW in the skate's frame,
  speed skating) acts ~80 mm below the sole. A rocking joint has to pass that roll moment. Round
  1's rocker sat on top, 79 mm up and 30 mm wide: the vertical load alone holds a lateral /
  vertical ratio of only ~0.19 before one side lifts, so everything above that must go through
  bending walls or the bolt. This is probably where the R1's weight goes too.

**So round 2 moves the rocker down and out** (`frame.rs`): the upper is a web under the sole and
two walls hanging outside the lower rails; each wall's foot is the rocker arc (7.5 ft), rolling on a
ledge on the rail's outer face just above the axle heads. The tracks are 34 mm apart and 46 mm up,
so the vertical load alone holds lateral / vertical up to 0.37. Both the walls and the rails are
Warren trusses in their own planes (deep, so stiff for their weight). Bolt + bushing in a ring
between the middle wheels; two ties across the walls' feet through the rails' truss windows.

**Result (stand-in foot 252 mm, 4 x 72):**
- **174.2 g** with axles, spacers, bolt, nut and bushing: upper 57.0, ties 4.0, lower 77.9, axles
  21.7, spacers 5.8, bolt 3.9, nut 1.3, bushing 2.7. Round 1 was 281.4 g; one-piece 7075 frames
  are 170-184 g (bigger sizes); the R1 282-302 g.
- No clashes at rest or rocked +-50 mm (+-1.25 deg); the frame leans to 64 deg before touching
  the ground, after the axle heads (61 deg).
- Hand checks (100 kg skater; stride 1.5 BW fatigue, impact 3 BW static): worst stress / allowable
  0.74 (the ledge and bottom chord between bosses, 125 MPa at the stride). Upper 54 MPa, lower
  65 MPa, wall feet 119 MPa, axles <= 88 MPa, contact 64 MPa peak.
- **Like for like:** a one-piece built from the same members (same rails, wheels, stack, axles)
  weighs 110.9 g, so the rocker costs 63.3 g. The first sizing (2.5 mm rails and walls) weighed
  221.4 g and cost 101 g over its one-piece, about the R1's premium over real one-pieces.
- Our one-piece at 111-134 g is lighter than real ones (170-184 g, bigger sizes), so our members
  are probably on the light side of what frame makers use, likely for impact, wear and lateral
  stiffness. The 174 g is a paper weight until the lateral case is checked.
- Sweep (`examples/sweep_frame.rs`): the lower truss's top chord is essential (without it the
  lower is 3.8x over); the ties are 4 g; 1.8 mm members reach 168 g at 0.82.
- sim-opt didn't fit a mass search: its optimisers train policies on sim-core environments, there
  is no plain minimise (CORTENFORGE.md). The sweep did the job, and few parameters are active.

**Not checked by step 1, and it decides the design:** the lateral push (662 N at the stride)
bending the walls and rails sideways, torsion, buckling of the 2 mm walls, stress at holes and
corners. Step 2 below.

## Round 2, step 2: FEM (first results 2026-11-10)

Jon: "lets do round 2, step 2". `src/fem.rs`: sim-soft as a linear-elastic solver (Tet10 from the
cf-design solid, static, solved at 1e-3 of the load and scaled back, stresses recovered by us).
Experiments: `examples/fem_cantilever.rs`, `fem_probe.rs`, `fem_lower.rs`.

- **The solver checks out** (`check` 25/25): a 7075 cantilever's tip deflection within 1% of beam
  theory (3.5% where the mesher shaved an edge) and its bending stress within 0.5%; a 2 mm strip
  bent sideways 0.97-0.99 and 1.00-1.01. Two fixes were needed, both in CORTENFORGE.md: density 0
  (static), and the load scaling (sim-soft's finite-strain Newton stalls on slender aluminium at
  full load).
- **The lower frame, vertical stride** (full model with its axles, 2 mm cells, held at the rocker
  contact and by the walls along the rails): peak 181 MPa at the bottom edge of the rails under
  the contact. The hand formula said 125 MPa there, so it was ~30% low; the 2 mm mesh is also
  9.5% short of material, so the FEM reads high. Both under the fatigue allowable (174 MPa)
  only roughly: this needs a finer mesh.
- **The sideways push (0.45 of the vertical) breaks the 174 g frame**: ~580 MPa (both walls
  bearing on the rails along the upper) or ~530 MPa (one wall), and ~1.8-2.0 mm sideways at the
  axles. The peaks sit where the ledge and its backing end (x = +-70 mm) and near the end axles.
- **But a one-piece built from the same 2 mm members does worse**: ~743 MPa and 2.8 mm (clamped
  at its two mounts; vertical stride alone 94 MPa). The upper's walls hold the rails just above
  the axles, lower than a one-piece's mounts do, which shortens the rails' sideways lever.
  So the rocker layout is not the weak point; 2 mm members are, under this push. That fits
  real one-pieces weighing 170-184 g against our same-member one-piece's 111 g. **(Overtaken
  below: this held the lower against rigid walls. With the upper's walls modelled, they are
  the soft part, and the rocker comes out the softer frame.)**
- With 3 mm rails: ~330 MPa (rocker lower, peak at the sharp end of the ledge) and ~285 MPa
  (one-piece), both under yield, both still over the fatigue allowable at this push.
- The sideways push of 0.45 is from speed skating (RESEARCH.md 13), and it now decides the
  frame's weight. A hockey number would be worth more than any other single input.
- Not done yet: the upper's walls under the push (they carry it from the rails to the boot); a
  finer mesh (1.5 mm: ~3M DOF); torsion; buckling; tapering the ledge ends.

## Round 2, step 2 continued: like for like (2026-11-11)

Jon: "thicken the one-piece model until it weighs what real frames do, use its sideways stress
and stiffness as the target, then size the rocker's rails and walls to match and see what it
weighs". `src/sizing.rs`; one run = `cargo run --release --example size_frame -- <part> key=value
...`, ~2 min for a one-piece, ~3 for a rocker; every run is a row in `docs/round2/size_runs.csv`.

**How the two frames are compared.**
- **The same for both:**
  - loads: 100 kg skater, stride 1.5 BW, sideways 0.45 of it;
  - mounts: clamped on top, 165 mm apart and 36 mm long (an assumption until Jon's TF9 rivet
    pattern is measured);
  - mesh: 2 mm cells.
- **Two measures:**
  - the four axles' mean sideways give;
  - the peak von Mises stress away from the clamps and the rocker contact.
- **The rocker is two models.** The lower is held at its contacts, and sim-soft's reaction
  readout gives the forces there; they sum to the applied loads within 0.1 N, and the readout is
  in `check` (26/26). Those forces then load the upper, clamped at the mounts. The lower rides on
  the upper's contact faces, so the two gives add.
- **The coupling is one way:** the contact forces don't shift as the upper gives.

**The target: our one-piece, its rail plates thickened to real weights** (with axles and spacers;
real ones: 170-184 g):

| One-piece rails | Weight | Mean sideways give | Peak stress |
|---|---|---|---|
| 2 mm | 110.9 g | 2.57 mm | 743 MPa |
| 3 mm | 141.8 g | 0.68 mm | 284 MPa |
| **3.9 mm** | **169.5 g** | **0.35 mm** | **208 MPa** |
| 4.4 mm | 185.0 g | 0.25 mm | 156 MPa |

Give falls as 1/t³ and stress as 1/t², as plate bending predicts. The peaks sit on the rails just
under the inner end of a mount. Even at real weight, the one-piece is over our fatigue allowable
(174 MPa) at this push, which is one more reason to compare like for like rather than against an
absolute limit.

**The rocker, sized by thickness** (upper 240 mm long, 6 wall panels; rails / walls / web in mm;
premium = rocker weight minus a one-piece as stiff, interpolated from the table above):

| Rails / walls / web | Weight | Give: total (lower + upper) | Peak, lower / upper | Premium |
|---|---|---|---|---|
| 2 / 2 / 1.5 | 191.6 g | 2.01 mm (0.15 + 1.86) | 401 / 1,200* MPa | 76 g |
| 2 / 3 / 3 | 238.8 g | 0.84 (0.15 + 0.69) | 415 / 425 | 103 g |
| 2 / 3.75 / 1.5 | 247.0 g | 0.69 (0.15 + 0.54) | 398 / 352 | 106 g |
| 2.5 / 4 / 1.5 | 271.6 g | 0.50 (0.09 + 0.41) | 234 / 288 | 118 g |
| 2.5 / 3.75 / 3 | 281.6 g | 0.47 (0.10 + 0.37) | 237 / 234 | 125 g |
| 2.5 / 4.5 / 1.5 | 288.1 g | 0.42 (0.09 + 0.32) | 234 / 239 | 127 g |

*The 2 mm wall is one element thick at this mesh. Its peak sits where the contact loads go in,
and is mostly a modelling artefact.

**What it found.**
- **By thickening alone, the rocker doesn't reach a real-weight one-piece's stiffness at a
  sensible weight.**
  - The stiffest rocker run is 0.42 mm at 288.1 g; the 170 g one-piece gives 0.35 mm.
  - Following those last runs on, 0.35 mm comes at roughly 300 g: the R1's weight (282-302 g).
  - The premium at equal stiffness grows as the target stiffens: ~76 g at 2 mm parts,
    103-127 g as the rocker approaches the one-piece's stiffness.
  - Stress tells the same story: ~235 MPa at the stiffest rockers against 208 MPa for the
    170 g one-piece.
- **Why: the walls do the job a one-piece's rails do.** They carry the push 35-40 mm up to the
  boot as plates bending out of their own plane. So they end up as thick as a one-piece's rails,
  and the whole lower frame is the premium on top. Of the give, 0.09-0.15 mm is in the lower and
  the rest in the upper:
  - **The walls' own bending** falls roughly as 1/t³ (2 → 3.75 mm walls: 1.86 → 0.54 mm).
  - **A floor of ~0.2 mm that thicker walls don't remove.** Between the two mounts the upper
    twists like an open U-channel, and its windowed web shears in its own plane (0.13 mm at the
    centre).
  - **The web:** against the 1.5 mm windowed web, a 3 mm web takes 0.09 mm off for 18 g, and a
    solid 3 mm web 0.18 mm for 62 g.
- **The upper has to cover the mounts.** Step 1's 180 mm upper doesn't reach the assumed mounts
  (±100.5 mm). With 2 mm parts:

  | Upper length | Weight | Give: total (lower + upper) |
  |---|---|---|
  | 180 mm (step 1) | 174.2 g | 3.09 mm (1.03 + 2.06) |
  | 204 mm (just covers the mounts) | 180.0 g | 2.73 mm (0.19 + 2.54) |
  | 240 mm (covers the end axles too; 6 wall panels) | 191.6 g | 2.01 mm (0.15 + 1.86) |

  With its walls modelled, step 1's 174 g frame gives 3.09 mm, softer than the one-piece from
  the same 2 mm members (2.57 mm).
- **At the 0.45 push nearly all the vertical load moves to one ledge** (1,438 of 1,472 N). Both
  ledges stay in compression, so the low ledges do hold the roll, as designed; the coupling
  bolt isn't needed for it.
- **Bonded to a custom boot's sole** along its length (track C) instead of bolted at the mounts,
  the upper gives ~40% less (the whole frame ~33% less): 0.32 mm at 281.6 g (2.5 / 3.75 / 3),
  under the 170 g one-piece's 0.35. That's not like for like (a one-piece bonded the same way
  would gain too), but it is the lever a custom boot offers.

**Limits of these numbers.**
- **Mesh:** 2 mm cells lose 5-10% of thin parts' volume. The rocker's thinner parts lose more,
  so they read a little soft.
- **Coupling:** one way only.
- **Load cases:** the rocker at rest only; with the contact between two wall-truss nodes
  (rocked), the foot carries the 1.4 kN point load in bending. No torsion case yet.
- **Peak stresses** on the lower sit at the ends of the side-bearing band, where its rigid
  support begins.
- **The one-piece stand-in** is our own truss with thicker plates; a maker's sections may be
  more efficient, which would make the target harder.

**Next, for Jon to choose** (superseded by the V section's list below):
1. **Stiffer walls per gram than a plate.** Ribs or gussets on the walls' outer faces, deep at
   the root where the bending moment is, plus a web with diagonal ribs instead of windows.
   By hand, a 3 x 6 mm rib makes 40 mm of 2 mm wall ~10x as stiff for ~2 g.
2. **Take the side push high**, at the web, where the upper is stiff in its own plane. The
   rails then work like a one-piece's, but by hand the roll then needs ~1.4 kN from the bolt at
   0.45. A bushing can't give that, so it would take a hard stop.
3. **Build the roller skate on the custom boot**, with the upper bonded to the sole (track C).
4. **Accept a heavier rocker** and size it to a softer target.

A hockey sideways-load number still matters most for absolute stress, less for this
comparison. Then the coupling (phase 4) and steps 3-4.

## Round 2, step 2: the V section, "less material" (2026-11-11)

Jon: "how could we engineer this so theres less material needed?", then "go ahead" on the first
two ideas: point the parts at the wheels' contact patch, and make the web a truss.

**The idea.** In front view the ground force enters at one point, the contact patch. Two
members whose lines meet there carry any force through it by tension and compression, with no
bending. So the walls lean out along the line from the patch through their feet (20.3°: ~62 mm
apart at the web instead of 36), and the ledges tilt square to that line (`FrameParams::vee`).
The lower's rails already lie within ~1 mm of that line below the ledge.

**The catch, found while building it.** A tilted ledge can only press where the arc touches it:
the rolling patch, mid-length. The upright layout's side bearing, which slid along the whole
wall, has no tilted equivalent (only a face square to y allows the rocking motion). So each
axle's push has to travel lengthwise: through the lower to the patch, and through the upper to
the mounts. Two more options carry it: `web_truss` (the web as a Warren truss in plan) and
`deck_panels` (a truss deck across the lower's rails, over the wheels), and then `bulk_t`
(plates across the rails in the gaps between wheels, clear of the tyres by 0.75 mm).

**Modelling.**
- **Mesh:** a 2 mm wall leaning across the 2 mm mesh lattice fell apart (the upper lost 70% of
  its volume), so the uppers here are meshed at 1.5 mm (`cell_up`; -5.3% leaning, -4.0%
  upright), and the upright layout re-run that way for a like-for-like baseline.
- **The tilted contact:** sim-soft holds nodes only along x, y and z. So the heavy side's patch
  is pinned (it may stick by friction) and the light side presses along its tilted normal
  only, with the force that leaves no roll moment on the heavy patch (two solves superposed;
  `sizing::lower_vee`). The lower then rides on the upper's feet the same way.
- **A first try held both patches pinned.** The light side then carried 148 N of sideways force,
  which a contact can't, and it bent its 2 mm wall 2 mm locally. That run is superseded (its row
  in out/size/runs.csv reads 0.264 mm, which is wrong).

**Results** (2 mm parts, 1.5 mm web, upper 240 mm, 6 wall panels; lower at 2 mm cells, upper
at 1.5 mm):

| Frame | Weight | Give: total (lower + upper) | Peak, lower / upper |
|---|---|---|---|
| Upright (the step-2 layout), re-run at the finer mesh | 191.6 g | 1.75 mm (0.15 + 1.60) | 401 / 991* MPa |
| V | 202.5 g | 1.73 (1.45 + 0.27) | 598 / 345 |
| V + deck (6 panels) | 213.7 g | 1.35 (1.07 + 0.27) | 451 / 345 |
| V + truss web + deck (4 panels each) | 217.2 g | 1.34 (1.08 + 0.25) | 454 / 344 |
| V + truss web + deck (6 panels each) | 222.6 g | 1.33 (1.07 + 0.26) | 451 / 338 |
| V + deck + 2 mm bulkheads | 227.5 g | 1.00 (0.73 + 0.28) | 321 / 345 |

*As before, the 2 mm upright wall's peak sits where the contact loads go in.

**What it found.**
- **The V works in front view, as designed.**
  - The heavy side presses 1,750 N (hand: 1,739) and needs friction of only 0.01, so the push
    runs along the wall.
  - The upper carries the axles 0.27 mm instead of 1.60. At mid-length its wall foot moves
    0.16 mm instead of 2.70.
  - The light side pulls 184 N at the 0.45 push, so the coupling bolt needs a preload of at
    least 346 N (hand: ~320). That is well under the 1.4 kN of "take the push high".
- **But the gain moves into the lower.** With the push crossing only at the middle, the lower
  carries each axle's push, and its twisting moment, up to 114 mm lengthwise. Its section is
  open (two rails, plus a deck if added), so it twists. At the end axles:

  | Lower | Roll of the section | Rails leaning (racking) | End axles' sideways give |
  |---|---|---|---|
  | Deck only | 0.028 rad | 0.35 mm | 1.9 mm |
  | Deck + bulkheads | 0.024 rad | 0.15 mm | 1.3 mm |

  The inner axles (38 mm from the middle) give 0.17-0.26 mm. The bulkheads stop most of the
  racking, but the twist is the bigger part.
- **The truss web stiffens the web** (its top moves 0.02 mm instead of 0.07 at mid-length), but
  the web was no longer where the give was.
- **Per gram, the V is no better than thickening.** At 1.00 mm the V costs ~97 g more than a
  one-piece as stiff (interpolated as in the table above). The thickened upright frames cost
  76-127 g.
- **Why the upright layout's lower was so stiff:** the side bearing held it sideways along its
  whole length (0.15 mm). Whichever way the push crosses, one of the two parts pays:
  - along the length (the side bearing), the walls bend;
  - at one place (the rolling contact), the lower twists.

**Limits.**
- The V runs are at rest only, with the contact mid-length; rocked, the patch moves 50 mm
  toward one end.
- Coupling one way, as before.
- In the deck-only V the end of the lower's rail swings to within ~0.1 mm of the upper's wall
  foot, which would start a side contact the model doesn't have.
- The bulkheads' 0.75 mm clearance to the tyres is tight for real wheels.

**Next, for Jon to choose** (superseded by the ribbed walls' list below):
1. **Upright layout, ribbed walls** (old option 1). The upright layout keeps the lower stiff
   (0.15 mm), and its walls' bending was 1.60 of its 1.75 mm. Ribs attack that directly: by
   hand, a 3 x 6 mm rib makes 40 mm of 2 mm wall ~10x as stiff for ~2 g.
2. **A torsion box for the V's lower**: a closed cell along the frame. The only room is above
   the wheels, under the upper's web (8 mm of stack there today), so a box deep enough to matter
   raises the boot by ~10 mm.
3. **Pass the push near the mounts instead of mid-length**: two short links with pins across
   the frame, front and back, in place of the rolling arcs. Neither part then carries the push
   lengthwise. A four-bar's motion only approximates the rocker arc, so this is a new joint,
   with its own kinematics to check.
4. **Bond the upper to the custom boot's sole** (track C), as before: ~40% stiffer.
5. **Accept a heavier rocker.**

## Round 2, step 2: ribbed walls, and bonded to the sole (2026-11-11)

Jon: "try the ribbed walls on the upright layout", and the pinned links and the bonded sole
"both are interesting".

**The ribs** (`FrameParams::rib_d`, `rib_t`, `rib_taper`, `rib_chords`). Every truss member of
each wall gets a rib on its outer face, so it bends sideways as a T-section, not a flat strip:
the diagonals, the top chord (the wall's root, at the web) and the foot chord. The ribs are
2 mm thick and stand out 3-5 mm, so the walls are 42-46 mm across instead of 36. The upper is
meshed at 1.5 mm like the V runs (thin ribs at an angle need it; -2.2% to -3.3% volume), and
the comparisons below are all at that mesh.

**Results** (2 mm rails and walls, 1.5 mm web, upper 240 mm, 6 wall panels; premium = weight
minus a one-piece as stiff, interpolated as before):

| Upper | Weight | Give: total (lower + upper) | Peak, upper | Premium |
|---|---|---|---|---|
| Plain 2 mm walls | 191.6 g | 1.75 mm (0.15 + 1.60) | 991* MPa | 75 g |
| Walls thickened to 3.75 mm | 247.0 g | 0.59 (0.15 + 0.44) | 265 | 101 g |
| Ribs 5 mm, diagonals only | 214.8 g | 1.00 (0.15 + 0.85) | 481 | 85 g |
| Ribs 5 mm, diagonals + foot chord | 229.3 g | 0.67 (0.15 + 0.52) | 290 | 87 g |
| Ribs 3 mm, all members | 222.9 g | 0.68 (0.15 + 0.53) | 274 | 81 g |
| Ribs 5 mm tapering to 2 mm at the foot | 228.1 g | 0.62 (0.15 + 0.47) | 273 | 84 g |
| Ribs 5 mm, all members | 243.8 g | 0.49 (0.15 + 0.35) | 197 | 92 g |

*At the contact, where the loads go in; as before mostly a modelling artefact.

**Bonded to a custom boot's sole** (the whole web top held instead of the two mounts):

| Upper, bonded | Weight | Give: total (lower + upper) | Peak, lower / upper |
|---|---|---|---|
| Plain 2 mm walls | 191.6 g | 1.19 mm (0.15 + 1.04) | 401 / 905 MPa |
| Ribs 3 mm, all members | 222.9 g | 0.37 (0.15 + 0.22) | 401 / 312 |
| Ribs 5 mm, all members | 243.8 g | 0.275 (0.15 + 0.13) | 401 / 167 |

**What it found.**
- **Ribs beat thickening, by less than the hand sum promised.** At about the same weight, 5 mm
  ribs give 0.49 mm and 3.75 mm walls 0.59. At about the same give (0.6 mm), ribs save ~19 g.
  The premium falls from 101 g to 81-92 g. The hand sum (a rib makes a strip of wall ~10x as
  stiff for 2 g) holds for a strip; on the frame it doesn't, because a rib on every member of
  both walls is 31-52 g, and the lower (0.15 mm) and the web don't change.
- **The top chord's rib earns its place.** It is the wall's root, and with it the web twists
  less: at mid-length the web moves 0.11 mm with it, 0.28 mm without.
- **Bolted at two mounts, the rocker still doesn't reach one-piece weight.** The premium is now
  mostly the lower: it weighs 77.9 g, about what a whole one-piece frame does (83 g without
  axles).
- **Bonded to the sole, ribbed walls reach one-piece stiffness.**
  - 0.37 mm at 222.9 g (3 mm ribs), 0.275 mm at 243.8 g (5 mm ribs), against 0.35 mm for the
    170 g one-piece and 0.25 mm for the 185 g one.
  - That's ~60-70 g heavier than a bolted one-piece as stiff, and ~40-80 g lighter than an R1
    (282-302 g).
  - Not like for like: a one-piece bonded to a sole would gain too, and the model holds the
    sole rigidly, so these are best cases.
  - With 5 mm ribs the upper's peak is 167 MPa, under the fatigue allowable (174). The lower's
    401 MPa (at the end of the side-bearing band, partly a modelling artefact) is now the
    highest stress, and its 0.15 mm more than half the give.

**Limits.** At rest only; one-way coupling; the bonded runs hold the sole rigidly. The ribs
widen the frame to 46 mm at the walls, which is less clearance when leaning (to check against
the boot and the lean angles). Ribs this thin need a 1.5 mm mesh, so the earlier 2 mm-mesh
rocker table above reads somewhat soft.

**Next, for Jon to choose** (this replaces the lists above):
1. **The custom-boot route, made real.**
   - Model the boot sole as a real plate, not a rigid clamp: its stiffness is what the bonded
     numbers lean on.
   - Merge the upper's web into the sole (the web is ~13 g, by hand).
   - Lighten and fix the lower, now half the give and the peak stress.
2. **The pinned links** (Jon: interesting). First the motion, no FE: find two links, front and
   back near the mounts, whose pivot moves like the 7.5 ft rocker's contact (35 mm under the
   sole, moving forward ~40 mm per degree of pitch) over the ±50 mm travel. Then the link and pin
   forces from the loads, then the structure, like for like. The pins need no play (preloaded
   bearings, or flexure links: thin strips that bend instead of pins that turn). Check the
   patents before making one (README).
3. **Bolted at two mounts (Jon's TF9s):** ribbed walls, 81-92 g over a one-piece as stiff.

## Track B, round 2: the hockey boot (2026-11-11)

Jon: "lets not use anything proprietary, so no i2 inserts. can we work on the boot first? it
would be cool to see the full boot in its glory then go back to the bladeholder".
`cargo run --release -- boot scans/stand-in/022_fBW_R.ply` (~6 min; `BOOT_VIEWER=1` adds the
3D viewer's light meshes, seconds more) builds the whole boot on stand-in 022 (`src/hockey_boot.rs`).
Renders and the summary are in `docs/boot/`; STLs and the viewer's files go to `out/boot/`. The
viewer page is `viewer/boot.html` (turn, explode, cut through the ankle or lengthwise).
Its meshes are the uniform ones decimated by our own `cluster_decimate`: cf-design's
`mesh_simplified` left spikes and slivers that Jon spotted in the first viewer (CORTENFORGE.md).

![The hockey boot, round 2, three-quarter views from the outside front and the inside back](boot/boot_hero.png)

**Placed from the scan, not from fractions of the length.**
- **The ankle bones:** the outermost skin points each side in the ankle region. On 022 the
  medial one is at (66, 46, 84) and the lateral at (48, -23, 78) mm, so the medial one is 6 mm
  higher and 18 mm further forward, as anatomy has it. They set the cuff height, the ankle
  pockets and the pads.
- **The dorsal line:** the line over the top of the foot, from the toes up the instep and the
  front of the shin. Rays are fanned from behind the ankle to find it, and it is centred
  between the sides of the foot and leg. The throat, tongue, eyelets and laces follow it.
  Centring matters: the shin's most forward point sits 6-9 mm toward the medial side, and
  following it put the medial eyelets on the side of the ankle.
- **The leg:** the scan stops 125 mm up, so above 110 mm the leg is carried up as a prism of
  its own section.

**The parts** (Jon's list, items 1 and 3-6):

| Part | g | Material | What it is |
|---|---|---|---|
| shell | 237.6 | PA 12 | 2.5 mm, cuff trimmed 75 mm above the higher ankle bone (z 159 at the leg's centre, rising 8° to 166 at the back), flaring with the liner at the top (below), ankle pockets bulging 3 mm, an eyelet facing (+1.5 mm) along each side of the throat |
| toe cap | 21.0 | PA 12 | 1.5 mm over the shell's front, ending at the throat |
| outsole | 83.0 | PA 12 | 3 mm flat plate under the shell with a small welt; where track C takes over (the roller's sole becomes the chassis's upper) |
| liner | 35.4 | foam | 4 mm, following the shell's flare at the top, so the room is between the leg and the liner |
| ankle pads | 3.0 | foam | the liner within 24 mm of each ankle bone, 7 mm thick in the pockets |
| footbed | 8.1 | foam | the liner under the foot |
| tongue | 11.5 | foam | from under the toe cap to 40 mm above the cuff, padding under the eyelet rows and a raised core filling the throat |
| tendon guard | 11.3 | TPU | 2.5 mm, overlapping the shell's back, 55 mm above the cuff, kicking 12 mm back at its top |
| eyelets | 5.6 | 6061 | 7 a side, grommets through the facing |
| laces | 6.0 | lace | criss-cross, tied in a bow |
| **total** | **422** | | the same shell in carbon/epoxy at 2.5 mm would weigh 365 g; a laid-up shell would be thinner |

- **Layers (item 6), from the foot out:** 1 mm fit allowance (plus the flare's room at the
  top), 4 mm liner (more at the ankle pads), 2.5 mm shell (+1.5 mm facing), 1.5 mm toe cap. The two sections
  (`docs/boot/boot_section_*.png`) show the stack.
- **The cuff: tight from heel to toe, open at the top (Jon, 2026-11-11).** "my favorite/go to
  for skating mechanics is train 2.0. a lot of it is able ankle flexion ... so having a bit of
  flare at the top of the skate help a lot to get the mobility. then tight from heels to toe.
  so like basically you theoretically wouldnt even have to tie the skate, and it would stay
  on/performant, and youd have near full range ankle mobility." He found the first version
  tight and restrictive going into the lower leg, "at least for shorter players". His reference
  (a photo of a player skate): flare at the top to both sides, a little room in front, and a
  slight negative flare at the back "to pitch you forward a little bit".
  - **What was wrong:** the first version flared only the shell, and the liner filled the
    flare with collar foam, so the leg met foam after ~1 mm (0.7° of tip).
  - **Now** the liner and shell flare out together above the ankle bones (from 5 mm above the
    higher one, growing as t^1.5 to the top), leaving room between the leg and the liner:
    **sides 13 mm, about 9.4° of tip each way** before the leg meets the liner (pivoting at the
    ankle bones' height; the liner's own give comes on top); **front 7 mm, 5.5°** before the
    tongue; **back -1 mm**, the liner pressing the calf to pitch the leg forward. Below the
    ankle bones nothing changed: the 1 mm fit allowance from heel to toe.
  - **To tune with Jon:** the angle is atan(room / height above the ankle bones). At this
    cuff height (78 mm from the ankle bones to the top at the sides), 15° each way needs
    ~21 mm of room and 20° ~28 mm, which starts to lose the cuff's support. A lower cuff gets
    the same angle with less flare: at 60 mm, 15° needs ~16 mm. So the cuff height (B1 on the
    TF9) and the flare are one choice.
- **The throat (item 3):** runs from the toe cap up the instep and the front of the leg to the
  top, 26 mm wide at the front, 32 mm at the cuff.
- **Weights:** each part is weighed from its mesh's enclosed volume. At a finer mesh the shell
  is unchanged (237.55 g at 0.75 mm cells, 237.64 at 0.5) and the tendon guard is within 0.3%.
  The toe cap needed 0.3 mm cells: it came out 3.9% light at 0.5. `mass_properties` agrees
  on the tendon guard (11.39 g against 11.34). Foam, TPU and lace densities are assumed.
- **Estimates until measured on the TF9** (MEASURE.md B1-B7): the cuff, tongue and tendon guard
  heights, the throat widths, and the eyelet count and spacing.
- **Not in yet:** Jon's own foot; a forward lean; a tighter heel lock (the last's round 2);
  puck protection (item 7, the air-gapped shield); a sensor bay (item 8); the fit simulation
  (B5). And the sole that becomes the roller chassis's upper or the ice holder (track C).

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

**Ice.** ~~The boot takes his Marsblade i2 (254) to start.~~ **Jon, 2026-11-11: a one-piece
skate**, "as the rocker sits inside the blade holder. and the blade itself rocks against it" (his
reference: a one-piece goalie skate whose carbon boot runs down into the holder, the steel bolted
to it). So the boot and the holder are one part. The rocking stays where the i2 has it: inside
the holder, between a rocker insert and the runner. It mirrors the roller skate, whose sole is
the rocker's upper: one last, two one-piece skates, each with the rocking joint built in.
- **Nothing proprietary (Jon, 2026-11-11: "lets not use anything proprietary, so no i2
  inserts").** Our own runner (the steel) and our own rocker insert, designed with the holder.
  The i2 is only a reference for the feel Jon knows.
- **The pitch can be matched by choosing our radii:** GIH's eq. 10 gives the effective radius
  of a runner on a rocker, so the pair can be set to the roller's 0.251°/cm. For reference, the
  i2's 11 ft runner on its 23 ft insert works out to 7.46 ft, 0.252°/cm (RESEARCH.md 14; the
  insert radii are unverified).
- **Our own runner is a real job:** hardened stainless steel, cut, heat-treated, ground to a
  profile, and sharpenable on a normal sharpener, so its blank shape stays standard where the
  sharpener holds it.
- **Patents:** check Marsblade's before making one to sell.

## Jon's list for the boot, the skates and the gear (2026-11-11)

Jon's items, in his words, each with a first engineering take and where it goes. Items 1 and
3-6 have a first version on the stand-in foot (Track B, round 2: the hockey boot, above); Jon's
own boot waits on his scans (B1).

1. **"tendon guard".** B4. The guard above the heel. How stiff it is decides how far the ankle
   can extend at push-off, so it's a feel choice, and the part can be made to suit either way: a
   separate flexure (printed TPU, or thin PA12 shaped as a spring) on the shell, sized by
   bending stiffness with sim-soft.
2. **"align wheel and blade radius point so the feel the same for anchors and soft drags for
   both roller and ice".** Track C, with `src/pitch.rs`. One "feel spec" in the foot's own
   frame (both boots come from the same last), and both skates set to match it:
   - **where the pivot sits under the foot:** the blade profile's radius point on ice (the
     runner's profile and the i2), and the rocker's rest contact on roller (`center`, 0.47 of
     foot length now);
   - **how fast the foot pitches** per cm of weight shift (the tables at the top of the README);
   - **the contact length that turns on an edge:** the blade profile's radius against the
     wheels' layout.

   **Jon's ice setup** (2026-11-11): "i just sharpened them half inch on the original profile the
   i2 blades come with". That's the factory i2 runner, an 11 ft radius, and a 1/2" hollow, which
   sets bite and glide, not pitch. The pitch also depends on which rocker insert is in his holder:
   with the factory 23 ft insert (if the catalog figure is right), the ice skate pitches
   0.252°/cm and the 7.5 ft roller rocker 0.251°/cm, so the two already match (RESEARCH.md 14).
   Since we use no i2 parts (Jon, 2026-11-11), that's the target our own runner and insert
   copy. **Still to measure, as the feel reference:** the insert in his holder, and where the
   runner's lowest point sits under the foot.
3. **"the tongue slot should be adjusted so its from the toes to the heel, direction wise".**
   B4. Round 1 cuts the throat as a straight box, 32% to 70% of the foot's length and 40 mm wide,
   vertical-walled (`boot.rs`). **Jon, 2026-11-11: "probably more like a traditional hockey
   skate, with the top cuff around the ankle flared a bit for ankle mobility"** (his reference:
   a player skate in profile). So:
   - the throat runs from the toe cap up the instep and the front of the ankle to the top, with
     a full-length tongue;
   - the cuff wraps the ankle and flares outward at its top edge;
   - the trim line rises at the back into the tendon guard (item 1).

   The lines are placed from landmarks on the scan (ankle bones, instep, heel) rather than as
   fractions of the length, so they follow his foot.
4. **"the tongue itself".** B4/B7. Its own part, shaped to the last's instep: padding against
   lace bite, a stiffer outer layer, and its stiffness sets part of the boot's forward flex. A
   graded printed lattice (`mesh-lattice`) could do comfort, flex and puck protection (item 7)
   in one piece.
5. **"eyelits".** B4. Two rows along the throat's edges, placed from the last so lace pressure
   spreads over the instep. A printed shell needs a reinforced boss at each eyelet; their
   pull-out under lace tension is a sim-soft check.
6. **"layers".** B4/B5. The stack per region (heel pocket, ankle, instep, toe box): shell, liner
   foam, padding, footbed, and the outsole that carries the chassis or holder. Round 1 used one
   uniform stack (1 mm ease, 4 mm liner, 2.5 mm shell); round 2 of the boot makes it regional.
7. **"potential air gapped layer, like the shot blocker attachment for skates, but built into
   these? ... it would hurt more for ice hockey than roller. as these are basically 0 tolerance
   to the foot (which performance wise is optimal, we just need to keep the comfort/protection
   too."** B4/B5. A stiff outer shield held off the foot over a gap (air or a soft lattice), so
   an impact bends the shield and spreads the load before anything reaches the foot. With a
   zero-gap fit the gap has to be designed in, outside the fit layers. The ice boot gets more
   of it than the roller boot.
   - **Sizing:** sim-soft can run the impact: a puck's mass and speed, the shield's stiffness and
     the gap's thickness, giving peak force on the foot. Puck masses and speeds need a source.
   - **Placement:** where pucks hit, to decide with Jon (laces and instep, toe cap, ankle sides).
   - **Cost:** weight, counted like the chassis.
8. **"thinking early about building in electronics/sensors to the skate to help you improve
   your skating".** Track C, planned now so the parts leave room for them:
   - an IMU on each part (stride, edge angle, and, from the upper and lower together, how much
     the rocker is actually used);
   - pressure insoles, like the Brock study's;
   - strain gauges on the chassis (load per wheel, the sideways push this sizing still lacks);
   - a sealed bay for battery and logger (wet and cold on ice).
9. **Shin pads made with the skates** (Jon: "tuning shinpads to skates and getting it perfect is
   kind of difficult. we could honestly probably just make a full set of equipment."). The pad's
   bottom meets the skate's tongue and cuff, so they're designed together, from the same scan
   extended up the shin (the Otter can scan a leg). The pad's cap and the tongue's shield
   (item 7) are one protection system: what overlaps, and what moves when the ankle flexes
   forward. A full set of equipment comes later, as its own track, shin pads first.

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
- Answered 2026-11-11: his ice blades are the i2's factory profile with a 1/2" hollow; the
  tongue slot and cuff like a traditional hockey skate, the cuff flared; the ice skate one
  piece, with the rocker inside the holder.
- Which rocker insert is in his i2 holder (the inserts may be marked), to confirm the radii of
  the feel he knows.
- Answered 2026-11-11: the one-piece ice holder takes our own runner and insert, nothing
  proprietary; and the boot comes first ("it would be cool to see the full boot in its glory
  then go back to the bladeholder").
