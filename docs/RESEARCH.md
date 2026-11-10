# Research

Sources, what they found, and what they can and can't support. Read 2026-11-10.

## 1. Brock University + George Brown College (2023): the Canadian study

Lockwood K, Dunne C, Hsu T-T (Brock University), Ellingson J-A (George Brown College), Ontario.
"Effect of a Fixed versus Dynamic Pitch on Hockey-Specific Skating Skills: A Comparative
Analysis". Conference poster, early 2023 study, released by Marsblade 2023-12-20.
PDF: https://images.marsblade.com/uploads/2023/12/poster_spin-2023_fixed-versus-dynamic.pdf
(file name "spin-2023"). Lockwood is also President of the Skating Lab Inc.

- **Pitch**: "the pivot point along the length of a skate blade that defines the angle between the
  blade and the ice". Fixed pitch = one pivot point; dynamic pitch = "a range of pivot points to
  maximize blade-ice contact".
- **Phase 1, model**: blade contour from an imaging system, the dynamic holder's range of pitch
  angles measured, and foot kinematics from skating trials used to compute contact point, contact
  length and pitch angle through the stride. No numbers on the poster.
- **Instruments**: Xsens Awinda, 17 IMUs at 60 Hz (hip, knee, ankle angles); XSENSOR X4 pressure
  insoles at 120 Hz (in-skate pressure). PCA with single-component reconstructions.
- **Phase 2, treadmill**: n = 11 male collegiate players; forward, inclined forward, c-cuts,
  backward. Marsblade's page gives the treadmill settings: 8 mph at 8°, and 4 mph at 2°.
- **Phase 3, on ice**: forward, c-cuts, backward, agility turns, and combination drills. n not
  stated.

Table 1 (treadmill), forward skating, fixed → dynamic:

| | fixed | dynamic | change |
|---|---|---|---|
| normalised stride length | 51.75 | 51.95 | |
| hip / knee / ankle angle (deg) | 59.57 / 65.54 / 12.67 | 59.87 / 65.07* / 12.50 | |
| mean force (N) | 383.13 | 362.96 | -5.3% |
| peak force (N) | 728.16 | 671.80 | -7.7% |
| impulse (N s) | 420.40 | 387.58 | -7.8% |

Marked significant (*) in Table 1: knee angle in forward skating; stride length in c-cuts
(88.73 → 90.90) and backward skating (106.89 → 110.05); ankle angle in backward skating
(0.61 → 1.21). The force columns carry no marks. (Some cells of the inclined-forward and
c-cut rows did not extract cleanly from the PDF; only the rows above are quoted.)

Table 2 (on ice), fixed → dynamic: forward mean force 476.89 → 350.43 N, peak 1136.33 → 898.92 N,
impulse 363.46 → 262.01 N s; c-cuts peak 883.37 → 500.26 N; backward peak 1009.40 → 874.00 N;
agility turns peak 1196.44 → 915.27 N. No significance marks.

Conclusion on the poster: "support for the contribution of a dynamic pitch to skating efficiency"
and "less force was required to maintain the same velocity".

**Limits.** A poster, not a paper. Hosted and announced by the maker. The force is plantar
pressure inside the boot, not the blade-ice force, so it misses shear and the lateral push. The
poster marks no force difference as significant. It tested the i2 ice holder, not a roller frame.
**Use here**: the mechanism (pitch range at equal contact), and in-skate peak forces of
~900-1200 N on ice as a starting load for phase 3.

## 2. Daggfeldt & Petré (2025): the model

Daggfeldt K, Petré H. "A mathematical model of dynamic skate blade holders". Proc IMechE Part P:
J Sports Engineering and Technology, 1-5. doi:10.1177/17543371251389073. Open full text on GIH's
DiVA (diva2:2012085). Received 2025-07-02, accepted 2025-10-01. No funding, no declared conflicts.

- Fixed holder: x = α r (eq. 1). 30 cm on a 2.7 m blade < 0.12 rad (~7°).
- NHL blades 8-12 ft in the early 2000s; today typically 9-13 ft (2.7-4 m).
- Dynamic holder (rocker radius r on a blade of radius s and height h):
  β = arctan(α r / (s − h)) (eq. 3); φ = θ + α + β (eq. 4); y = β s (eq. 5);
  φ(y) = θ + tan(y/s)(s − h)/r + y/s (eq. 7); φ ≈ θ + y (r + s − h)/(r s) (eq. 9);
  effective radius r s / (r + s − h) (eq. 10); amplification A ≈ (r + s − h)/r (eq. 11).
- Their claim, linearisation error < 0.1° at β = 0.12 rad (s 2.7 m, h 3 cm, r 1 m): reproduced,
  0.0886° (`check`).
- Real blades: fixed radius over ~60% of the length, tighter radius in the front and back 20%.
- The trade-off a rocker removes: a smaller blade radius gives more pitch but less contact area,
  and beyond a pressure threshold plowing resistance dominates over wet friction.

**For roller**: a flat lower chassis on its wheels is s → ∞. β = 0 and the foot pitches x / r,
so the rocker radius is the effective radius (`check`, `pitch`).

## 3. Psilander, Arndt & Petré (2025): sprints

"Velocity profiles of high-calibre ice hockey players: a comparison between fixed and dynamic
skating blade holders". Footwear Science, epub 2025-09-23. doi:10.1080/19424280.2025.2563837.
GIH's first study with Marsblade. Elite players, 5-40 m sprints. GIH's news item: the dynamic
holder "significantly enhanced backward skating speed", by 1-2% in speed and finish time. It
reports no forward gain. Full paper not read yet.

## 4. Marsblade's roller frames

- **Mechanism** (Inline Warehouse): "The upper chassis has a rounded bottom portion that sits on top
  of a flat lower chassis". Sizes: S 8x68 (skate 3-5), M 8x72 (5.5-7.5), L 8x76 (8-11);
  "advanced" setups 6 large + 2 smaller per pair.
- **R1** (Hockey World Blog, 2020-04-20): upper glass-fibre reinforced polyamide, lower
  aluminium-magnesium alloy, 282 g without wheels and bearings, "almost 100 grams lighter" than
  the O1. Swappable rocker inserts, radius 5-15 ft, four in the box. Hi-Lo: L and M 80 rear / 76
  front, S 72 / 68. Retail weight 302 g (MD, Inline Warehouse; inclusions not stated).
- **O1**: 376 g (MD), resin.

## 5. The patent

US 9,782,665 B2, "Roller skate". Flow Motion Technology AB (formerly MarsBlade AB), inventor Per
Mårs. Priority SE1000121, 2010-02-09; granted 2017-10-10; continuation of US 9,101,816 B2. Active;
Google Patents estimates expiry 2031-02-02 (an estimate, not legal advice).
- Upper chassis: boot mounts front and rear; a curved (radius) underside; vertical side segments
  outside the lower chassis for torsional stiffness.
- Lower chassis: wheels (four in the main embodiment), a flat top.
- Coupling: a vertical bolt through both, in a rubber (or spring) bushing held by a nut. Tightening
  the nut preloads the bushing and narrows the rocking range. The bushing compresses on the side
  the skater presses and springs back.
- A stiffening rod from the upper through a groove in the lower adds torsional stiffness.
- Variants: both surfaces curved, an exchangeable insert between them (the R1's inserts), outer
  wheels set higher.
- No dimensions are given in the text.

## 6. One-piece chassis weights

- Hi-Lo Kryptonium (7000 series): 175 g S/M, 184 g L, with axles and spacers (Coast Hockey Shop).
- Rink Rat Rinkster Sniper Hi-Lo (7075-T6): ~170 g MD (Inline Warehouse).
- Konixx Bolt Hi-Lo: 173 g MD (spec sheet), 170 g (product copy) (Ice Warehouse).
- Heavier: Hi-Lo S19 207 g (SR MD), Tour Fuse Split Hi-Lo 223 g (MD).

## 7. Sizing

- **R1 size chart** (retailers, 2026-11-10; the maker's chart not found): S fits skate 3.0-5.5,
  (2) 68 + (2) 72 mm; M fits 6.0-8.5, (2) 76 + (2) 80; L fits 9.0-11.5, (2) 76 + (2) 80.
- **O1 size chart** (Inline Warehouse): S 3.0-5.0, 8x68 per pair (advanced 6x68 + 2x64); M 5.5-7.5,
  8x72 (6x72 + 2x68); L 8.0-11, 8x76 (6x76 + 2x72). The "M 5.5-7.5, L 8-11" listing that seemed to
  conflict with the R1 chart is this one: **the O1 and R1 Mediums are different sizes.**
- O1 M vs R1 M: four wheels' diameters add up to 288 mm (4x72) or 284 mm (3x72 + 68) on the O1,
  312 mm (76/76/80/80) on the R1, so the R1 M is ~24-28 mm longer (12-14 mm a side) and its 80 mm
  rear wheels put the heel higher.
- **The M and L share one lower chassis**; only the plastic mounting plate grows (Coast Hockey
  Shop, below). So the M's wheelbase is the one used for feet up to 11.5.
- **Ice holder sizes** are a length in mm (211-322 mm across brands, Ice Warehouse guide). A
  generic chart puts 246 mm at a junior 5 / 5.0-5.5; Marsblade's own i2 chart wasn't found.
- **Jon** (2026-11-10): True size 6. He has both an R1 and an O1, Medium, on different skates.
  **The O1 Medium feels good; the R1 Medium feels a bit big**; a Small "feels too small". His i2
  ice holder is size 254. The review below reports the same R1 overhang on a 6.0 skate: the
  Medium "overhung on the front and back of the boot a bit".
  - O1 Medium: on a **True TF9, size 6** (a True ice boot, so the frame is riveted on where an
    ice holder goes), stock **4x72** wheels.
  - R1 Medium: on a **Tour roller boot, size 7**, a size bigger than his True, and True runs
    large ("go down a half size from Bauer and CCM", Coast Hockey Shop). So "the R1 feels big"
    mixes a bigger boot, a longer frame and taller wheels.
- Wheels set the shortest possible frame: front of the front wheel to back of the rear wheel is
  the sum of the four diameters plus three gaps. 76/76/80/80 = 312 mm + gaps, 72/72/76/76 =
  296 mm + gaps, 68/68/72/72 = 280 mm + gaps.

## 8. A shop's critique: "Why I don't use Marsblade" (Coast Hockey Shop)

https://coasthockeyshop.com/blogs/roller-inline-hockey-equipment-news-reviews/why-i-dont-use-marsblade-part-1
- Weight: "The frames still weigh 2-3x more than a high end CNC frame."
- O1: "heavy, unstable", the toe felt "like I was hitting a rock with my front wheel", no snap-back
  ("riding a teeter-totter"), unstable even with the bushings "cranked as tight as we could go".
- R1: much better: "I couldn't even feel the rocker", and he missed its "smooth motion" back on a
  rigid frame. Bigger wheels (80/76) = speed, stopping power.
- Durability: plastic parts at risk from puck shots indoors; frames "shredded" on rough asphalt.
- Sizing: Medium too small on an 8.0 True, overhanging on a 6.0; the lower is the same M to L,
  so larger sizes get a proportionally shorter wheelbase.

**Requirements this gives us**: no play and a clear snap-back to centre; all-metal where pucks and
asphalt hit; sized to the foot, not to a size band.

Jon's own experience differs on the O1: he has no complaints about his O1s (2026-11-10).

## 9. Other rocking frames: Sprung, and a TF9 + R1 build

- **Sprung**: wheels on two rocker arms with an adjustable spring between them (independent
  suspension, patents US 6,644,673 and US 6,454,280). The wheels move; the boot rides a rigid
  frame. Marsblade is the reverse: the boot rocks on a rigid wheel frame. No published weight found.
  **Jon has Sprungs on a pair of TF9s** (2026-11-10): "a good idea", but the bushings break (his
  have) and once they're old "it gets sketchy"; "they didnt do enough engineering work before
  manufacturing". This is the failure our coupling is designed against (PLAN, phase 4).
- **Coast Hockey Shop, "True TF9 skates + Marsblade R1 conversions"**
  (https://coasthockeyshop.com/blogs/roller-inline-hockey-equipment-news-reviews/true-tf9-skates-marsblade-r1-conversions):
  TF9 7W + R1 with 80/76 wheels = 1396 g a skate; Mako 8EE + Sprung = 1231 g and 1334 g. Turns and
  controlled stops: new Sprungs "A+", R1 "A", a flat chassis "C". R1 "more stable" with full stride
  extension. Riveted on with True-sized rivets, none loose. TF9 runs large: half a size down from
  Bauer/CCM. Author 6'2", 200+ lb.

## 10. Custom boots from scans (2026-11-10, first pass)

- **True custom skates**: a 3D foot scan, weight-bearing ("Most important, the scan is taken in a
  weight-bearing position"), plus photos of the arch and marked hot spots, sent to True. A custom 3D
  last from the scan; the carbon shell is built around it, in Canada, 3-4 weeks (retailer and True
  pages, e.g. https://iceboxhockey.com/pages/true-custom-skates,
  https://www.truetempersports.com/en-ca/golf/hockey/hockey-more-information/custom-skates.html).
- **Bauer patent US 12,135,869 B2** (Bauer Hockey LLC, filed 2022-09-19, granted 2024-11-05): scan a
  foot, build a 3D model, fit it to a customised last.
- **No 3D-printed hockey boot found.** Scan-to-print footwear exists: ATHOS climbing shoes (HP MJF),
  Superfeet FitStation insoles (MJF); Bauer + EOS and CCM + Carbon printed helmet liners.
- **Phone scanning accuracy**: smartphone photogrammetry of 12 casts, dimensional error 0.89 ±
  0.51 mm and volume error 0.89 ± 0.68% against a reference scanner (Ellis et al. 2026, via
  braceworks.ca). A depth-camera phone method, ~1.13 mm on feet (Kobayashi et al., 3DBODY.TECH
  2018). TrueDepth iPhone X vs Structure sensor on feet: "moderate to excellent reliability"
  (Footwear Science 2020, doi:10.1080/19424280.2020.1734867). Live feet move during capture; casts
  don't.

## 11. The scanner: Creality CR-Scan Otter (Jon's)

Retailer spec sheets (MatterHackers, 3DJake, B&H, 2026-11-10): handheld, four-lens stereo vision
with infrared structured light, 8 infrared LEDs for marker tracking; accuracy up to 0.02 mm (at a
60 mm working distance), 3D resolution 0.05-2 mm, up to 20 fps, working distance 110-1000 mm,
objects 10 mm to 2 m; exports OBJ, STL, PLY. No body-specific mode is listed. Some listings
disagree (one says laser, ±0.3 mm); the 0.05 mm figure is the Otter Lite. Infrared structured
light suits skin, but a live foot moves during a scan, and the sole can't be scanned while it
bears weight (PLAN, B1). Our own `check` on a known object sets the real error.

## 12. The stand-in foot scans: Schuster et al. 2024 (Dryad, CC0)

"Foot shape-function model data", Robert Schuster (Griffith University), Dryad, published
2023-12-11, doi:10.5061/dryad.g4f4qrfwt, **license CC0-1.0** (from the Dryad API). Paper: Schuster
RW, Cresswell AG, Kelly LA (2024), "Human foot form and function: variable and versatile, yet
sufficiently related to predict function from form", Proc R Soc B, doi:10.1098/rspb.2023.2543.
- 100 healthy participants (IDs 001-103; 021, 030, 064 absent); each one's dominant foot scanned
  bearing minimal weight (`mBW`) and full body weight (`fBW`). 200 PLY files of ~4.2-4.9 MB, named
  `<id>_<mBW|fBW>_<foot>.ply`; left-dominant feet were mirrored to match right feet ("Lr").
- `treadmill_data.mat` (281 MB) holds the participant table (sex, height, weight, leg dominance)
  and walking/running joint angles and moments. Scanner and units aren't stated (`scan` detects
  units).
- The Dryad API wants a token to download, and the site returns 403 to scripted requests, so the
  files are fetched by hand in a browser:
  https://datadryad.org/dataset/doi:10.5061/dryad.g4f4qrfwt

## Still to find

- How a hockey boot is built and what each part does: shell, toe cap, tendon guard, liner, tongue,
  footbed, eyelets; materials, wall thicknesses, weights; any puck-protection standard. Ice vs
  roller boots: what differs.
- Foot soft-tissue moduli for sim-soft, and plantar pressure-pain thresholds.
- How to scan a foot well with a handheld scanner (markers, stance, foam boxes, casts).

- Real one-piece dimensions: axle spacing, mount spacing, rail height and wall thickness (measure
  one, or find drawings).
- The R1's bushing: material, durometer, size; the exact radii of the four inserts; how far the
  rocker can travel.
- Skating loads beyond in-skate pressure: lateral and torsional loads on an inline frame (look for
  inline frame testing standards, e.g. EN 13843 / ASTM F1714 for inline skates).
- The full Footwear Science paper (forward vs backward, sample size).
