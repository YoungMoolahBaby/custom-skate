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
  units: mm).
- Seen in the files (2026-11-10): ASCII PLY exported from Meshmixer, ~33-35k vertices with
  normals and colours; closed (the cut above the ankle is capped, flat on some scans and sagging
  on others); already aligned: x heel to toe, y across, z up, floor at z = 0, cut at z = 125 mm.
  Loaded (`fBW`) soles are flat at z = 0; lightly loaded (`mBW`) soles curve, and on some the toes
  lift off.
- The participant table is a MATLAB `table` inside a v7.3 (HDF5) file, which h5py can't decode
  (an MCOS object); the `mat-io` Python package reads it (`uv run --with mat-io --with pandas`).
  50 men, 50 women. Saved as `D:\custom-skate-data\info-clean.csv` (ID, M/F, height cm, weight kg,
  leg), next to the data, not in git.
- The Dryad API wants a token to download, and the site returns 403 to scripted requests, so the
  files are fetched by hand in a browser:
  https://datadryad.org/dataset/doi:10.5061/dryad.g4f4qrfwt

## 13. Loads on a skate frame (round 2, 2026-11-10)

What the strength checks (`src/strength.rs`) start from. No source gives frame design loads for
hockey yet, so the cases are assumptions built on these:
- **EN 13843:2009**, "Roller sports equipment - Inline skates - Safety requirements and test
  methods" (CEN). From the free preview of the 2003 edition (iTeh, SIST EN 13843:2003): users of
  20-100 kg (class A; class B up to 60 kg and feet up to 260 mm); the chassis (supports, axles,
  wheels, bearings, fixings) "shall withstand the impact loads, the constant load and the static
  load to which it is exposed during use", judged by function after the tests of clause 5. The
  2009 contents list a chassis-to-shoe attachment test and a vertical impact test. The load values
  are past the preview's last page (paid). **Used**: 100 kg, the top of class A.
- **Brock / George Brown poster** (section 1): in-skate peak forces up to 1196 N on ice (agility
  turns, fixed holder), from insoles, which miss shear.
- **van der Kruk et al., "Wireless instrumented klapskates for long-track speed skating"**,
  Sports Engineering 19 (2016) 273-281, doi:10.1007/s12283-016-0208-8 (TU Delft / VU). Force
  sensors between shoe and blade measure normal and lateral force in the skate's own frame. Their
  introduction cites an earlier single-skater study: peak lateral force 0.3-0.6 BW, 25-45% of the
  peak normal force (so the normal peak is near 1.3 BW). Speed skating, n = 1, read second-hand
  (abstract and excerpts; the full text is paywalled).
- **Stidwill (McGill thesis) and Stidwill, Pearsall, Dixon, Turcotte, "Force transducer system
  for measurement of ice hockey skating force"**: strain gauges on an ice hockey blade holder
  measured vertical and medial-lateral force in forward skating (11 male players, resolution
  1.9 N). The force numbers weren't in what we could read. The best next source for hockey.

**The cases used** (assumptions, `strength::LOADS`): 100 kg skater; stride 1.5 BW on one skate,
repeated (fatigue); impact 3 BW once (static); lateral / normal 0.45 in the skate's frame.

## 14. Marsblade i2: the ice holder's radii (2026-11-11)

- **Runner:** "The I2 steel comes delivered with an 11 ft radius and can be profiled and
  sharpened just like regular skate blades" (HockeyMonkey, i2 holder kit page,
  https://www.hockeymonkey.com/marsblade-hockey-skate-components-i2-holder-kit.html, read
  2026-11-11).
- **Rocker:** set by inserts in the holder, not by the runner. The kit has "Rocker Radius
  Inserts x 5 pairs" (same page).
- **The inserts' radii:** 14, 17, 20, 23 and 26 ft, with 23 ft fitted from the factory. This
  comes from a web search's summary of Marsblade's 2023 catalog (issuu, Interhockey AG); the
  catalog itself wouldn't load, so **not verified**. To be checked on Jon's inserts.
- **Jon's runners** (2026-11-11): the factory profile, sharpened with a 1/2" hollow. The hollow
  is the cross-section (bite against glide); it doesn't change the pitch.
- **What that means for pitch** (hand, GIH eq. 10, effective radius r s / (r + s - h), with the
  blade height h taken as 3 cm, which hardly matters: 2-4 cm moves it 0.01 ft):

  | Insert (r) on the 11 ft runner | Effective radius | Foot pitch per cm |
  |---|---|---|
  | 14 ft | 6.18 ft | 0.304 deg |
  | 17 ft | 6.70 ft | 0.280 deg |
  | 20 ft | 7.12 ft | 0.264 deg |
  | 23 ft (factory, if the catalog is right) | 7.46 ft | 0.252 deg |
  | 26 ft | 7.75 ft | 0.243 deg |

  The round-2 roller rocker (7.5 ft on a flat lower) gives 0.251 deg/cm, within 1% of the 23 ft
  insert.

## 15. Foot volume over a game: exercise and standing (2026-11-11)

For the fit before and after a game (Jon: "factor in the blood in feet, sweat in socks"). Found
by web search; **only abstracts or summaries read, not the full papers**, so the figures are
approximate:
- Short treadmill bouts (10 min): foot fluid volume up ~2% after walking and ~3% after running
  ([PubMed 8535472](https://pubmed.ncbi.nlm.nih.gov/8535472/); same group: "The effects of
  walking, running, and shoe size on foot volumetrics").
- A 30 min endurance run: foot volume up 2.2% in men; 30 min of standing: up 1.4% in women
  ([German Journal of Sports Medicine, 2009](https://www.germanjournalsportsmedicine.com/archive/archive-2009/heft-4/der-einsatz-von-koerperkomposition-und-sportlicher-aktivitaet-auf-die-fussdimension/)).
- Before and after exercise, a volleyball study found no significant change
  ([NAU, "Variations in foot volume before and after exercise"](https://experts.nau.edu/en/publications/variations-in-foot-volume-before-and-after-exercise/)).
- Within-day and between-day changes of foot shape, separated from load: Ballester et al.,
  Footwear Science 2019 ([UPV repository](https://riunet.upv.es/bitstream/10251/165795/3/Ballester%3bPIEROLA%3bSolves%20-%20Study%20on%20controllable%20and%20uncontrollable%20factors%20affectng%20foot%20shape.pdf));
  results not read yet. Moholkar & Fenelon (2001), "Diurnal variations in volume of the foot
  and ankle", is the older source; not retrieved.

So ~1.4-3% after standing or exercise, sometimes none. On stand-in 022 (1.05 L, 734 cm2 of
skin to the scan's top) that's 0.2-0.4 mm if spread evenly; in fact it gathers in the soft
tissue over the top and front of the foot.

**Sweat and friction** (Jon: "sweat in socks"):
- **Friction rises with moisture, about twofold wet.** Gerhardt et al. 2008, *J R Soc
  Interface* 5:1317 ([PMC2607440](https://pmc.ncbi.nlm.nih.gov/articles/PMC2607440), full text
  read): forearm skin on a plain-weave 50/50 cotton/polyester fabric, 14.8 N normal load,
  22 people. Median friction coefficient 0.42 for very dry skin, 0.53 (men) and 0.61 (women) for
  normally moist skin, and 0.88 +- 0.08 (men) and 0.95 +- 0.04 (women) against the fabric
  soaked through. Forearm, not foot; a sock on a liner is fabric on fabric, not measured here.
  A review cites a factor of 1.5-7 for moisture at the skin-textile interface (Ke et al. 2015,
  *Tribology Letters*, [RERO](https://doc.rero.ch/record/331602); secondhand there).
- **How much a foot sweats in exercise** (abstracts and a review; conditions vary widely):
  ~30 ml/h from one foot, peaking at ~50 ml/h, in exercise-induced hyperthermia (Taylor et al.
  2006, [PubMed 17042246](https://www.pubmed.gov/17042246/)); 68 ml/h from one foot in heated
  exercise, 22-33 ml/h at rest with a modest core temperature rise (review, *Eur J Appl
  Physiol* 2014, [PDF](https://springerlink.bibliotecabuap.elogim.com/content/pdf/10.1007/s00421-014-2940-8.pdf));
  most from the top of the foot and the medial ankle, least from the toes (West et al. 2019,
  *Ergonomics*, [link](https://informahealthcare.com/doi/full/10.1080/00140139.2019.1657185)).
  An older military march study weighed socks: 2.0-8.5 g/h a foot
  ([NLM](https://collections.nlm.nih.gov/ocr/nlm:nlmuid-101679654-bk), rough OCR).
- So over a game a sock can take on grams to tens of grams of sweat, and the friction going in
  and out roughly doubles from dry to soaked.


## 16. Skates Jon pointed to: CCM Jetspeed FT6 Pro, Bauer Konekt (2026-11-11)

Mostly retailer copy; no published dimensions or lab data found.
- **CCM Jetspeed FT6 Pro** (Jon's reference for a wide throat): a one-piece boot, "molded
  around a foot form before leaving the factory" and heat-mouldable; mid-stiffness composite,
  with a carbon insert and a carbon "flex tendon"; a replaceable tongue buckled into the boot
  (ADPT Comfort, FlexMotion), said to improve wrap and forward flex; an injected outsole; three
  widths (tapered, regular, wide); one owner: snug from the forefoot to the top of the ankle,
  the heel never moves. ([Inline Warehouse](https://www.inlinewarehouse.com/CCM_Jetspeed_FT6_Pro/descpage-FT6P.html),
  [Hockey World](https://www.hockeyworld.com/CCM-Jetspeed-FT6-Pro-Hockey-Skates-Sr),
  [Hockey Monkey](https://www.hockeymonkey.com/ccm-hockey-skates-jetspeed-ft6-pro-sr.html),
  [Ice Warehouse](https://www.icewarehouse.com:443/CCM_Jetspeed_FT6_Pro/descpage-FT6P.html)).
  Nothing verified on the throat's width or on any player's own pair.
- **Stiffness across the FT6 Pro's boot** (Jon: "if they have varying stiffness throughout
  their boot"): no zone-by-zone figures are published. A single boot rating, CCM's "stiffness
  performance index" 195 on most listings (180 on one); a one-piece composite boot whose sole
  wraps under it; a reviewer: "the rest of the boot is stiff apart from the tendon guard", and
  a few weeks' break-in. The give is put in parts, not soft shell zones: the carbon "flex
  tendon" guard for extension at the end of the stride (a reviewer: it still stops a skate to
  the heel but has "just enough give"), the removable buckled-in tongue for forward flex, and
  multi-density memory foam round the ankle. On the heel, only a roller Jetspeed review: a
  tighter heel pocket "to keep your feet locked in place for better energy transfer".
  ([Hockey World](https://www.hockeyworld.com/CCM-Jetspeed-FT6-Pro-Hockey-Skates-Sr),
  [Source for Sports](https://sourceforsports.ca/products/ccm-jetspeed-ft6-pro-senior-hockey-skates-2023),
  [Outside Bozeman (FT6)](https://outsidebozeman.com/gear/product-reviews/misc/review-ccm-jetspeed-ft6-skates),
  [Hockey World Blog, roller JetSpeed](https://hockeyworldblog.com/?p=19559)).
- **Bauer Konekt2 goalie skate:** a two-piece Grilamid shell "with a hinge on the ankle",
  up to 22° of forward flex (a limiter cuts it to 5-10°), upper and lower buckles, shell and
  liner heat-moulded ([The Hockey Shop review](https://www.thehockeyshop.com/blogs/the-hockey-blog/bauer-konekt2-goalie-skate-review)).
- **True's custom goalie boots** are a one-piece (monocoque) shell, and more NHL goalies are
  moving to them ([NHL.com](https://nhl.com/news/unmasked-evolving-skates-give-goalies-edge-304203896)).

## 17. Hockey helmets: Virginia Tech's ratings against price (2026-11-12)

Virginia Tech's Helmet Lab rates helmets by **Hockey STAR**, the concussions a player would be
expected to have in a season of hits wearing that helmet, so lower is better
([ratings](https://www.helmet.beam.vt.edu/hockey-helmet-ratings.html); test protocol Begonia et
al. 2020, [Hockey STAR Methodology](https://vtechworks.lib.vt.edu/items/53c334da-710f-4cd7-a163-1b5d884cb9b6/full);
update Rowson et al. 2022). A pendulum strikes a helmeted headform at the front, side, top and
back, at 3.1, 4.9 and 6.4 m/s, two helmets twice each (48 tests). Peak linear and rotational
acceleration go into a concussion risk function. The risks are weighted by how often each hit
happens to a player (227 head impacts a season on average) and summed. The risk function's
coefficients are not copied here yet.

The data, read from the ratings page's own data file (`js/hockeyData.js`, 2026-11-12): **75 adult
helmets** tested 2014-2026, each with the price at the time of its test. **Price against STAR
score: r = -0.17** (a dearer helmet scores slightly better on average; price explains ~3% of
the spread). Some tested since 2021:

| Helmet | Tested | Price then | Stars | STAR |
|---|---|---|---|---|
| Valor Axiom | 2023 | $299.99 | 5 | 0.43 |
| Bauer RE-AKT 155 | 2024 | $229.99 | 4 | 0.57 |
| Bauer Hyperlite 2 | 2023 | $399.99 | 4 | 0.65 |
| Bauer RE-AKT 85 | 2022 | $139.99 | 4 | 0.66 |
| Warrior Covert CF 100 | 2022 | $159.99 | 4 | 0.71 |
| CCM Tacks 920 | 2026 | $399.99 | 4 | 0.72 |
| Bauer RE-AKT 3D | 2026 | $459.99 | 3 | 0.92 |
| CCM Super Tacks X | 2021 | $399.99 | 1 | 2.65 |

Youth helmets have their own list (first ratings December 2025:
[VT News](https://news.vt.edu/articles/2025/12/research-youth-hockey-helmets.html)). Standards
for the sticker: ASTM F1045 (HECC certifies to it) and CSA Z262.1; CSA's 1990 edition introduced
a drop test with a 275 g limit; the current limits weren't checked. Printed custom liners exist
(Bauer with EOS, SLS: [3D Printing Industry](https://3dprintingindustry.com/news/bauer-and-eos-combine-to-produce-personalized-ice-hockey-helmet-inserts-220959/);
CCM with Carbon for the Super Tacks X: [3DPrint.com](https://3dprint.com/272687/nhl-will-now-use-3d-printed-helmets-thanks-to-carbon-and-ccm)).
Whether VT tested the Super Tacks X with the printed liner is not stated.

## 18. The stand-in leg: MakeHuman and ANSUR II (2026-11-12)

**MakeHuman** ([makehumancommunity/makehuman](https://github.com/makehumancommunity/makehuman),
`master`): the base mesh `makehuman/data/3dobjs/base.obj` and the targets (per-vertex offsets,
`index dx dy dz`) are **CC0** ("explicitly released as CC0 in september 2020", in each target
file; `LICENSE.ASSETS.md`); the program's code is AGPL and is not used here. Fetched to
`D:\custom-skate-data\makehuman\` (not in git): the base mesh, the three `*-male-young` macro
targets, the leg's measure targets (`measure-{ankle,calf,knee,thigh}-circ`,
`measure-{lowerleg,upperleg}-height`, each `-decr`/`-incr`), the right lower leg's scale and shape
targets, and, to read, `apps/humanmodifier.py` and the modifier JSON. Seen in `base.obj`: 19,158
vertices, the skin is group `body`, the rest are helpers (`helper-*`, eyes, teeth, tights) and
joint markers (`joint-*`, 8 vertices each). Units are decimetres, y up, z forward, +x the model's
left; the ground marker sits at y = -8.183, the base model is 16.9 dm tall. Right leg (x < 0):
ankle joint 73.5 mm above the ground, knee joint 449 mm.

Daz was the other option (Jon). Its licence text wasn't read; Daz forum posts say renders can be
published but the meshes, modified or not, can't be shared, and the viewer publishes meshes
([Daz forums](https://www.daz3d.com/forums/discussion/comment/489889/)).

**ANSUR II** (US Army anthropometric survey, 2012; public working data,
[Open Design Lab](https://www.openlab.psu.edu/ansur2/), file `ANSUR_II_MALE_Public.csv`, 4,082
men, lengths in mm; methods: Gordon et al. 2014, NATICK/TR-15/007). Fetched to
`D:\custom-skate-data\ansur2\`. The men 1700-1800 mm tall (2,222; mean 1750, stand-in 022 is a
175 cm man), percentiles 5 / 25 / 50 / 75 / 95, mm:

| Measure | p5 | p25 | median | p75 | p95 |
|---|---|---|---|---|---|
| ankle circumference (smallest, above the ankle bones) | 207 | 220 | 228 | 237 | 252 |
| calf circumference (largest) | 347 | 373 | 391 | 410 | 440 |
| knee height, mid-patella | 457 | 473 | 486 | 497 | 517 |
| tibial height (floor to the top of the shin bone) | 440 | 455 | 466 | 478 | 493 |
| lateral malleolus height (outer ankle bone) | 65 | 69 | 73 | 76 | 82 |
| lower thigh circumference | 361 | 388 | 408 | 427 | 460 |

Stand-in 022's outer ankle bone is at z = 78 (PLAN.md, the boot), within this range. ANSUR's 3D
scans are not public. The army is not the hockey population (fitter, younger), but nothing
better is open.

## 19. Shin pads: pucks, shots, injuries, pads, temperature (2026-11-12)

From a research pass on 2026-11-12. **R** rule or standard, **M** measured, **C** a maker's or
retailer's claim. Energies (½ m v²) are ours. Not yet checked against the full text: the
Pearsall 2001 speeds (its abstract gives none; the PDF couldn't be read here).

**Pucks.**
- Ice (R): vulcanised rubber, 25.4 mm thick, 76.2 mm across, **156-170 g** (IIHF Rule 13.1,
  2026-27; USA Hockey Rule 306: 5½-6 oz).
- Ice, cold (M): Bigford & Smith 2009, J ASTM Int 6(7), doi:10.1520/JAI101859 (Anderson/Bigford
  MS thesis, Washington State 2008). Two brands, 12 each, fired at 24.6 m/s into load cells:
  restitution 0.383 / 0.349 at 22 °C, 0.267 / 0.256 at -4 °C; dynamic stiffness up 626% / 488%
  from 22 to -4 °C, contact time halved. Pucks in play at two rinks were -11 to -4 °C. So a
  room-temperature puck is far softer than the one that hits you.
- Roller (R): Inline Hockey Australia (on World Skate), Rule 3.2.12: plastic, 77 mm across,
  22-30 mm thick (without/with pins), **120-130 g**. USARS 2021-22 Rule 13: 1" x 3", no mass; the
  IDS puck official. Retail (C): Green Biscuit Roller, IDS Pro Shot, Alkali RPD Quantum, Sonic
  3.6 oz (~102 g); Rocket 102 g; Stilmat 4.3 oz; Bauer Slivvver 4.4 oz. No roller puck
  stiffness data found.

**Shots** (M, stationary shots).
- Slap shots, recreational: 23.3 ± 3.8 m/s (10 players), varsity 30.0 ± 2.5 (10) (Pearsall et
  al. 2001, ISBS, [ojs](https://ojs.ub.uni-konstanz.de/cpa/article/view/3779)); 80.3 ± 11.6 km/h
  (22.3 m/s; 5 recreational) and 120 ± 18 km/h (4 elite) (Villaseñor et al. 2006, J Appl
  Biomech 22:202, doi:10.1123/jab.22.3.202).
- Pro and semi-pro on ice, radar: wrist 105.6 ± 10.0 km/h, slap 125.2 ± 12.6 km/h (20 players;
  Bežák & Přidal 2017, Acta Gymnica 47(2):78, doi:10.5507/ag.2017.007).
- NHL: hardest in-game shot of 2025 105.05 mph (NHL EDGE, [nhl.com](https://nhl.com/news/nhl-edge-stats-trends-and-storylines-for-2025)).
- No roller shot speeds found.
- **Energy:** a 165 g ice puck carries 44 J at 23 m/s and 74 J at 30 m/s; a 102 g roller puck
  27 J at 23 m/s and 46 J at 30 m/s, about 60% of the ice puck's at the same speed (125 g:
  76%).

**Injuries** (M).
- NCAA men 2014/15-2018/19: blocking a shot caused 11.1% of injuries; lower leg 2.3%, ankle
  6.8%, knee 9.2% of all (Boltz et al. 2021, J Athl Train 56(7):703, doi:10.4085/1062-6050-611-20).
- 27 elite players, 31 acute foot and ankle injuries (MRI): 17 direct blows, 15 from a moving
  puck, 6 fractures and 6 high-grade bone bruises, **14 of 17 on the inner side** (medial
  malleolus, navicular, first metatarsal); 10.6 games missed against 2.4 (Baker et al. 2016,
  AJSM 44(5):1317, doi:10.1177/0363546515626181). That is the boot's shield (PLAN item 7) and
  the pad's bottom edge on the inside.
- 30 NHL ankle fractures from public data: 36.7% "hit by puck" (Tham et al. 2025, J Orthop
  68:331, doi:10.1016/j.jor.2025.07.033).
- Where pucks land on the leg when blocking: not found.

**Impact tests and standards.**
- **No hockey shin-guard standard exists.** HECC's puck tests are for face and head: ASTM
  F1587 goalie masks 80 mph; ASTM F513 face protectors 63 / 74 mph; CSA Z262.2 visors
  ([hecc.org](https://hecc.org)). BNQ 9415-370 (neck guards) is blade cuts only.
- Football shin guards, EN 13061:2009: 1 kg striker, transmitted force ≤ 2.0 kN (R). Soccer,
  NOCSAE ND090: 5.0 kg sphere at 1.73 m/s at the centre (~7.5 J), mean ≤ 200 g (R).
- Hockey shin guards on a leg form (M): Fournier 2002, MHK thesis, Univ. Windsor,
  [hdl:20.500.14776/1759](https://hdl.handle.net/20.500.14776/1759). A 3.3 kg steel drop mass
  (contact ~10-15 mm across), 0.2 / 0.4 / 0.6 m (6.5 / 12.9 / 19.4 J), 14 adult guards (brands
  not named): mean peak force 559-1852 N at the tibia, 545-1342 N at the knee; many guards were
  damaged. Cadaver tibia fracture force 2927 ± 403 N (Francisco 2000, cited there). A drop, not
  a puck.
- Puck on a helmet (M): 160 g at 24.2 m/s (47 J), mean force 3425 N; at 33 m/s (87 J), 6175 N;
  contact 51-61 cm² (Ouckama & Pearsall 2014, IRCOBI IRC-14-16). The only puck-force data found.

**Pads today** (C, one pad, 14", weighed by Inline/Ice Warehouse, ±5%).

| Pad | g | | Pad | g |
|---|---|---|---|---|
| CCM Next | 481 | | Warrior Alpha LT | 605 |
| Warrior Rise | 492 | | CCM Tacks XR Pro | 675 |
| CCM JetSpeed FT8 Pro | 637 | | Warrior Alpha Pro | 673 |
| Sherwood Code Photon Pro | 699 | | CCM Tacks XR | 772 |
| CCM Tacks XF Pro | 788 | | Bauer Supreme Ultrasonic | 792 |

- Construction (C): a moulded shin cap and a separate knee cap, a foam liner (PE, HD foam;
  D3O in the calf and knee on some), a foam calf wrap, a knee strap and a calf strap (CCM Next,
  Tacks XR Pro; Bauer Supreme 1S: Curv composite shin cap).
- Sizing (C): Bauer and CCM: centre of the knee cap to the top of the boot. Ice Warehouse:
  seated at 90°, bottom of the knee cap to the top of the foot over the tongue, **1" less if
  worn under the tongue**; 15" for 5'7"-5'11", 16" for 5'10"-6'2".
- "Undersized" pads: no study found; retailers say a short pad leaves a gap above the skate and
  may miss the knee, and slim lower sections "trade some lower-leg coverage for mobility".

**Temperature.**
- Ice rinks (R): game air +8 to +10 °C at 1 m, ice -4 to -5 °C (IIHF Ice Arena Guide 2024,
  Table 4.1). NHL 60-64 °F air (secondary reports).
- Pad materials (M): back protectors, 50 J, 45 mm flat striker, 20 vs -5 °C (Nicotra et al. 2014,
  Procedia Eng 72:678): three EVA-blend foams 5301 → 6292 N, 5728 → 11196 N, 8644 → 5220 N;
  a PU foam with PDMS 5549 → 15534 N (time to peak 4.3 → 0.8 ms); a hard PP shell over
  polyolefin foam 15537 → 7644 N, its shell cracked at -5 °C. Cold can double or treble what a
  foam passes on, and differs by foam.
- D3O LP1 (C, data sheet bar chart, EN 1621-1): ~16 kN ambient, ~18 kN at -10 °C, ~23 kN at
  +40 °C: for that part, heat is worse than cold.
- Not found: roller playing temperatures; EVA, PE, TPU or Poron XRD impact against
  temperature from 5 to 35 °C.

## 20. Puck impacts, foams, shells, the shin: numbers for D3 (2026-11-12)

From a research pass on 2026-11-12. **M** measured, **C** a maker's claim, "read" read off a
graph. Sources' PDFs are kept locally, not in git.

**The puck on a rigid plate** (Anderson (Bigford) 2008, MS thesis, Washington State Univ,
[hdl:2376/102825](https://hdl.handle.net/2376/102825), full text; journal version Bigford & Smith
2009, J ASTM Int 6(7), doi:10.1520/JAI101859). Air cannon, the puck flying **rim first** into a
rigid plate on three load cells (100 kHz); speed by light gates; two unnamed brands, PA and PB,
12 pucks each; 164 g used. Dynamic stiffness k_d = (1/m)(F_peak/V)^2.

| At 24.6 m/s (55 mph) | restitution | peak force | contact | k_d |
|---|---|---|---|---|
| 22 °C, PA (M) | 0.383 | ~14.2 kN (read) | 0.81 ms | ~1.9 MN/m (read) |
| 22 °C, PB (M) | 0.349 | ~12 kN (read) | | ~1.3 MN/m |
| -4 °C, PA (M) | 0.267 | ~37 kN, 3 pucks (read) | 0.41 ms | ~13.7 MN/m |
| -4 °C, PB (M) | 0.256 | | | ~7.5 MN/m |

- Pucks in play at rinks: -11 to -4 °C. Peak force against temperature (PA, read): 14.7 kN at 22
  °C, 19.3 at 13, 24.7 at 8, 27.8 at 4, 30.2 at 1, ~37 at -4 to -2.
- Speed (PA, 22 °C, read): 24.6 / 29.1 / 33.5 / 38.0 m/s gave restitution 0.39 / 0.375 / 0.358 /
  0.34, peaks ~13.6 / 17.8 / 20.2 / 25.4 kN, and the same **contact time (0.84 ms) at every
  speed**. Caveats: one of the thesis's figures contradicts another on these peaks (the one
  consistent with its k_d is used); its displacement axes carry an integration offset.
- Kays 2013 (MS thesis, same lab), 15.6 m/s, 22 °C: restitution 0.427, peak 8.46 kN, contact
  0.939 ms (M); an LS-DYNA viscoelastic puck tuned to it; no cold set exists.
- Puck material: Shore A 85-89, density 1.34-1.43 g/cm³, 157-168 g (US 5,330,184, patentee's
  tests). No roller puck stiffness or restitution found.

**Pucks on protective gear** (M): Ouckama & Pearsall 2014 (IRCOBI IRC-14-16), a 160 g puck at
room temperature on helmets on a Hybrid III head: 24.2 m/s (47 J) gave a mean net force of
3425 N, 32.9 m/s (87 J) 6175 N, over 51-61 cm², under 2 ms. Hart 2015 (MSc, U Manitoba): bare
composite tibias (Sawbones, 4th gen., n = 8) struck by a 165 g puck broke at 30.4 ± 0.8 m/s
(76 ± 4 J), after 1-4 hits.

**Foams** (room temperature unless stated):
- Poron XRD (M, Bhagavathula et al. 2022, Exp Mech 62:505, doi:10.1007/s11340-021-00772-z): at
  195 / 244 / 404 kg/m³, modulus 0.77 / 1.73 / 4.37 MPa at 1 /s, 3.6 / 9.3 / 25 at 175-250 /s,
  14 / 24 / 92 at 3300-4600 /s. Rate matters by 15-30x.
- EVA 80-180 kg/m³ (M, Chen 2023, Polymers 16:34): modulus 2.8-7.0 MPa, quasi-static.
- An EVA/NBR blend pad (M, Nicotra 2014; Signetti 2019, JSAMS 22:S65): 50 Hz modulus 6.4-18 MPa at
  20 °C, 2-5x stiffer at -5 °C; densification strain 0.61-0.69; a 50 J drop passed 5.7 kN at 20
  °C and 11.2 kN at -5 °C.
- No EVA or PE data at impact strain rates were found.

**Shells:** HP PA 12 (MJF), C: modulus 1800 MPa, strength 48 MPa, elongation 20% (XY), notched
Izod 3.5 kJ/m², 1.01 g/cm³. Sintered PA 12 at 128-456 /s (M, Xu 2023, Mater Des 232:112162):
strength 54-61 MPa against 45-48 quasi-static, fracture strain down from 22-33% to 3-10%. No
public low-temperature data for printed PA 12. Shin guard shells in patents: nylon, PC, PE,
HDPE, PP; Bauer's FlyPro uses Curv self-reinforced PP (C: 4.2 GPa).

**The shin** (M): soft tissue over the mid-tibia 2-7 mm (Pejović-Milić 2002; Zhang 2021, 2022).
Fracture references: Kress & Porta 2001, 5.9-7.2 kN; an FE human model 1.6-2.6 kN (Shakib et
al.); the often-quoted "2927 ± 403 N" (Francisco 2000) is a synthetic tibia's end force, its
context unconfirmed. Thresholds: bruise, 50% risk at 3.2 J (Desmoulin & Anderson 2007,
abstract); football shin guards (EN 13061) ≤ 2 kN for a ~2 J impact; cricket leg guards (BS
6183-3) 4-6 kN.

## 21. The body for the girdle: ANSUR II and MakeHuman (2026-11-12)

ANSUR II (as in 18: `ANSUR_II_MALE_Public.csv`, men 1700-1800 mm tall, n = 2,222), mm:

| Measure | p5 | median | p95 |
|---|---|---|---|
| waist circumference (at the navel) | 770 | 938 | 1124 |
| waist height (navel) | 1003 | 1052 | 1099 |
| buttock circumference | 903 | 1016 | 1140 |
| buttock height | 833 | 882 | 940 |
| crotch height | 796 | 842 | 890 |
| thigh circumference (at the gluteal furrow) | 534 | 625 | 717 |
| lower thigh circumference | 361 | 408 | 460 |
| trochanterion height (the bony point of the hip) | 848 | 896 | 951 |
| iliocristale height (the top of the pelvis) | 1009 | 1058 | 1105 |
| hip breadth | 310 | 344 | 385 |
| waist breadth / depth | 272 / 188 | 326 / 234 | 382 / 299 |
| buttock depth | 205 | 246 | 289 |

MakeHuman (as in 18; CC0): also fetched `measure-hips-circ` and `measure-waist-circ` (each
`-decr`/`-incr`), and the body-weight targets `macrodetails/universal-male-young-averagemuscle-
{minweight,maxweight}.target`, to `D:\custom-skate-data\makehuman\`.

## 22. Skin sliding on the bone: soft tissue artefact (2026-11-12)

For G3: posing moves skin with the bones under it, and a girdle or pad worn on the skin can move
further than that.
- **Peters, Galna, Sangeux, Morris & Baker (2010)**, "Quantification of soft tissue artifact in
  lower limb human motion analysis: a systematic review", *Gait & Posture* 31(1):1-8
  ([Salford repository](https://salford-repository.worktribe.com/output/1452613/quantification-of-soft-tissue-artifact-in-lower-limb-human-motion-analysis-a-systematic-review)).
  Its abstract: skin markers moved relative to the bone by **more than 30 mm on the thigh** and
  **up to 15 mm on the tibia**. Read from the abstract only; the per-study values and the tasks
  behind the 30 mm are not checked here.
- Limits: these are markers stuck to the skin during motion capture, not garments. A girdle's
  shorts and a strapped pad move with the skin and can also slide on it. These numbers say how
  far the skin alone goes, not how far gear goes.

## 23. The skating stance's knee angle (2026-11-13)

For G3, round 2: the shin pad's knee shield built as a cup on the knee bent as in skating.
- **Lockwood et al. (2023)** (section 1, the Brock / George Brown poster): treadmill forward
  skating, n = 11 male collegiate players, Xsens IMUs. Hip / knee / ankle angle **59.57 / 65.54
  / 12.67°** (fixed holder; 59.87 / 65.07 / 12.50 dynamic). The poster doesn't say at which
  instant of the stride these are taken, or whether they are means over it.
- **Yu, Bi, Wang, Qin, Song & Wu (2025)**, "What are the differences between on-ice and off-ice
  side-cutting maneuver? A kinematic and electromyographic comparative analysis of ice hockey
  players", *Frontiers in Bioengineering and Biotechnology*, doi 10.3389/fbioe.2025.1692676
  (open access). 20 elite male players, 18.0 ± 0.7 years. Table 2, peaks: knee flexion
  **75.18 ± 6.21°** on ice (63.22 ± 4.81 off ice), hip flexion **65.83 ± 11.48°** on ice
  (43.39 ± 3.51 off). A side-cut, not the forward stride, and peaks, not a typical angle.
- **Buckeridge, LeVangie, Stetter, Nigg & Nigg (2015)**, "An On-Ice Measurement Approach to
  Analyse the Biomechanics of Ice Hockey Skating", *PLoS ONE* 10(5): e0127324. 9 high- and
  9 low-calibre players on ice, electrogoniometers at the hip and knee. Its knee angles are
  curves only (Figs 7-8, knee flexion positive); the text gives no degrees. In steady strides
  the knee is more flexed at ice contact than in acceleration strides.
- Not used: a coaching page's "90-110° for elite skaters, 120-140° for average" (included angles,
  so 70-90° and 40-60° of flexion), credited to a University of Calgary lab without a citation.
- **So the cup is built at 70° of knee flexion and 60° at the hip:** between the stride's 65.5
  and the cut's peak of 75. The hip doesn't shape the knee in the model (the skin there moves
  with the thigh and shin bones only).

## 24. How the kneecap moves (2026-11-13)

For the knee cup's hinges (G3): the cap rides the kneecap, so its turn on the shin shell and on
the thigh is the patella's.
- **Amis, Senavongse & Bull (2006)**, "Patellofemoral kinematics during knee flexion-extension:
  an in vitro study", *Journal of Orthopaedic Research* (December 2006). Eight fresh-frozen
  cadaver knees, the quadriceps loaded. "As the knee flexed, the patella flexed by 0.7 times the
  tibiofemoral flexion angle" (relative to the femur, so 0.3 times relative to the tibia). It
  moved medially 4 mm at 20° to engage the trochlear groove, then laterally to 7 mm by 90°.
  Read from the abstract (via read.qxmd.com); cadavers, not living knees.
- **Li, Papannagari, Nha, DeFrate, Gill & Rubash (2007)**, "The coupled motion of the femur and
  patella during in vivo weightbearing knee flexion", *Journal of Biomechanical Engineering*
  129(6):937-43 (PMID 18067400). Ten living knees, dual fluoroscopy and MRI, weight-bearing
  flexion from full extension to the most each could bend, the patella measured relative to the
  tibia: "Knee flexion was strongly correlated with patellar flexion (R(2)=0.91)". The slope
  isn't in the abstract, so no living-knee ratio to set against the cadavers' 0.7.
- **Iranpour, Merican, Baena, Cobb & Amis (2010)**, "Patellofemoral joint kinematics: the
  circular path of the patella around the trochlear axis", *Journal of Orthopaedic Research*
  28(5):589-94 (PMID 19950364). Seven cadaver knees, the quadriceps loaded, CT and an optical
  tracker: "The path of the center of the patella was circular and uniplanar (root-mean-square
  error 0.3 mm) above 16+/-3 degrees (mean+/-SD) knee flexion", the circle within 1-3° of square
  to the epicondylar and trochlear axes. Read from the abstract (PubMed).
- **Janani, Lakshmi, Prakash et al. (2023)**, "Preoperative templating of bone-patellar
  tendon-bone graft for anterior cruciate ligament reconstruction: a morphometry-based graft
  harvest method", *Clinics in Orthopedic Surgery* 15(3):410-417 (PMID 37274507). MRI of 1,002
  knees, 18-50 years, 712 men and 290 women, in Chennai: patella length 40.3 mm, width 40.2,
  thickness 18.6; patellar tendon length 45.2 mm (Insall-Salvati ratio 1.13). Means for both
  sexes together, from the abstract; one population, not split by sex.
- Not found: how thick the skin and fat over the kneecap are in a lean adult (searched PubMed and
  the web, 2026-11-13: studies measure prepatellar fat in knee replacement patients, most of them
  heavy). The body model takes 5 mm and checks what 3 or 8 would change.
- **MakeHuman's body has no kneecap.** Its skin over the knee blends the thigh's and shin's
  motions. Fitted to that skin, the knee shield turned about 21° on the thigh and 53° on the shin
  as the knee straightened from 70° (G3, round 2): it follows the thigh, where the real patella,
  by the ratio above, turns 49° on the thigh and 21° on the shin. **The body now has one**
  (`pose::Patella`, 2026-11-13), a bone on the thigh built from the three sources above: it turns
  0.7 times the knee's turn (Amis); its centre stays on a circle about the knee's axis (Iranpour;
  the model's axis is MakeHuman's knee joint, not the trochlea's); its tendon keeps its length to
  the tibial tuberosity (Janani's sizes, placed under the skin from ANSUR's mid-patella height).
  The tendon makes its centre go further round the knee than it turns: 1.03-1.10° a degree of
  the knee's (the `body` run). Not checked against a measured path in a living knee.

## 25. Foams that go in a washing machine (2026-11-13)

For the compression suit (G3, round 4): the knee's foam and the girdle's pads in it have to be
machine washable (Jon's requirement, PLAN.md, Open questions).
- **Rogers Corporation** (the maker of PORON), Dave Sherman, "Foam Properties: What Makes One
  Foam Different From Another?", PORON Cushioning blog, 21 August 2018: "If your application
  requires lighter weight and washability, opt for a closed cell foam." Closed-cell foams ("or
  EVA foams") are sealed bubbles of air; "padding made solely from closed cell foam become[s]
  less comfortable over time" as the air leaks and the foam goes flat. "All PORON Comfort
  materials are open cell polyurethane foams." A maker's blog, not a test.
- **Biomechanical Services** (an orthotics lab), "Foam Utilization" (undated, site © 2011-19):
  closed-cell polyolefin foam (their Polyfoam, Plastazote) "is lightweight, easy to form, with
  excellent washable surfaces"; open-cell foam (PORON) "is subject to break down, as cell walls
  break apart".
- Not found: wash-cycle test data for any of these foams (how many washes, at what temperature,
  before they shrink, take a set or come apart). That needs a maker's datasheet for the foam
  chosen. An Italian distributor (Gierre Gomma, "PORON urethanes") describes PORON's
  polyurethanes, the XRD impact foam among them, as "cellule calibrate semichiuse" (semi-closed
  cells), 240-480 kg/m³, sealing well against water; nothing on washing.

## Still to find

- How far gear shifts on the leg in play (G3's margin): a girdle's shorts riding down the thigh,
  a shin pad moving on its straps under a sock or pants.
- The knee's flexion through a forward stride, as numbers (contact, glide, push-off): theses from
  Pearsall's lab at McGill are the likely source (their pages moved to mcgill.scholaris.ca).
- How modern shin pads join the knee cap to the shin shell and the top to the cap, and how much
  those joints give (G3, round 3: hinged, the cap turns up to 52° on the shell and the top up to
  44° on the cap, and how far they give depends on how stiff they are; PLAN.md, G3).

- Frame design loads for hockey: Stidwill's thesis numbers; the EN 13843 test loads (paid).
- How a hockey boot is built and what each part does: shell, toe cap, tendon guard, liner, tongue,
  footbed, eyelets; materials, wall thicknesses, weights; any puck-protection standard. Ice vs
  roller boots: what differs.
- Foot soft-tissue moduli for sim-soft, and plantar pressure-pain thresholds.
- How to scan a foot well with a handheld scanner (markers, stance, foam boxes, casts).

- Real one-piece dimensions: axle spacing, mount spacing, rail height and wall thickness (measure
  one, or find drawings).
- The R1's bushing: material, durometer, size; the exact radii of the four inserts; how far the
  rocker can travel.
- Torsional loads on an inline frame, and ASTM F1714 if it has frame tests.
- The full Footwear Science paper (forward vs backward, sample size).
