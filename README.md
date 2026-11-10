# custom-skate

Two custom hockey skates, one for ice and one for roller, on boots made from a 3D scan of the
skater's feet. The roller skate rides an aluminium chassis that rocks like Marsblade's but weighs
the same as a premium one-piece chassis; boot and chassis are designed together, so the boot's sole
is the part that rocks. The ice skate takes a dynamic (rocking) blade holder. Modelled with
[CortenForge](https://crates.io/crates/cortenforge). The chassis came first, so most of this page
is about it; the boot plan is in `docs/PLAN.md` (track B).

A Marsblade-style frame is split in two. A rounded upper chassis carries the boot and rocks
heel-to-toe on a flat lower chassis that carries the wheels, held together by a sprung coupling.
The skater's foot can pitch the way it does on an ice blade while all four wheels stay on the
floor. The catch is weight. The two-piece frame is far heavier:

| Chassis | Weight | Source |
|---|---|---|
| Hi-Lo Kryptonium, 7000-series one-piece | 175 g (S/M), 184 g (L), with axles and spacers | Coast Hockey Shop |
| Rink Rat Rinkster Sniper Hi-Lo, 7075-T6 one-piece | ~170 g (MD) | Inline Warehouse |
| Konixx Bolt Hi-Lo, one-piece | 173 g (MD) | Ice Warehouse |
| Marsblade R1 (glass-fibre nylon upper, Al-Mg lower) | 282 g without wheels and bearings / 302 g (MD) | Hockey World Blog / Inline Warehouse |
| Marsblade O1 (resin) | 376 g (MD) | Inline Warehouse |

**The goal is a rocking chassis in the 170-185 g class: about 110-130 g lighter than the R1.**

## Why rock? The science

1. **The geometry: Daggfeldt & Petré (2025)**, Swedish School of Sport and Health Sciences (GIH),
   *Proc IMechE Part P*, [doi:10.1177/17543371251389073](https://doi.org/10.1177/17543371251389073).
   On ice, the foot can pitch only as far as the blade rolls. Moving the contact point 30 cm on a
   2.7 m (9 ft) blade gives about 6.4°. A smaller blade radius would give more pitch, but it
   shortens the contact with the ice and raises plowing resistance. A rocker between the holder and
   the blade separates the two: pitch is set by the rocker radius, glide by the blade radius.
   Their equations are in `src/pitch.rs`, checked against the paper's own numbers.
2. **The Canadian study: Lockwood, Dunne, Hsu (Brock University) and Ellingson (George Brown
   College), 2023**, "Effect of a Fixed versus Dynamic Pitch on Hockey-Specific Skating Skills"
   ([poster PDF](https://images.marsblade.com/uploads/2023/12/poster_spin-2023_fixed-versus-dynamic.pdf)).
   11 collegiate players skated on a treadmill and on ice, wearing IMU motion capture and pressure
   insoles. On Marsblade's i2 dynamic holder, the in-skate forces were lower at the same speed:
   forward skating on the treadmill had 5% lower mean force, 8% lower peak force and 8% lower
   impulse. C-cuts and backward skating had longer strides and larger joint angles.
3. **The sprint study: Psilander, Arndt & Petré (2025)**, *Footwear Science*,
   [doi:10.1080/19424280.2025.2563837](https://doi.org/10.1080/19424280.2025.2563837): elite
   players skated backward 1-2% faster on the dynamic holder.

**What this means for roller.** A one-piece roller chassis is a flat, rigid line of wheels. The
foot can pitch only by tipping the whole frame onto its end wheels. A rocker on a flat lower
chassis is the paper's model with a flat "blade" (s → ∞): the foot pitches x / r while every wheel
stays down. From `cargo run --release -- pitch`:

| | deg of foot pitch per cm of weight shift |
|---|---|
| Ice blade, 9 / 11 / 13 ft | 0.209 / 0.171 / 0.145 |
| Roller rocker, 5 / 7.5 / 10 / 15 ft (the R1's inserts span 5-15 ft) | 0.376 / 0.251 / 0.188 / 0.125 |
| One-piece roller chassis | 0 until it tips |

So the R1's rocker range brackets an ice blade's. The rocker gives a roller skate an ice
skate's pitch back.

**Limits of the evidence.** None of these studies tested a roller chassis; all three tested the
i2 ice holder, which uses the same rocker. The Brock / George Brown study is a conference poster
hosted by Marsblade. Its force numbers come from insoles inside the boot, not from the blade on
the ice, and the poster marks none of the force differences as significant. The GIH sprint study
found a gain only in backward skating. Details and numbers are in `docs/RESEARCH.md`.

## Run

```
cargo run --release -- check         # every tool used, against known results (15/15)
cargo run --release -- pitch         # the pitch tables above
cargo run --release -- scan <file>   # measure a foot scan (OBJ, STL, PLY)
```

Builds go to `D:/cargo-target/custom-skate` (`.cargo/config.toml`, local only). The first
release build takes about 3 minutes.

## Status

Phase 0 (the science, the pitch model, the mass tool checked) is done. See `docs/PLAN.md` for
what comes next and `docs/CORTENFORGE.md` for what CortenForge did well and where it fell short.

## Patents

Marsblade's split rocking frame is patented: US 9,782,665 B2 (Flow Motion Technology AB, active;
Google Patents estimates expiry 2031-02-02). Bauer holds a scan-to-last patent, US 12,135,869 B2.
Modelling and studying them is research. Check the patents before making anything to sell.

## Privacy

Foot scans and photos stay out of git (`scans/` and `measure/` are ignored). Only measurements
written into the docs are published.

## License

Code and docs: MIT or Apache-2.0, at your option (`LICENSE-MIT`, `LICENSE-APACHE`).
