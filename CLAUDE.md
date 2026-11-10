# custom-skate

Two custom skates for Jon, one ice and one roller, on boots from a 3D scan of his feet (Creality
CR-Scan Otter). The roller skate rides a Marsblade-style rocking aluminium chassis as light as a
one-piece chassis. Modelled with CortenForge. Public on GitHub. Read first:
- `README.md`: the goal, the science, the weight targets.
- `docs/PLAN.md`: phases and status. **Start here.**
- `docs/RESEARCH.md`: the sources, with their numbers and their limits.
- `docs/CORTENFORGE.md`: end-user findings for CortenForge. Add to it while building.
- `docs/MEASURE.md`: measurements from Jon's own gear (his skates, his Marsblade).

## Rules

- Use CortenForge only as an end user would: the published `cortenforge` 0.9 crates and their
  sources/docs under `~/.cargo/registry/src/*/cortenforge-*-0.9.0/`. Don't use its git repo.
- CortenForge is the focus. Other crates only render, display and wire things up. An outside
  solver is allowed as a check on CortenForge, not instead of it.
- Check physics against known results (textbook formulas, the papers' own numbers) in `check`
  before building on it. Every number in the docs comes from a real run or a cited source.
- Log friction in `docs/CORTENFORGE.md` under its sub-crate: type (bug, docs, API, perf, setup,
  works), date, file:line. Add to its **Suggestions** list whenever the project wants a missing
  feature, with what we built instead.
- Weights are compared like for like: same size (MD unless stated), and say what is included
  (axles, spacers, hardware).
- The repo is public. Foot scans and photos are personal: keep them in `scans/` and `measure/`
  (gitignored). Publish one only if Jon says so.
- Keep Jon's own experience separate from reviewers' opinions in the docs.

## Working with Jon

- Commit only when he says "commit"; push only when he says "push".
- Ask before downloads over 1 GB and before deleting anything.
- C: is nearly full: build output lives on D: (`.cargo/config.toml`, gitignored).

## Build notes

- Windows 11, RTX 4070 Ti, Ryzen 5 5600. Always run in release: `cargo run --release -- <mode>`.
