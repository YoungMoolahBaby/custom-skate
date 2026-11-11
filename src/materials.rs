//! Materials. Only densities are used so far; each has its source.

/// A material with the source of its numbers.
#[derive(Debug, Clone, Copy)]
pub struct Material {
    pub name: &'static str,
    /// kg/m^3
    pub density: f64,
    pub source: &'static str,
}

/// 7075-T6, the premium one-piece chassis alloy (Kryptonium, Rinkster Sniper).
pub const AL_7075_T6: Material = Material {
    name: "7075-T6",
    density: 2810.0,
    source: "ASM Aerospace Specification Metals / MatWeb, 7075-T6: 2.81 g/cc",
};

/// 6061-T6, the common extrusion/CNC alloy.
pub const AL_6061_T6: Material = Material {
    name: "6061-T6",
    density: 2700.0,
    source: "ASM Aerospace Specification Metals / MatWeb, 6061-T6: 2.70 g/cc",
};

/// Printed nylon for a boot shell (HP Multi Jet Fusion).
pub const PA12_MJF: Material = Material {
    name: "PA 12 (MJF)",
    density: 1010.0,
    source: "HP 3D High Reusability PA 12 datasheet: part density 1.01 g/cm3",
};

/// Carbon/epoxy laminate, the usual hockey boot shell.
pub const CFRP: Material = Material {
    name: "carbon/epoxy",
    density: 1550.0,
    source: "typical carbon/epoxy laminate at ~60% fibre volume, 1.5-1.6 g/cm3 (an assumption, not a datasheet)",
};

/// Liner foam: an assumption until a real liner is chosen.
pub const LINER_FOAM: Material = Material {
    name: "liner foam (assumed)",
    density: 150.0,
    source: "assumed 0.15 g/cm3, mid-range for EVA / PE skate-liner foams; replace with a real liner",
};

/// Titanium Ti-6Al-4V, for bolts.
pub const TI_6AL_4V: Material = Material {
    name: "Ti-6Al-4V",
    density: 4430.0,
    source: "ASM Aerospace Specification Metals / MatWeb, Ti-6Al-4V: 4.43 g/cc",
};

/// Polyurethane elastomer for the coupling bushing.
pub const PU_BUSHING: Material = Material {
    name: "polyurethane (assumed)",
    density: 1200.0,
    source: "assumed 1.2 g/cm3, typical cast polyurethane elastomers 1.1-1.25",
};
