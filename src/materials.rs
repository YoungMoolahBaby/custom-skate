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

/// TPU for a flexible part (the tendon guard): an assumption until a grade is chosen.
pub const TPU: Material = Material {
    name: "TPU (assumed)",
    density: 1200.0,
    source: "assumed 1.2 g/cm3, typical for 90-95A thermoplastic polyurethanes; replace with a chosen grade",
};

/// A hockey lace, braided polyester with air in the braid: an assumption.
pub const LACE: Material = Material {
    name: "lace (assumed)",
    density: 800.0,
    source: "assumed 0.8 g/cm3 for a braided polyester lace (solid polyester ~1.38); weigh a real lace",
};

/// Titanium Ti-6Al-4V, for bolts.
pub const STRAP: Material = Material {
    name: "elastic strap (assumed)",
    density: 800.0,
    source: "assumed 0.8 g/cm3 for elastic webbing with hook-and-loop (like the lace); weigh a real strap",
};

pub const FABRIC: Material = Material {
    name: "compression fabric (assumed)",
    density: 250.0,
    source: "assumed 250 g/m2 for a 1 mm compression knit (polyester-elastane), as 0.25 g/cm3 over 1 mm; weigh a real one",
};

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

/// Mechanical properties for the strength checks (MPa, except `nu`). `fatigue` is the
/// fully reversed (R = -1) fatigue strength at `fatigue_cycles`, unnotched.
#[derive(Debug, Clone, Copy)]
pub struct Strength {
    pub name: &'static str,
    pub e: f64,
    pub nu: f64,
    pub yield_: f64,
    pub uts: f64,
    pub fatigue: f64,
    pub fatigue_cycles: f64,
    pub source: &'static str,
}

/// 7075-T6 plate/bar.
pub const AL_7075_T6_STRENGTH: Strength = Strength {
    name: "7075-T6",
    e: 71_700.0,
    nu: 0.33,
    yield_: 503.0,
    uts: 572.0,
    fatigue: 159.0,
    fatigue_cycles: 5e8,
    source: "ASM Aerospace Specification Metals / MatWeb, 7075-T6: E 71.7 GPa, nu 0.33, yield 503 MPa, UTS 572 MPa, fatigue strength 159 MPa at 5e8 cycles (R. R. Moore, completely reversed)",
};

/// Ti-6Al-4V (grade 5, annealed), bolts.
pub const TI_6AL_4V_STRENGTH: Strength = Strength {
    name: "Ti-6Al-4V",
    e: 113_800.0,
    nu: 0.342,
    yield_: 880.0,
    uts: 950.0,
    fatigue: 510.0,
    fatigue_cycles: 1e7,
    source: "ASM Aerospace Specification Metals / MatWeb, Ti-6Al-4V grade 5 annealed: E 113.8 GPa, yield 880 MPa, UTS 950 MPa, fatigue 510 MPa unnotched at 1e7",
};

/// Stainless steel for an ice runner: an assumption until a grade is chosen.
pub const STAINLESS: Material = Material {
    name: "stainless steel (assumed)",
    density: 7750.0,
    source: "assumed 7.75 g/cm3, typical of the martensitic stainless steels used for skate runners (7.7-7.8); replace with the chosen grade",
};
