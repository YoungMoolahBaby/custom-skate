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
