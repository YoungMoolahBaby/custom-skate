//! Skate pitch geometry.
//!
//! Daggfeldt K, Petré H (2025). "A mathematical model of dynamic skate blade holders".
//! Proc IMechE Part P: J Sports Engineering and Technology. doi:10.1177/17543371251389073.
//!
//! Symbols follow the paper. Lengths in metres, angles in radians.
//! - `theta`: the skate's built-in pitch (a constant offset)
//! - `r`: rocker radius (dynamic holder); in eq. 1 the blade radius of a fixed holder
//! - `s`: blade radius of curvature
//! - `h`: blade height, from the ice up to the straight top edge the rocker rolls on
//! - `alpha`: rocker rotation, `beta`: blade rotation, `phi = theta + alpha + beta`: foot pitch
//! - `y`: how far the blade's contact point with the ice has moved
//!
//! The paper models an ice holder (Marsblade i2). A roller chassis is the same model with
//! a flat "blade": the lower chassis stands on its wheels and does not rotate (s -> inf),
//! so the foot pitch is `theta + x / r`, where `x` is how far the rocker's contact point has
//! moved along the lower chassis. See [`roller_alpha`].

/// A dynamic holder: a rocker of radius `r` rolling on a blade of radius `s` and height `h`.
#[derive(Debug, Clone, Copy)]
pub struct Rocker {
    pub r: f64,
    pub s: f64,
    pub h: f64,
    pub theta: f64,
}

impl Rocker {
    /// Eq. 3: the blade's equilibrium rotation for a rocker rotation `alpha`.
    pub fn beta_of_alpha(&self, alpha: f64) -> f64 {
        (alpha * self.r / (self.s - self.h)).atan()
    }

    /// Eq. 4: foot pitch as a function of the rocker rotation.
    pub fn phi_of_alpha(&self, alpha: f64) -> f64 {
        self.theta + alpha + self.beta_of_alpha(alpha)
    }

    /// Eq. 5: the ice contact point's shift as a function of the rocker rotation.
    pub fn y_of_alpha(&self, alpha: f64) -> f64 {
        self.beta_of_alpha(alpha) * self.s
    }

    /// Eq. 6: the rocker rotation that puts the ice contact point at `y`.
    pub fn alpha_of_y(&self, y: f64) -> f64 {
        (y / self.s).tan() * (self.s - self.h) / self.r
    }

    /// Eq. 7: foot pitch as a function of the ice contact point (exact).
    pub fn phi_of_y(&self, y: f64) -> f64 {
        self.theta + self.alpha_of_y(y) + y / self.s
    }

    /// Eq. 9: the linearised eq. 7.
    pub fn phi_of_y_linear(&self, y: f64) -> f64 {
        self.theta + y * (self.r + self.s - self.h) / (self.r * self.s)
    }

    /// Eq. 10: the fixed-holder blade radius that gives the same pitch per contact shift.
    pub fn effective_radius(&self) -> f64 {
        self.r * self.s / (self.r + self.s - self.h)
    }

    /// Eq. 11: pitch amplification over a flat rocker (r -> inf), linearised.
    pub fn amplification(&self) -> f64 {
        (self.r + self.s - self.h) / self.r
    }
}

/// Eq. 1, fixed holder: skate rotation when the contact point moves `x` along a blade of
/// radius `r`.
pub fn fixed_alpha(x: f64, r: f64) -> f64 {
    x / r
}

/// Roller rocker: foot rotation when the rocker's contact point moves `x` along the flat top
/// of a lower chassis that keeps all its wheels on the floor. This is eq. 7 with s -> inf.
pub fn roller_alpha(x: f64, r: f64) -> f64 {
    x / r
}

/// Passive fore-aft rocking stiffness of a rigid body standing on a circular rocker of radius
/// `r` that rolls on a level surface, with its centre of mass `h_com` above the contact point.
///
/// The centre of mass rises as `r - (r - h_com) cos(alpha)`, so the potential energy is
/// `m g (r - (r - h_com) cos alpha)` and its curvature at zero is `m g (r - h_com)`, in N m/rad.
/// Positive is stable (the body rocks back to centre), negative tips over. A model: the skater
/// is treated as rigid, which a real skater isn't (the ankle and knees steer).
pub fn passive_rocking_stiffness(mass_kg: f64, r: f64, h_com: f64) -> f64 {
    mass_kg * 9.81 * (r - h_com)
}

/// Feet to metres.
pub const FT: f64 = 0.3048;

#[cfg(test)]
mod tests {
    use super::*;

    const PAPER: Rocker = Rocker { r: 1.0, s: 2.7, h: 0.03, theta: 0.0 };

    #[test]
    fn the_two_routes_to_pitch_agree() {
        for i in -20..=20 {
            let alpha = f64::from(i) * 0.01;
            let y = PAPER.y_of_alpha(alpha);
            assert!((PAPER.phi_of_alpha(alpha) - PAPER.phi_of_y(y)).abs() < 1e-12);
            assert!((PAPER.alpha_of_y(y) - alpha).abs() < 1e-12);
        }
    }

    #[test]
    fn flat_rocker_is_a_fixed_holder_and_flat_blade_is_a_roller() {
        let flat_rocker = Rocker { r: 1e9, ..PAPER };
        assert!((flat_rocker.effective_radius() - PAPER.s).abs() < 1e-6);
        let flat_blade = Rocker { s: 1e9, ..PAPER };
        assert!((flat_blade.effective_radius() - PAPER.r).abs() < 1e-6);
    }
}
