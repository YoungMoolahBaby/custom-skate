//! Posing the stand-in body (track D: G3's gap above the knee, D4's knee check).
//!
//! MakeHuman's default skeleton and skin weights (`data/rigs/default.mhskel`,
//! `default_weights.mhw`, both CC0 in their own headers; fetched to
//! `D:\custom-skate-data\makehuman\rigs\`). Each bone's ends are groups of mesh vertices, so the
//! skeleton follows the fitted body; each skin vertex moves with a weighted mix of its bones
//! (linear blend skinning), on MakeHuman's control mesh, which is then smoothed by the same
//! Catmull-Clark as the rest (`leg::Subdivision`), as MakeHuman poses and smooths.
//!
//! Units and axes are MakeHuman's (dm; y up, z forward, +x the model's left).

use std::collections::HashMap;
use std::path::Path;

use nalgebra::{Rotation3, Vector3};
use serde_json::Value;

pub struct Bone {
    pub name: String,
    pub head: String,
    pub tail: String,
    pub parent: Option<usize>,
}

/// The skeleton (bones parents first) and its skin weights.
pub struct Rig {
    pub bones: Vec<Bone>,
    pub index: HashMap<String, usize>,
    pub joints: HashMap<String, Vec<u32>>,
    /// Per bone, (vertex, weight).
    pub weights: Vec<Vec<(u32, f64)>>,
    /// Kneecaps, bones MakeHuman's skeleton doesn't have (`add_patella`), after its own.
    pub patellae: Vec<Patella>,
}

/// The kneecap's flexion on the thigh per degree of the knee's: "As the knee flexed, the patella
/// flexed by 0.7 times the tibiofemoral flexion angle" (Amis, Senavongse & Bull 2006, eight
/// cadaver knees; RESEARCH.md 24).
pub const PATELLA_FLEX: f64 = 0.7;

/// A kneecap (RESEARCH.md 24). MakeHuman has none: the skin over the knee blends the thigh's and
/// the shin's motions and follows the thigh, where the real patella turns mostly with the shin.
/// Here it is a bone on the thigh, moved by the knee's turn (the shin's own, about `knee`) by
/// three rules:
/// - it turns `PATELLA_FLEX` times the knee's turn, the same way (Amis et al. 2006);
/// - its centre stays on a circle about the knee's axis (the patella's centre runs on a circle
///   round the trochlea: Iranpour et al. 2010, from ~16 deg of flexion);
/// - its tendon doesn't stretch: its lower pole stays as far from the tibial tuberosity, on the
///   shin, as standing.
///
/// So it turns 0.7 times the knee's turn and goes round the knee by whatever keeps the tendon's
/// length. Points in the rest frame, as the rig's.
#[derive(Clone, Debug)]
pub struct Patella {
    pub bone: usize,
    pub femur: usize,
    pub shin: usize,
    /// The knee's turning centre (the shin bone's head).
    pub knee: Vector3<f64>,
    pub centre: Vector3<f64>,
    /// The lower pole, where the patellar tendon starts.
    pub low: Vector3<f64>,
    /// The tibial tuberosity, where the tendon ends.
    pub tub: Vector3<f64>,
}

impl Patella {
    /// Its motion on the thigh for the knee's turn `turn` (the shin's own rotation about `knee`),
    /// and how far round the knee its centre goes as a share of the knee's turn. None if no turn
    /// round the knee up to 2.5 times the knee's keeps the tendon's length.
    pub fn on_thigh(&self, turn: &Rotation3<f64>) -> Option<(Motion, f64)> {
        let w = turn.scaled_axis();
        if w.norm() < 1e-9 {
            return Some((Motion::identity(), 0.0));
        }
        let tub = turn * (self.tub - self.knee) + self.knee;
        let len = (self.low - self.tub).norm();
        let own = Rotation3::new(w * PATELLA_FLEX);
        let at = |k: f64| {
            let c = Rotation3::new(w * k) * (self.centre - self.knee) + self.knee;
            Motion { r: own, t: c - own * self.centre }
        };
        let stretch = |k: f64| (at(k).apply(self.low) - tub).norm() - len;
        // Going round the knee by only the patella's own turn leaves the tendon too long (the shin
        // has turned further), so the root is above PATELLA_FLEX: the first sign change from there.
        let step = 0.005;
        let mut k0 = PATELLA_FLEX;
        let mut s0 = stretch(k0);
        let (mut lo, mut hi) = loop {
            let k1 = k0 + step;
            if k1 > 2.5 {
                return None;
            }
            let s1 = stretch(k1);
            if s0 == 0.0 {
                break (k0, k0);
            }
            if s0.signum() != s1.signum() {
                break (k0, k1);
            }
            (k0, s0) = (k1, s1);
        };
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if stretch(mid).signum() == stretch(lo).signum() {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let k = 0.5 * (lo + hi);
        Some((at(k), k))
    }
}

/// A rigid motion x -> r x + t.
#[derive(Clone, Copy, Debug)]
pub struct Motion {
    pub r: Rotation3<f64>,
    pub t: Vector3<f64>,
}

impl Motion {
    pub fn identity() -> Self {
        Motion { r: Rotation3::identity(), t: Vector3::zeros() }
    }

    pub fn apply(&self, x: Vector3<f64>) -> Vector3<f64> {
        self.r * x + self.t
    }

    /// self after other: x -> self(other(x)).
    pub fn after(&self, other: &Motion) -> Motion {
        Motion { r: self.r * other.r, t: self.r * other.t + self.t }
    }

    pub fn inverse(&self) -> Motion {
        let r = self.r.inverse();
        Motion { r, t: -(r * self.t) }
    }
}

fn read_json(path: &Path) -> Result<Value, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
}

impl Rig {
    pub fn load(dir: &Path) -> Result<Self, String> {
        let skel = read_json(&dir.join("rigs/default.mhskel"))?;
        let wts = read_json(&dir.join("rigs/default_weights.mhw"))?;
        let bones_v = skel["bones"].as_object().ok_or("mhskel: no bones")?;
        // Parents first: repeatedly take the bones whose parent is placed.
        let mut order: Vec<String> = Vec::new();
        let mut placed: HashMap<String, usize> = HashMap::new();
        while order.len() < bones_v.len() {
            let before = order.len();
            for (name, b) in bones_v {
                if placed.contains_key(name) {
                    continue;
                }
                let parent = b["parent"].as_str();
                if parent.is_none_or(|p| placed.contains_key(p)) {
                    placed.insert(name.clone(), order.len());
                    order.push(name.clone());
                }
            }
            if order.len() == before {
                return Err("mhskel: a bone's parent never resolves".into());
            }
        }
        let bones: Vec<Bone> = order
            .iter()
            .map(|name| {
                let b = &bones_v[name];
                Bone {
                    name: name.clone(),
                    head: b["head"].as_str().unwrap_or("").to_string(),
                    tail: b["tail"].as_str().unwrap_or("").to_string(),
                    parent: b["parent"].as_str().and_then(|p| placed.get(p).copied()),
                }
            })
            .collect();
        let mut joints = HashMap::new();
        for (name, list) in skel["joints"].as_object().ok_or("mhskel: no joints")? {
            let idx: Vec<u32> = list.as_array().map(|a| a.iter().filter_map(|v| v.as_u64().map(|x| x as u32)).collect()).unwrap_or_default();
            joints.insert(name.clone(), idx);
        }
        let mut weights = vec![Vec::new(); bones.len()];
        for (name, list) in wts["weights"].as_object().ok_or("mhw: no weights")? {
            let Some(&b) = placed.get(name) else { continue };
            weights[b] = list
                .as_array()
                .map(|a| {
                    a.iter()
                        .filter_map(|p| {
                            let p = p.as_array()?;
                            Some((p.first()?.as_u64()? as u32, p.get(1)?.as_f64()?))
                        })
                        .collect()
                })
                .unwrap_or_default();
        }
        Ok(Rig { bones, index: placed, joints, weights, patellae: Vec::new() })
    }

    /// Adds a kneecap to a leg (`side` "R" or "L"): a bone on the lower thigh bone, its points
    /// (rest frame), and the share of each skin vertex's weight it takes (every bone's weight on
    /// that vertex scaled down by the share, the kneecap taking the rest).
    pub fn add_patella(&mut self, raw: &[Vector3<f64>], side: &str, centre: Vector3<f64>, low: Vector3<f64>, tub: Vector3<f64>, share: &[(u32, f64)]) -> Result<usize, String> {
        let femur = *self.index.get(&format!("upperleg02.{side}")).ok_or("no upperleg02")?;
        let shin = *self.index.get(&format!("lowerleg01.{side}")).ok_or("no lowerleg01")?;
        let knee = self.joint(raw, &self.bones[shin].head).ok_or("no knee joint")?;
        let name = format!("patella.{side}");
        let bone = self.bones.len();
        self.bones.push(Bone { name: name.clone(), head: String::new(), tail: String::new(), parent: Some(femur) });
        self.index.insert(name, bone);
        let a: HashMap<u32, f64> = share.iter().copied().collect();
        let mut sum: HashMap<u32, f64> = HashMap::new();
        for ws in &mut self.weights {
            for (i, w) in ws.iter_mut() {
                if let Some(s) = a.get(i) {
                    *sum.entry(*i).or_default() += *w;
                    *w *= 1.0 - s;
                }
            }
        }
        self.weights.push(share.iter().map(|&(i, s)| (i, s * sum.get(&i).copied().unwrap_or(0.0))).collect());
        self.patellae.push(Patella { bone, femur, shin, knee, centre, low, tub });
        Ok(bone)
    }

    /// A joint's position: the mean of its vertices.
    pub fn joint(&self, raw: &[Vector3<f64>], name: &str) -> Option<Vector3<f64>> {
        let idx = self.joints.get(name)?;
        (!idx.is_empty()).then(|| idx.iter().fold(Vector3::zeros(), |a, &i| a + raw[i as usize]) / idx.len() as f64)
    }

    /// Every bone's motion, given rotations for some bones, each about its own head in the rest
    /// frame and composed down the hierarchy (a child moves with its parent).
    pub fn motions(&self, raw: &[Vector3<f64>], local: &HashMap<String, Rotation3<f64>>) -> Vec<Motion> {
        let mut out: Vec<Motion> = Vec::with_capacity(self.bones.len());
        for b in &self.bones {
            let own = match (local.get(&b.name), self.joint(raw, &b.head)) {
                (Some(r), Some(h)) => Motion { r: *r, t: h - r * h },
                _ => Motion::identity(),
            };
            let global = match b.parent {
                Some(p) => out[p].after(&own),
                None => own,
            };
            out.push(global);
        }
        // The kneecaps, from the knee's turn. Where no turn keeps the tendon's length (none in the
        // poses used: the `body` run reports them), it turns about the knee as the shin does.
        for p in &self.patellae {
            let turn = local.get(&self.bones[p.shin].name).copied().unwrap_or_else(Rotation3::identity);
            let on = p.on_thigh(&turn).map_or_else(
                || Motion { r: Rotation3::new(turn.scaled_axis() * PATELLA_FLEX), t: p.knee - Rotation3::new(turn.scaled_axis() * PATELLA_FLEX) * p.knee },
                |(m, _)| m,
            );
            out[p.bone] = out[p.femur].after(&on);
        }
        out
    }

    /// Linear blend skinning: each vertex moved by its bones' motions, weighted; any weight short
    /// of 1 stays put (and weights over 1 are normalised).
    pub fn skin(&self, raw: &[Vector3<f64>], motions: &[Motion]) -> Vec<Vector3<f64>> {
        let mut acc = vec![Vector3::zeros(); raw.len()];
        let mut sum = vec![0.0f64; raw.len()];
        for (b, ws) in self.weights.iter().enumerate() {
            let m = &motions[b];
            for &(i, w) in ws {
                let i = i as usize;
                if i < raw.len() {
                    acc[i] += m.apply(raw[i]) * w;
                    sum[i] += w;
                }
            }
        }
        raw.iter()
            .enumerate()
            .map(|(i, &x)| if sum[i] > 1.0 { acc[i] / sum[i] } else { acc[i] + x * (1.0 - sum[i]) })
            .collect()
    }
}

/// A pose of the legs, degrees: hip flexion (thigh forward), knee flexion (shin back), ankle
/// (foot up toward the shin, dorsiflexion). Per side: right, left.
#[derive(Clone, Copy, Debug, Default)]
pub struct LegPose {
    pub hip: f64,
    pub knee: f64,
    pub ankle: f64,
}

/// The local rotations for a pose: each leg first stood (turned at the hip by `stand`, the
/// rotation that puts its knee over its ankle), then flexed about MakeHuman's sideways axis (x):
/// hip and knee in the leg's own frame after standing.
pub fn leg_rotations(stand: [Rotation3<f64>; 2], pose: [LegPose; 2]) -> HashMap<String, Rotation3<f64>> {
    let mut m = HashMap::new();
    for (k, side) in ["R", "L"].iter().enumerate() {
        let x = Vector3::x_axis();
        let p = pose[k];
        // Thigh forward: the knee moves to +z, a turn about +x by a negative angle.
        let hip = Rotation3::from_axis_angle(&x, -p.hip.to_radians());
        m.insert(format!("upperleg01.{side}"), hip * stand[k]);
        // Knee: the shin swings back (-z), a turn about +x by a positive angle. Its bone turns in
        // the rest frame and is then carried by the thigh (hip x stand), so the turn is written
        // as stand^-1 x R x stand: about the sideways axis as it is once the leg is stood. About
        // the rest axis, tilted by standing, the shin swung out sideways as it bent (the ankle
        // 29-59 mm off line, 2026-11-12).
        let inv = stand[k].inverse();
        m.insert(format!("lowerleg01.{side}"), inv * Rotation3::from_axis_angle(&x, p.knee.to_radians()) * stand[k]);
        // Ankle: the toes up toward the shin, the same way.
        if p.ankle != 0.0 {
            m.insert(format!("foot.{side}"), inv * Rotation3::from_axis_angle(&x, -p.ankle.to_radians()) * stand[k]);
        }
    }
    m
}
