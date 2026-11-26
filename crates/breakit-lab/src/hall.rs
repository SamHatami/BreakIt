//! The test hall: a steel portal-frame shed with crane, roof lights, high
//! windows, high-bay lamps and a mezzanine control room. All in lab metres,
//! floor at y = 0, length along x, span along z.

use bevy::math::{Quat, Vec3};

use crate::palette::Mat;
use crate::scenery::{Anchor, Scenery};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HallSpec {
    /// Number of bays between portal frames.
    pub bays: u32,
    /// Frame spacing, m.
    pub bay: f32,
    /// Clear span between wall lines, m.
    pub span: f32,
    /// Height of the eaves, m.
    pub eave: f32,
    /// Height of the ridge, m.
    pub ridge: f32,
    /// Top of the crane rail, m.
    pub crane_rail: f32,
}

impl Default for HallSpec {
    fn default() -> Self {
        Self {
            bays: 10,
            bay: 6.0,
            span: 30.0,
            eave: 11.0,
            ridge: 13.5,
            crane_rail: 8.6,
        }
    }
}

impl HallSpec {
    pub fn length(&self) -> f32 {
        self.bays as f32 * self.bay
    }

    pub fn half(&self) -> (f32, f32) {
        (self.length() * 0.5, self.span * 0.5)
    }

    /// Underside of the roof sheeting at `z`.
    pub fn roof_y(&self, z: f32) -> f32 {
        let hz = self.span * 0.5;
        self.ridge - (self.ridge - self.eave) * (z.abs() / hz).min(1.0)
    }

    pub fn frame_xs(&self) -> impl Iterator<Item = f32> + '_ {
        let hx = self.length() * 0.5;
        (0..=self.bays).map(move |i| -hx + i as f32 * self.bay)
    }

    /// Where the high-bay lamps hang (their glowing face).
    pub fn lamp_positions(&self) -> Vec<Vec3> {
        let hx = self.length() * 0.5;
        let mut out = Vec::new();
        for &z in &[-7.5f32, 7.5] {
            for i in 0..self.bays {
                let x = -hx + (i as f32 + 0.5) * self.bay;
                out.push(Vec3::new(x, self.lamp_height(z), z));
            }
        }
        out
    }

    fn lamp_height(&self, z: f32) -> f32 {
        self.roof_y(z) - 1.3
    }
}

const COL_DEPTH: f32 = 0.6;
const RAFTER_DEPTH: f32 = 0.55;

pub fn build(spec: &HallSpec) -> Scenery {
    let mut s = Scenery::default();
    floor(&mut s, spec);
    frames(&mut s, spec);
    secondary(&mut s, spec);
    cladding(&mut s, spec);
    crane(&mut s, spec, -6.0, 2.5);
    s.anchor(Anchor::CraneHook, Vec3::new(-6.0, 4.5, 2.5));
    lamps(&mut s, spec);
    control_room(&mut s, spec);
    let (hx, hz) = spec.half();
    s.anchor(Anchor::ControlRoom, Vec3::new(-hx + 3.4, 5.0, -hz + 3.2));
    for p in spec.lamp_positions() {
        s.anchor(Anchor::HighBayLamp, p);
    }
    s
}

fn floor(s: &mut Scenery, spec: &HallSpec) {
    let (hx, hz) = spec.half();
    s.m(Mat::Concrete).aabb(
        Vec3::new(-hx - 1.0, -0.4, -hz - 1.0),
        Vec3::new(hx + 1.0, 0.0, hz + 1.0),
    );
    // Saw-cut joints on a 6 m grid.
    let joint = s.m(Mat::Dark);
    let y = 0.002;
    let mut x = -hx;
    while x <= hx + 0.01 {
        joint.aabb(
            Vec3::new(x - 0.008, -0.01, -hz),
            Vec3::new(x + 0.008, y, hz),
        );
        x += 6.0;
    }
    let mut z = -hz + 3.0;
    while z < hz {
        joint.aabb(
            Vec3::new(-hx, -0.01, z - 0.008),
            Vec3::new(hx, y, z + 0.008),
        );
        z += 6.0;
    }
    // Walkway lines along both long walls and across the door end.
    let paint = s.m(Mat::FloorPaint);
    let y = 0.004;
    for &zl in &[-hz + 3.2, hz - 3.2] {
        for off in [-0.6f32, 0.6] {
            paint.aabb(
                Vec3::new(-hx + 1.0, -0.01, zl + off - 0.05),
                Vec3::new(hx - 1.0, y, zl + off + 0.05),
            );
        }
    }
    // Hatched keep-clear box in front of the door.
    let (dx, dz) = (hx - 4.0, 4.0);
    for k in 0..9 {
        let z0 = dz - 3.0 + k as f32 * 0.75;
        paint.cuboid(
            Vec3::new(dx, y * 0.5, z0),
            Vec3::new(1.3, y * 0.5 + 0.005, 0.07),
            Quat::from_rotation_y(0.785),
        );
    }
    // Strong floor: steel plates with anchor points, under the test rigs.
    let (min, max) = (Vec3::new(-14.0, -0.02, -6.0), Vec3::new(-2.0, 0.006, 1.0));
    s.m(Mat::FloorSteel).aabb(min, max);
    let dark = s.m(Mat::Dark);
    let mut x = min.x + 0.5;
    while x < max.x {
        let mut z = min.z + 0.5;
        while z < max.z {
            dark.disc(Vec3::new(x, max.y + 0.002, z), Vec3::Y, 0.06, 8);
            z += 1.0;
        }
        x += 1.0;
    }
}

fn frames(s: &mut Scenery, spec: &HallSpec) {
    let (_, hz) = spec.half();
    let zc = hz - 0.35;
    for x in spec.frame_xs() {
        for side in [-1.0f32, 1.0] {
            let z = side * zc;
            let top = spec.eave - 0.3;
            let steel = s.m(Mat::FrameSteel);
            steel.i_beam(
                Vec3::new(x, 0.02, z),
                Vec3::new(x, top, z),
                COL_DEPTH,
                0.3,
                0.014,
                0.022,
                Vec3::Z,
            );
            // Rafter from the knee to the ridge, under the purlins.
            let knee = Vec3::new(x, spec.roof_y(z) - 0.2 - RAFTER_DEPTH * 0.5, z);
            let apex = Vec3::new(x, spec.roof_y(0.0) - 0.2 - RAFTER_DEPTH * 0.5, 0.0);
            steel.i_beam(knee, apex, RAFTER_DEPTH, 0.25, 0.012, 0.02, Vec3::Y);
            // Haunch: a deeper section for the first few metres.
            let along = (apex - knee).normalize();
            let h0 = knee - Vec3::Y * 0.45;
            let h1 = knee + along * 3.0 - Vec3::Y * 0.3;
            steel.bar(h0, h1, 0.012, 0.35, Vec3::Y);
            steel.bar(h0, knee + along * 3.0 - Vec3::Y * 0.5, 0.25, 0.02, Vec3::Y);
            // Knee stiffener and cap plate.
            steel.aabb(
                Vec3::new(x - 0.14, top - 0.9, z - 0.3),
                Vec3::new(x + 0.14, top - 0.88, z + 0.3),
            );
            // Base plate, grout and anchor nuts.
            s.m(Mat::LightSteel).aabb(
                Vec3::new(x - 0.3, 0.0, z - 0.4),
                Vec3::new(x + 0.3, 0.03, z + 0.4),
            );
            for (dx, dz) in [(-0.2, -0.3), (0.2, -0.3), (-0.2, 0.3), (0.2, 0.3)] {
                s.m(Mat::Dark).cylinder(
                    Vec3::new(x + dx, 0.03, z + dz),
                    Vec3::new(x + dx, 0.08, z + dz),
                    0.025,
                    6,
                    true,
                );
            }
        }
        // Apex plates.
        let ay = spec.roof_y(0.0) - 0.2 - RAFTER_DEPTH * 0.5;
        s.m(Mat::FrameSteel).aabb(
            Vec3::new(x - 0.14, ay - 0.6, -0.03),
            Vec3::new(x + 0.14, ay + 0.3, 0.03),
        );
    }
}

fn secondary(s: &mut Scenery, spec: &HallSpec) {
    let (hx, hz) = spec.half();
    let lt = s.m(Mat::LightSteel);
    // Purlins on both slopes.
    let n = 9;
    for side in [-1.0f32, 1.0] {
        for k in 0..n {
            let z = side * (0.4 + (hz - 0.7) * k as f32 / (n - 1) as f32);
            let y = spec.roof_y(z) - 0.1;
            lt.bar(
                Vec3::new(-hx, y, z),
                Vec3::new(hx, y, z),
                0.07,
                0.2,
                Vec3::Y,
            );
        }
        // Side rails (girts) on the long walls.
        for &y in &[2.6f32, 5.2, 7.6, 10.0] {
            let z = side * (hz - 0.05);
            lt.bar(
                Vec3::new(-hx, y, z),
                Vec3::new(hx, y, z),
                0.08,
                0.2,
                Vec3::Y,
            );
        }
        // Cross bracing in the end bays, walls and roof.
        for bay in [0, spec.bays - 1] {
            let x0 = -hx + bay as f32 * spec.bay;
            let x1 = x0 + spec.bay;
            let z = side * (hz - 0.12);
            lt.cylinder(
                Vec3::new(x0, 0.4, z),
                Vec3::new(x1, spec.eave - 0.6, z),
                0.02,
                6,
                false,
            );
            lt.cylinder(
                Vec3::new(x1, 0.4, z),
                Vec3::new(x0, spec.eave - 0.6, z),
                0.02,
                6,
                false,
            );
            let za = side * (hz - 0.4);
            let ya = spec.roof_y(za) - 0.25;
            let yb = spec.roof_y(0.5) - 0.25;
            lt.cylinder(
                Vec3::new(x0, ya, za),
                Vec3::new(x1, yb, side * 0.5),
                0.02,
                6,
                false,
            );
            lt.cylinder(
                Vec3::new(x1, ya, za),
                Vec3::new(x0, yb, side * 0.5),
                0.02,
                6,
                false,
            );
        }
    }
    // End-wall wind posts.
    for &x in &[-hx + 0.15, hx - 0.15] {
        for k in -2..=2 {
            let z = k as f32 * 5.0;
            if x > 0.0 && (z - 4.0).abs() < 3.5 {
                continue; // the door opening
            }
            s.m(Mat::FrameSteel).i_beam(
                Vec3::new(x, 0.02, z),
                Vec3::new(x, spec.roof_y(z) - 0.3, z),
                0.3,
                0.15,
                0.01,
                0.014,
                Vec3::X,
            );
        }
    }
}

fn cladding(s: &mut Scenery, spec: &HallSpec) {
    let (hx, hz) = spec.half();
    let window = (7.8f32, 9.6f32);
    // Long walls: lower precast panels, profiled sheet above, a band of
    // high windows.
    for side in [-1.0f32, 1.0] {
        let z = side * hz;
        let n = -side; // inward
        let wall = |s: &mut Scenery, mat: Mat, y0: f32, y1: f32, inset: f32| {
            let zz = z + n * inset;
            let (a, b) = (Vec3::new(-hx, y0, zz), Vec3::new(hx, y0, zz));
            let (c, d) = (Vec3::new(hx, y1, zz), Vec3::new(-hx, y1, zz));
            if side < 0.0 {
                s.m(mat).quad([a, b, c, d]);
            } else {
                s.m(mat).quad([b, a, d, c]);
            }
        };
        wall(s, Mat::Concrete, 0.0, 2.4, 0.06);
        wall(s, Mat::Cladding, 2.4, window.0, 0.0);
        wall(s, Mat::Skylight, window.0, window.1, 0.0);
        wall(s, Mat::Cladding, window.1, spec.eave + 0.2, 0.0);
        // Panel joints on the precast band and ribs on the sheeting.
        let mut x = -hx;
        while x < hx {
            s.m(Mat::Dark).aabb(
                Vec3::new(x - 0.01, 0.0, z + n * 0.07),
                Vec3::new(x + 0.01, 2.4, z + n * 0.065),
            );
            x += 3.0;
        }
        let mut x = -hx + 0.15;
        while x < hx {
            for (y0, y1) in [(2.4, window.0), (window.1, spec.eave)] {
                s.m(Mat::Cladding).bar(
                    Vec3::new(x, y0, z + n * 0.02),
                    Vec3::new(x, y1, z + n * 0.02),
                    0.05,
                    0.04,
                    Vec3::Z,
                );
            }
            x += 0.33;
        }
        // Window mullions.
        let mut x = -hx;
        while x <= hx {
            s.m(Mat::LightSteel).bar(
                Vec3::new(x, window.0, z + n * 0.03),
                Vec3::new(x, window.1, z + n * 0.03),
                0.06,
                0.06,
                Vec3::Z,
            );
            x += 1.5;
        }
        for y in [window.0, window.1] {
            s.m(Mat::LightSteel).bar(
                Vec3::new(-hx, y, z + n * 0.03),
                Vec3::new(hx, y, z + n * 0.03),
                0.1,
                0.1,
                Vec3::Y,
            );
        }
    }

    // Roof: underside of the sheeting, ribs, and roof lights in every
    // other bay.
    for side in [-1.0f32, 1.0] {
        let ze = side * (hz + 0.3);
        let ye = spec.roof_y(hz) - (spec.ridge - spec.eave) * 0.3 / hz;
        let yr = spec.ridge;
        let (a, b) = (Vec3::new(-hx, ye, ze), Vec3::new(hx, ye, ze));
        let (c, d) = (Vec3::new(hx, yr, 0.0), Vec3::new(-hx, yr, 0.0));
        // Facing down and in.
        if side > 0.0 {
            s.m(Mat::Cladding).quad([a, b, c, d]);
        } else {
            s.m(Mat::Cladding).quad([b, a, d, c]);
        }
        for i in 0..spec.bays {
            if i % 2 == 1 {
                continue;
            }
            let x0 = -hx + i as f32 * spec.bay + 1.0;
            let x1 = x0 + spec.bay - 2.0;
            let (z0, z1) = (side * 4.0, side * 9.0);
            let y = |z: f32| spec.roof_y(z) - 0.02;
            let (p, q) = (Vec3::new(x0, y(z0), z0), Vec3::new(x1, y(z0), z0));
            let (r, t) = (Vec3::new(x1, y(z1), z1), Vec3::new(x0, y(z1), z1));
            if side > 0.0 {
                s.m(Mat::Skylight).quad([t, r, q, p]);
            } else {
                s.m(Mat::Skylight).quad([p, q, r, t]);
            }
        }
        let mut x = -hx + 0.15;
        while x < hx {
            let z0 = side * hz;
            s.m(Mat::Cladding).bar(
                Vec3::new(x, spec.roof_y(z0) - 0.03, z0),
                Vec3::new(x, spec.roof_y(0.0) - 0.03, 0.0),
                0.05,
                0.04,
                Vec3::Y,
            );
            x += 0.33;
        }
    }

    // Gable walls. The +x end has the roller door.
    for end in [-1.0f32, 1.0] {
        let x = end * hx;
        let n = -end;
        let wall_quad =
            |s: &mut Scenery, mat: Mat, z0: f32, z1: f32, y0: f32, y1a: f32, y1b: f32| {
                let pts = [
                    Vec3::new(x, y0, z0),
                    Vec3::new(x, y0, z1),
                    Vec3::new(x, y1b, z1),
                    Vec3::new(x, y1a, z0),
                ];
                if n > 0.0 {
                    s.m(mat).quad([pts[1], pts[0], pts[3], pts[2]]);
                } else {
                    s.m(mat).quad(pts);
                }
            };
        // Split the wall into vertical strips so each strip's top follows
        // the roof, leaving a hole for the door at the +x end.
        let strips = 30;
        for k in 0..strips {
            let z0 = -hz + k as f32 * spec.span / strips as f32;
            let z1 = z0 + spec.span / strips as f32;
            let (ya, yb) = (spec.roof_y(z0) + 0.2, spec.roof_y(z1) + 0.2);
            let in_door = end > 0.0 && z0 >= 0.99 && z1 <= 7.01;
            let base: f32 = if in_door { 6.0 } else { 0.0 };
            if !in_door {
                wall_quad(s, Mat::Concrete, z0, z1, 0.0, 2.4, 2.4);
            }
            wall_quad(s, Mat::Cladding, z0, z1, base.max(2.4), ya, yb);
            // A rib per strip.
            s.m(Mat::Cladding).bar(
                Vec3::new(x + n * 0.02, base.max(2.4), z0),
                Vec3::new(x + n * 0.02, ya - 0.2, z0),
                0.05,
                0.04,
                Vec3::X,
            );
        }
        if end > 0.0 {
            roller_door(s, x, n, 1.0, 7.0, 6.0);
        } else {
            // A big window high in the far gable: a bright backdrop.
            wall_quad(s, Mat::Skylight, -6.0, 6.0, 7.5, 10.5, 10.5);
            let mut z = -6.0;
            while z <= 6.01 {
                s.m(Mat::LightSteel).bar(
                    Vec3::new(x + n * 0.03, 7.5, z),
                    Vec3::new(x + n * 0.03, 10.5, z),
                    0.08,
                    0.08,
                    Vec3::X,
                );
                z += 1.5;
            }
        }
    }
}

fn roller_door(s: &mut Scenery, x: f32, n: f32, z0: f32, z1: f32, h: f32) {
    // Slats, slightly open at the bottom so daylight shows.
    let gap = 0.35;
    s.m(Mat::Skylight).quad(if n < 0.0 {
        [
            Vec3::new(x + 0.3, 0.0, z0),
            Vec3::new(x + 0.3, 0.0, z1),
            Vec3::new(x + 0.3, gap + 0.05, z1),
            Vec3::new(x + 0.3, gap + 0.05, z0),
        ]
    } else {
        [
            Vec3::new(x - 0.3, 0.0, z1),
            Vec3::new(x - 0.3, 0.0, z0),
            Vec3::new(x - 0.3, gap + 0.05, z0),
            Vec3::new(x - 0.3, gap + 0.05, z1),
        ]
    });
    let mut y = gap;
    while y < h {
        s.m(Mat::MachineGrey).bar(
            Vec3::new(x + n * 0.05, y + 0.06, z0),
            Vec3::new(x + n * 0.05, y + 0.06, z1),
            0.04,
            0.11,
            Vec3::Y,
        );
        y += 0.12;
    }
    // Guides, box and the hazard-striped posts.
    for z in [z0, z1] {
        s.m(Mat::Yellow).aabb(
            Vec3::new(x + n * 0.02 - 0.08, 0.0, z - 0.1),
            Vec3::new(x + n * 0.02 + 0.08, h, z + 0.1),
        );
        for k in 0..6 {
            let y = 0.05 + k as f32 * 0.3;
            s.m(Mat::Dark).aabb(
                Vec3::new(
                    x + n * 0.25 - 0.11,
                    y,
                    z - 0.11 + if z == z0 { -0.3 } else { 0.3 },
                ),
                Vec3::new(
                    x + n * 0.25 + 0.11,
                    y + 0.15,
                    z + 0.11 + if z == z0 { -0.3 } else { 0.3 },
                ),
            );
            s.m(Mat::Yellow).aabb(
                Vec3::new(
                    x + n * 0.25 - 0.1,
                    y + 0.15,
                    z - 0.1 + if z == z0 { -0.3 } else { 0.3 },
                ),
                Vec3::new(
                    x + n * 0.25 + 0.1,
                    y + 0.3,
                    z + 0.1 + if z == z0 { -0.3 } else { 0.3 },
                ),
            );
        }
    }
    s.m(Mat::MachineGrey).bar(
        Vec3::new(x + n * 0.3, h + 0.35, z0 - 0.2),
        Vec3::new(x + n * 0.3, h + 0.35, z1 + 0.2),
        0.6,
        0.7,
        Vec3::Y,
    );
}

/// Overhead travelling crane: runway beams on column brackets, a double
/// box-girder bridge at `bridge_x`, trolley at `trolley_z`.
fn crane(s: &mut Scenery, spec: &HallSpec, bridge_x: f32, trolley_z: f32) {
    let (hx, hz) = spec.half();
    let rail = spec.crane_rail;
    let zr = hz - 1.0;
    for side in [-1.0f32, 1.0] {
        let z = side * zr;
        let beam_top = rail - 0.08;
        s.m(Mat::FrameSteel).i_beam(
            Vec3::new(-hx, beam_top - 0.35, z),
            Vec3::new(hx, beam_top - 0.35, z),
            0.7,
            0.32,
            0.014,
            0.025,
            Vec3::Y,
        );
        s.m(Mat::Dark).bar(
            Vec3::new(-hx, rail - 0.04, z),
            Vec3::new(hx, rail - 0.04, z),
            0.07,
            0.08,
            Vec3::Y,
        );
        for x in spec.frame_xs() {
            // Bracket from the column to under the runway.
            let zc = side * (hz - 0.35);
            s.m(Mat::FrameSteel).bar(
                Vec3::new(x, beam_top - 0.9, zc),
                Vec3::new(x, beam_top - 0.9, z),
                0.25,
                0.4,
                Vec3::Y,
            );
            s.m(Mat::FrameSteel).bar(
                Vec3::new(x, beam_top - 2.0, zc),
                Vec3::new(x, beam_top - 0.95, z),
                0.02,
                0.25,
                Vec3::X,
            );
        }
        // End stops.
        for x in [-hx + 0.4, hx - 0.4] {
            s.m(Mat::Yellow).aabb(
                Vec3::new(x - 0.15, rail, z - 0.2),
                Vec3::new(x + 0.15, rail + 0.4, z + 0.2),
            );
        }
    }
    // Bridge girders.
    let gy = rail + 0.3 + 0.55;
    for dx in [-1.1f32, 1.1] {
        s.m(Mat::Yellow).bar(
            Vec3::new(bridge_x + dx, gy, -zr - 0.1),
            Vec3::new(bridge_x + dx, gy, zr + 0.1),
            0.5,
            1.1,
            Vec3::Y,
        );
        // Stiffener lines on the girder sides.
        let mut z = -zr + 1.0;
        while z < zr {
            s.m(Mat::Yellow).aabb(
                Vec3::new(bridge_x + dx - 0.27, gy - 0.55, z - 0.02),
                Vec3::new(bridge_x + dx + 0.27, gy + 0.55, z + 0.02),
            );
            z += 1.5;
        }
        // Walkway rail on the outer girder.
        s.m(Mat::Yellow).cylinder(
            Vec3::new(bridge_x + dx * 1.25, gy + 1.5, -zr),
            Vec3::new(bridge_x + dx * 1.25, gy + 1.5, zr),
            0.025,
            6,
            false,
        );
    }
    // End carriages with hazard stripes and wheels.
    for side in [-1.0f32, 1.0] {
        let z = side * zr;
        s.m(Mat::Yellow).aabb(
            Vec3::new(bridge_x - 1.8, rail + 0.1, z - 0.3),
            Vec3::new(bridge_x + 1.8, rail + 0.75, z + 0.3),
        );
        for ex in [-1.8f32, 1.8] {
            for k in 0..4 {
                let y0 = rail + 0.1 + k as f32 * 0.16;
                let mat = if k % 2 == 0 { Mat::Dark } else { Mat::Yellow };
                s.m(mat).aabb(
                    Vec3::new(bridge_x + ex - 0.02 * ex.signum(), y0, z - 0.31),
                    Vec3::new(bridge_x + ex + 0.02 * ex.signum(), y0 + 0.16, z + 0.31),
                );
            }
            s.m(Mat::Dark).cylinder(
                Vec3::new(bridge_x + ex * 0.75, rail + 0.15, z - 0.12),
                Vec3::new(bridge_x + ex * 0.75, rail + 0.15, z + 0.12),
                0.2,
                12,
                true,
            );
        }
    }
    // Trolley and hoist.
    let ty = gy + 0.55;
    s.m(Mat::Yellow).aabb(
        Vec3::new(bridge_x - 1.4, ty, trolley_z - 1.0),
        Vec3::new(bridge_x + 1.4, ty + 0.25, trolley_z + 1.0),
    );
    s.m(Mat::Dark).cylinder(
        Vec3::new(bridge_x - 0.9, ty + 0.65, trolley_z),
        Vec3::new(bridge_x + 0.6, ty + 0.65, trolley_z),
        0.38,
        16,
        true,
    );
    s.m(Mat::MachineGrey).aabb(
        Vec3::new(bridge_x + 0.6, ty + 0.25, trolley_z - 0.4),
        Vec3::new(bridge_x + 1.3, ty + 1.0, trolley_z + 0.4),
    );
    // Ropes and hook block.
    let hook_y = 4.2;
    for (dx, dz) in [
        (-0.15f32, -0.12f32),
        (0.15, -0.12),
        (-0.15, 0.12),
        (0.15, 0.12),
    ] {
        s.m(Mat::Dark).cylinder(
            Vec3::new(bridge_x + dx, ty + 0.3, trolley_z + dz),
            Vec3::new(bridge_x + dx, hook_y + 0.5, trolley_z + dz),
            0.012,
            5,
            false,
        );
    }
    s.m(Mat::Yellow).aabb(
        Vec3::new(bridge_x - 0.3, hook_y, trolley_z - 0.18),
        Vec3::new(bridge_x + 0.3, hook_y + 0.6, trolley_z + 0.18),
    );
    s.m(Mat::Dark).cylinder(
        Vec3::new(bridge_x, hook_y, trolley_z),
        Vec3::new(bridge_x, hook_y - 0.35, trolley_z),
        0.05,
        8,
        false,
    );
    s.m(Mat::Dark).ring(
        Vec3::new(bridge_x, hook_y - 0.55, trolley_z),
        Vec3::Z,
        0.2,
        0.07,
        0.08,
        14,
    );
}

fn lamps(s: &mut Scenery, spec: &HallSpec) {
    for p in spec.lamp_positions() {
        let top = p + Vec3::Y * 0.45;
        s.m(Mat::LampHousing)
            .frustum(p + Vec3::Y * 0.02, top, 0.32, 0.16, 16, false);
        s.m(Mat::LampHousing).aabb(
            top - Vec3::new(0.15, 0.0, 0.12),
            top + Vec3::new(0.15, 0.22, 0.12),
        );
        s.m(Mat::LampGlow).disc(p, -Vec3::Y, 0.3, 16);
        s.m(Mat::Dark).cylinder(
            top + Vec3::Y * 0.22,
            Vec3::new(p.x, spec.roof_y(p.z) - 0.2, p.z),
            0.008,
            4,
            false,
        );
    }
}

/// Mezzanine with a glazed control room in the far (-x, -z) corner.
fn control_room(s: &mut Scenery, spec: &HallSpec) {
    let (hx, hz) = spec.half();
    let (x0, x1) = (-hx + 0.4, -hx + 9.0);
    let (z0, z1) = (-hz + 0.4, -hz + 6.5);
    let deck = 3.6;
    // Columns and deck.
    for x in [x0 + 0.2, (x0 + x1) * 0.5, x1 - 0.2] {
        for z in [z0 + 0.2, z1 - 0.2] {
            s.m(Mat::FrameSteel).bar(
                Vec3::new(x, 0.0, z),
                Vec3::new(x, deck - 0.3, z),
                0.2,
                0.2,
                Vec3::X,
            );
        }
    }
    s.m(Mat::FrameSteel)
        .aabb(Vec3::new(x0, deck - 0.4, z0), Vec3::new(x1, deck - 0.1, z1));
    s.m(Mat::FloorSteel)
        .aabb(Vec3::new(x0, deck - 0.1, z0), Vec3::new(x1, deck, z1));
    // Handrail along the open edges.
    let rail = |s: &mut Scenery, a: Vec3, b: Vec3| {
        let n = (a.distance(b) / 1.2).ceil() as i32;
        for k in 0..=n {
            let p = a.lerp(b, k as f32 / n as f32);
            s.m(Mat::Yellow)
                .cylinder(p, p + Vec3::Y * 1.1, 0.025, 6, false);
        }
        for h in [0.55f32, 1.1] {
            s.m(Mat::Yellow)
                .cylinder(a + Vec3::Y * h, b + Vec3::Y * h, 0.025, 6, false);
        }
    };
    rail(
        s,
        Vec3::new(x0 + 6.0, deck, z1 - 0.05),
        Vec3::new(x1 - 0.05, deck, z1 - 0.05),
    );
    rail(
        s,
        Vec3::new(x1 - 0.05, deck, z1 - 0.05),
        Vec3::new(x1 - 0.05, deck, z0 + 2.5),
    );
    // Stair up the x1 side.
    let steps = 18;
    for k in 0..steps {
        let t = k as f32 / steps as f32;
        let y = t * deck;
        let z = z0 + 0.3 + t * 2.2 * (deck / 3.6);
        s.m(Mat::FloorSteel).aabb(
            Vec3::new(x1 + 0.05, y + 0.17, z),
            Vec3::new(x1 + 1.05, y + 0.2, z + 0.25),
        );
    }
    for xx in [x1 + 0.05, x1 + 1.05] {
        s.m(Mat::FrameSteel).bar(
            Vec3::new(xx, 0.0, z0 + 0.2),
            Vec3::new(xx, deck, z0 + 2.6),
            0.03,
            0.25,
            Vec3::Y,
        );
        s.m(Mat::Yellow).cylinder(
            Vec3::new(xx, 1.0, z0 + 0.2),
            Vec3::new(xx, deck + 1.0, z0 + 2.6),
            0.025,
            6,
            false,
        );
    }
    // Cabin: solid back walls, glazing on the two faces looking at the hall.
    let (cx0, cx1, cz0, cz1) = (x0, x0 + 6.0, z0, z1 - 0.6);
    let (cy0, cy1) = (deck, deck + 2.9);
    let grey = s.m(Mat::MachineGrey);
    grey.aabb(
        Vec3::new(cx0, cy1, cz0),
        Vec3::new(cx1 + 0.1, cy1 + 0.15, cz1 + 0.1),
    );
    grey.aabb(
        Vec3::new(cx0, cy0, cz1 - 0.05),
        Vec3::new(cx1, cy0 + 0.9, cz1 + 0.05),
    );
    grey.aabb(
        Vec3::new(cx1 - 0.05, cy0, cz0),
        Vec3::new(cx1 + 0.05, cy0 + 0.9, cz1),
    );
    // Lit glazing.
    s.m(Mat::WindowWarm).quad([
        Vec3::new(cx0, cy0 + 0.9, cz1 + 0.04),
        Vec3::new(cx1, cy0 + 0.9, cz1 + 0.04),
        Vec3::new(cx1, cy1, cz1 + 0.04),
        Vec3::new(cx0, cy1, cz1 + 0.04),
    ]);
    s.m(Mat::WindowWarm).quad([
        Vec3::new(cx1 + 0.04, cy0 + 0.9, cz1),
        Vec3::new(cx1 + 0.04, cy0 + 0.9, cz0),
        Vec3::new(cx1 + 0.04, cy1, cz0),
        Vec3::new(cx1 + 0.04, cy1, cz1),
    ]);
    let mut x = cx0;
    while x <= cx1 + 0.01 {
        s.m(Mat::Dark).bar(
            Vec3::new(x, cy0 + 0.9, cz1 + 0.06),
            Vec3::new(x, cy1, cz1 + 0.06),
            0.06,
            0.06,
            Vec3::Z,
        );
        x += 1.2;
    }
    let mut z = cz0;
    while z <= cz1 + 0.01 {
        s.m(Mat::Dark).bar(
            Vec3::new(cx1 + 0.06, cy0 + 0.9, z),
            Vec3::new(cx1 + 0.06, cy1, z),
            0.06,
            0.06,
            Vec3::X,
        );
        z += 1.2;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roof_falls_from_ridge_to_eaves() {
        let h = HallSpec::default();
        assert_eq!(h.roof_y(0.0), h.ridge);
        assert!((h.roof_y(h.span * 0.5) - h.eave).abs() < 1e-5);
        assert!(h.roof_y(5.0) < h.ridge);
    }

    #[test]
    fn lamps_hang_above_the_crane() {
        let h = HallSpec::default();
        for p in h.lamp_positions() {
            assert!(
                p.y > h.crane_rail + 2.0,
                "lamp at {p} clashes with the crane"
            );
            assert!(p.y < h.roof_y(p.z));
        }
    }

    #[test]
    fn hall_builds_within_budget() {
        let s = build(&HallSpec::default());
        let tris = s.triangles();
        assert!(tris > 10_000 && tris < 400_000, "{tris} triangles");
    }
}
