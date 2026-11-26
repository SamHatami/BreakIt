//! Lab dressing: wave flume, reaction frame, hydraulics, tanks, pumps,
//! pipework, cabinets, racks and clutter. Lab metres, floor at y = 0.
//! The play table stands in the open area around `LabSettings::table_at`.

use bevy::math::{Quat, Vec3};

use crate::hall::HallSpec;
use crate::palette::Mat;
use crate::scenery::{Anchor, Rng, Scenery};

pub fn build(spec: &HallSpec, seed: u64) -> Scenery {
    let mut s = Scenery::default();
    let mut rng = Rng::new(seed);
    let (hx, hz) = spec.half();

    wave_flume(&mut s, -24.0, 6.0, -10.0, 1.6, 1.3);
    reaction_frame(&mut s, Vec3::new(-8.5, 0.0, -2.5));
    power_unit(
        &mut s,
        Vec3::new(-3.2, 0.0, -5.0),
        Quat::from_rotation_y(0.0),
    );
    power_unit(
        &mut s,
        Vec3::new(15.5, 0.0, -9.0),
        Quat::from_rotation_y(1.57),
    );
    column_rig(&mut s, Vec3::new(12.0, 0.0, -4.0));
    s.anchor(Anchor::FlumePaddle, Vec3::new(-25.6, 1.4, -10.0));
    s.anchor(Anchor::TestRig, Vec3::new(-8.5, 2.0, -2.5));
    s.anchor(Anchor::TestRig, Vec3::new(12.0, 2.0, -4.0));
    s.anchor(Anchor::PowerUnit, Vec3::new(-3.2, 1.0, -5.0));
    s.anchor(Anchor::PowerUnit, Vec3::new(15.5, 1.0, -9.0));
    s.anchor(Anchor::ElectricalCabinets, Vec3::new(-5.5, 1.5, -hz + 0.8));
    s.anchor(Anchor::ElectricalCabinets, Vec3::new(hx - 0.8, 1.5, -9.5));
    shake_table(&mut s, Vec3::new(22.0, 0.0, -2.0));

    // Tanks along the back wall feeding the flume.
    for (i, x) in [9.0f32, 12.2, 15.4].into_iter().enumerate() {
        tank(
            &mut s,
            Vec3::new(x, 0.0, -hz + 1.8),
            1.1,
            3.6 + i as f32 * 0.3,
        );
    }
    // Pumps by the flume's paddle end.
    for k in 0..3 {
        pump(&mut s, Vec3::new(-25.0 + k as f32 * 2.2, 0.0, -13.0));
        s.anchor(Anchor::Pump, Vec3::new(-25.0 + k as f32 * 2.2, 0.5, -13.0));
    }
    back_wall_pipes(&mut s, spec);
    cable_trays(&mut s, spec);

    // Electrical cabinets along the back wall.
    let mut x = -9.5;
    while x < -1.0 {
        cabinet(&mut s, &mut rng, Vec3::new(x, 0.0, -hz + 0.55), 0.0);
        x += 0.85;
    }
    // A second bank by the door end.
    let mut z = -12.0;
    while z < -7.0 {
        cabinet(
            &mut s,
            &mut rng,
            Vec3::new(hx - 0.55, 0.0, z),
            -std::f32::consts::FRAC_PI_2,
        );
        z += 0.85;
    }

    // Pallet racking along the front wall.
    for k in 0..4 {
        rack(
            &mut s,
            &mut rng,
            Vec3::new(-25.0 + k as f32 * 2.8, 0.0, hz - 1.0),
        );
    }
    // Workbenches, a tool chest and gas bottles.
    bench(&mut s, &mut rng, Vec3::new(13.0, 0.0, hz - 1.2));
    bench(&mut s, &mut rng, Vec3::new(16.2, 0.0, hz - 1.2));
    tool_chest(&mut s, Vec3::new(18.4, 0.0, hz - 1.0));
    gas_bottles(&mut s, Vec3::new(hx - 1.3, 0.0, hz - 7.5));
    // Stock: concrete test cubes, steel sections, timber.
    cube_stack(&mut s, Vec3::new(-16.0, 0.0, 5.5));
    cube_stack(&mut s, Vec3::new(-14.6, 0.0, 6.2));
    section_stillage(&mut s, Vec3::new(20.5, 0.0, 6.0));
    timber_stack(&mut s, Vec3::new(-20.5, 0.0, 9.5));
    scissor_lift(&mut s, Vec3::new(-19.0, 0.0, 2.5));
    pallet_truck(&mut s, Vec3::new(9.5, 0.0, 10.5), 0.6);
    // Safety railings round the rig bays, with gaps to walk in, and a few
    // cones where the reaction-frame bay is open.
    let v = |x: f32, z: f32| Vec3::new(x, 0.0, z);
    railing(&mut s, &[v(-14.4, -4.0), v(-14.4, 1.6), v(-11.0, 1.6)]);
    railing(&mut s, &[v(-7.0, 1.6), v(-1.6, 1.6), v(-1.6, -6.5)]);
    for k in 0..3 {
        cone(&mut s, Vec3::new(-10.4 + k as f32 * 1.5, 0.0, 1.8));
    }
    railing(
        &mut s,
        &[
            v(19.0, -5.0),
            v(25.0, -5.0),
            v(25.0, 1.0),
            v(19.0, 1.0),
            v(19.0, -1.0),
        ],
    );
    railing(
        &mut s,
        &[v(10.2, -6.0), v(13.8, -6.0), v(13.8, -2.0), v(11.5, -2.0)],
    );
    // Drain gratings along the wet side of the flume.
    floor_grating(
        &mut s,
        Vec3::new(-22.0, 0.0, -8.45),
        Vec3::new(4.0, 0.0, -8.15),
    );
    floor_grating(
        &mut s,
        Vec3::new(-26.0, 0.0, -12.0),
        Vec3::new(-25.7, 0.0, -8.15),
    );
    // Steel sections racked by the door end, and two silos outside the
    // flume's beach end.
    section_rack(&mut s, Vec3::new(hx - 1.2, 0.0, 10.8));
    tank(&mut s, Vec3::new(3.5, 0.0, -hz + 1.6), 0.8, 5.5);
    tank(&mut s, Vec3::new(5.4, 0.0, -hz + 1.6), 0.8, 5.5);

    s
}

fn pipe_run(s: &mut Scenery, mat: Mat, pts: &[Vec3], r: f32) {
    for w in pts.windows(2) {
        s.m(mat).cylinder(w[0], w[1], r, 10, false);
        // Flanges every few metres.
        let len = w[0].distance(w[1]);
        let n = (len / 3.0).floor() as i32;
        let dir = (w[1] - w[0]) / len.max(1e-6);
        for k in 1..=n {
            let p = w[0] + dir * (k as f32 * len / (n + 1) as f32);
            s.m(mat).ring(p, dir, r * 1.45, r * 0.5, r * 0.5, 12);
        }
    }
    for p in &pts[1..pts.len().saturating_sub(1)] {
        s.m(mat).sphere(*p, r * 1.02, 10, false);
    }
}

fn back_wall_pipes(s: &mut Scenery, spec: &HallSpec) {
    let (hx, hz) = spec.half();
    let z = -hz + 0.45;
    let runs = [
        (Mat::PipeGreen, 3.2, 0.12),
        (Mat::PipeBlue, 3.6, 0.09),
        (Mat::PipeRed, 4.0, 0.07),
        (Mat::PipeGrey, 5.6, 0.16),
    ];
    for (mat, y, r) in runs {
        pipe_run(
            s,
            mat,
            &[Vec3::new(-hx + 1.0, y, z), Vec3::new(hx - 1.0, y, z)],
            r,
        );
    }
    // Brackets.
    let mut x = -hx + 2.0;
    while x < hx - 1.0 {
        s.m(Mat::LightSteel).bar(
            Vec3::new(x, 2.9, z - 0.3),
            Vec3::new(x, 5.9, z - 0.3),
            0.06,
            0.06,
            Vec3::X,
        );
        for y in [3.0f32, 5.4] {
            s.m(Mat::LightSteel).bar(
                Vec3::new(x, y, -hz + 0.1),
                Vec3::new(x, y, z + 0.25),
                0.05,
                0.06,
                Vec3::Y,
            );
        }
        x += 3.0;
    }
    // Drops from the green main into the pumps and up from the tanks.
    for k in 0..3 {
        let x = -25.0 + k as f32 * 2.2 + 0.9;
        pipe_run(
            s,
            Mat::PipeGreen,
            &[
                Vec3::new(x, 3.2, z),
                Vec3::new(x, 1.2, z),
                Vec3::new(x, 0.45, -13.0),
            ],
            0.1,
        );
    }
    for x in [9.0f32, 12.2, 15.4] {
        pipe_run(
            s,
            Mat::PipeBlue,
            &[
                Vec3::new(x + 0.6, 3.6, z),
                Vec3::new(x + 0.6, 3.6, -hz + 1.8),
                Vec3::new(x + 0.6, 0.5, -hz + 1.8 + 0.9),
            ],
            0.07,
        );
    }
    // Flume return along the floor.
    pipe_run(
        s,
        Mat::PipeGreen,
        &[
            Vec3::new(-23.0, 0.3, -12.0),
            Vec3::new(5.5, 0.3, -12.0),
            Vec3::new(5.5, 0.3, -10.9),
        ],
        0.14,
    );
    // A pipe bridge across the hall high up, to the door end cabinets.
    pipe_run(
        s,
        Mat::PipeGrey,
        &[
            Vec3::new(hx - 3.0, 5.6, z),
            Vec3::new(hx - 3.0, 7.0, z),
            Vec3::new(hx - 3.0, 7.0, -4.0),
            Vec3::new(hx - 0.6, 7.0, -4.0),
        ],
        0.12,
    );
}

fn cable_trays(s: &mut Scenery, spec: &HallSpec) {
    let (hx, hz) = spec.half();
    for side in [-1.0f32, 1.0] {
        let z = side * (hz - 0.6);
        let y = 6.4;
        let lt = s.m(Mat::LightSteel);
        lt.bar(
            Vec3::new(-hx, y, z),
            Vec3::new(hx, y, z),
            0.45,
            0.02,
            Vec3::Y,
        );
        for dz in [-0.22f32, 0.22] {
            lt.bar(
                Vec3::new(-hx, y + 0.05, z + dz),
                Vec3::new(hx, y + 0.05, z + dz),
                0.02,
                0.1,
                Vec3::Y,
            );
        }
        s.m(Mat::Dark).bar(
            Vec3::new(-hx, y + 0.04, z),
            Vec3::new(hx, y + 0.04, z),
            0.3,
            0.05,
            Vec3::Y,
        );
    }
}

/// A glass-walled wave flume on a plinth with a piston paddle at `x0`.
fn wave_flume(s: &mut Scenery, x0: f32, x1: f32, zc: f32, width: f32, height: f32) {
    let plinth = 0.7;
    let hw = width * 0.5;
    s.m(Mat::Concrete).aabb(
        Vec3::new(x0 - 0.4, 0.0, zc - hw - 0.3),
        Vec3::new(x1 + 0.4, plinth, zc + hw + 0.3),
    );
    let floor = plinth + 0.05;
    s.m(Mat::FloorSteel).aabb(
        Vec3::new(x0, plinth, zc - hw),
        Vec3::new(x1, floor, zc + hw),
    );
    let top = floor + height;
    // Glass panels between steel frames.
    for side in [-1.0f32, 1.0] {
        let z = zc + side * hw;
        s.m(Mat::Glass).quad([
            Vec3::new(x0, floor, z),
            Vec3::new(x1, floor, z),
            Vec3::new(x1, top, z),
            Vec3::new(x0, top, z),
        ]);
        s.m(Mat::FrameSteel).bar(
            Vec3::new(x0, top, z),
            Vec3::new(x1, top, z),
            0.12,
            0.12,
            Vec3::Y,
        );
    }
    let mut x = x0;
    while x <= x1 + 0.01 {
        // A U-frame round the channel.
        let fs = s.m(Mat::FrameSteel);
        for side in [-1.0f32, 1.0] {
            let z = zc + side * (hw + 0.06);
            fs.bar(
                Vec3::new(x, plinth, z),
                Vec3::new(x, top + 0.06, z),
                0.1,
                0.12,
                Vec3::X,
            );
        }
        fs.bar(
            Vec3::new(x, top + 0.1, zc - hw - 0.1),
            Vec3::new(x, top + 0.1, zc + hw + 0.1),
            0.1,
            0.08,
            Vec3::Y,
        );
        x += 1.5;
    }
    // Water, with a gentle swell frozen in time.
    let wl = floor + height * 0.55;
    let n = ((x1 - x0) / 0.25) as i32;
    for k in 0..n {
        let xa = x0 + k as f32 * 0.25;
        let xb = xa + 0.25;
        let h = |x: f32| wl + 0.06 * ((x - x0) * 0.9).sin() * ((x1 - x) / (x1 - x0)).max(0.2);
        s.m(Mat::Water).quad([
            Vec3::new(xa, h(xa), zc + hw - 0.01),
            Vec3::new(xb, h(xb), zc + hw - 0.01),
            Vec3::new(xb, h(xb), zc - hw + 0.01),
            Vec3::new(xa, h(xa), zc - hw + 0.01),
        ]);
    }
    for side in [-1.0f32, 1.0] {
        let z = zc + side * (hw - 0.01);
        s.m(Mat::Water).quad([
            Vec3::new(x0, floor, z),
            Vec3::new(x1, floor, z),
            Vec3::new(x1, wl, z),
            Vec3::new(x0, wl, z),
        ]);
    }
    // Rubble absorber beach at the far end.
    s.m(Mat::Foam).quad([
        Vec3::new(x1 - 4.0, floor, zc - hw),
        Vec3::new(x1, top - 0.2, zc - hw),
        Vec3::new(x1, top - 0.2, zc + hw),
        Vec3::new(x1 - 4.0, floor, zc + hw),
    ]);
    // Paddle and its actuator frame.
    s.m(Mat::Stainless).aabb(
        Vec3::new(x0 + 0.3, floor, zc - hw + 0.05),
        Vec3::new(x0 + 0.38, top - 0.1, zc + hw - 0.05),
    );
    let fx = x0 - 1.6;
    s.m(Mat::MachineBlue).aabb(
        Vec3::new(fx - 0.6, plinth, zc - hw - 0.2),
        Vec3::new(fx + 0.6, plinth + 0.2, zc + hw + 0.2),
    );
    for side in [-1.0f32, 1.0] {
        let z = zc + side * 0.45;
        s.m(Mat::MachineBlue).bar(
            Vec3::new(fx, plinth + 0.2, z),
            Vec3::new(fx, top + 0.5, z),
            0.2,
            0.2,
            Vec3::X,
        );
        s.m(Mat::Dark).cylinder(
            Vec3::new(fx, floor + height * 0.5, z),
            Vec3::new(x0 - 0.4, floor + height * 0.5, z),
            0.11,
            12,
            true,
        );
        s.m(Mat::Stainless).cylinder(
            Vec3::new(x0 - 0.4, floor + height * 0.5, z),
            Vec3::new(x0 + 0.3, floor + height * 0.5, z),
            0.045,
            10,
            false,
        );
    }
    s.m(Mat::MachineBlue).bar(
        Vec3::new(fx, top + 0.5, zc - 0.6),
        Vec3::new(fx, top + 0.5, zc + 0.6),
        0.25,
        0.25,
        Vec3::Y,
    );
    // Wave gauges on a carriage.
    let gx = (x0 + x1) * 0.5;
    s.m(Mat::Yellow).aabb(
        Vec3::new(gx - 0.3, top + 0.12, zc - hw - 0.15),
        Vec3::new(gx + 0.3, top + 0.3, zc + hw + 0.15),
    );
    for dz in [-0.3f32, 0.0, 0.3] {
        s.m(Mat::Stainless).cylinder(
            Vec3::new(gx, top + 0.12, zc + dz),
            Vec3::new(gx, floor + 0.3, zc + dz),
            0.008,
            5,
            false,
        );
    }
    s.m(Mat::IndicatorGreen).sphere(
        Vec3::new(gx + 0.25, top + 0.33, zc + hw + 0.1),
        0.03,
        8,
        false,
    );
}

/// A heavy steel portal for beam tests with a vertical actuator.
fn reaction_frame(s: &mut Scenery, at: Vec3) {
    let span = 5.0;
    let h = 4.2;
    for dz in [-0.6f32, 0.6] {
        for dx in [-span * 0.5, span * 0.5] {
            s.m(Mat::MachineBlue).i_beam(
                at + Vec3::new(dx, 0.0, dz),
                at + Vec3::new(dx, h, dz),
                0.5,
                0.35,
                0.025,
                0.04,
                Vec3::X,
            );
        }
        s.m(Mat::MachineBlue).i_beam(
            at + Vec3::new(-span * 0.5 - 0.3, h + 0.35, dz),
            at + Vec3::new(span * 0.5 + 0.3, h + 0.35, dz),
            0.7,
            0.35,
            0.025,
            0.04,
            Vec3::Y,
        );
    }
    // Spreader and actuator.
    s.m(Mat::MachineBlue).aabb(
        at + Vec3::new(-0.4, h - 0.1, -0.8),
        at + Vec3::new(0.4, h, 0.8),
    );
    s.m(Mat::MachineOrange).cylinder(
        at + Vec3::new(0.0, h - 0.1, 0.0),
        at + Vec3::new(0.0, h - 1.4, 0.0),
        0.22,
        16,
        true,
    );
    s.m(Mat::Stainless).cylinder(
        at + Vec3::new(0.0, h - 1.4, 0.0),
        at + Vec3::new(0.0, 1.35, 0.0),
        0.08,
        12,
        false,
    );
    s.m(Mat::Dark).cylinder(
        at + Vec3::new(0.0, 1.35, 0.0),
        at + Vec3::new(0.0, 1.15, 0.0),
        0.18,
        12,
        true,
    );
    // Hoses.
    for dz in [-0.15f32, 0.15] {
        pipe_run(
            s,
            Mat::Dark,
            &[
                at + Vec3::new(0.25, h - 0.5, dz),
                at + Vec3::new(0.9, h - 0.8, dz),
                at + Vec3::new(1.4, 0.15, dz),
                at + Vec3::new(5.3, 0.06, -2.5 + dz),
            ],
            0.025,
        );
    }
    // Specimen: a concrete beam on two roller supports.
    for dx in [-1.8f32, 1.8] {
        s.m(Mat::MachineGrey).aabb(
            at + Vec3::new(dx - 0.25, 0.0, -0.3),
            at + Vec3::new(dx + 0.25, 0.6, 0.3),
        );
        s.m(Mat::Stainless).cylinder(
            at + Vec3::new(dx, 0.66, -0.25),
            at + Vec3::new(dx, 0.66, 0.25),
            0.06,
            10,
            true,
        );
    }
    s.m(Mat::Concrete).aabb(
        at + Vec3::new(-2.2, 0.72, -0.15),
        at + Vec3::new(2.2, 1.12, 0.15),
    );
    // Displacement transducer stand.
    s.m(Mat::Dark).cylinder(
        at + Vec3::new(0.5, 0.0, 0.5),
        at + Vec3::new(0.5, 0.7, 0.5),
        0.02,
        6,
        false,
    );
    s.m(Mat::Yellow).aabb(
        at + Vec3::new(0.35, 0.0, 0.35),
        at + Vec3::new(0.65, 0.04, 0.65),
    );
}

/// A tall loading frame for column tests.
fn column_rig(s: &mut Scenery, at: Vec3) {
    let h = 6.0;
    for (dx, dz) in [(-0.9f32, -0.9f32), (0.9, -0.9), (-0.9, 0.9), (0.9, 0.9)] {
        s.m(Mat::MachineOrange).cylinder(
            at + Vec3::new(dx, 0.0, dz),
            at + Vec3::new(dx, h, dz),
            0.12,
            12,
            false,
        );
        s.m(Mat::Dark).cylinder(
            at + Vec3::new(dx, h, dz),
            at + Vec3::new(dx, h + 0.15, dz),
            0.17,
            6,
            true,
        );
    }
    s.m(Mat::MachineOrange).aabb(
        at + Vec3::new(-1.2, h - 0.6, -1.2),
        at + Vec3::new(1.2, h, 1.2),
    );
    s.m(Mat::MachineOrange).aabb(
        at + Vec3::new(-1.2, 0.0, -1.2),
        at + Vec3::new(1.2, 0.5, 1.2),
    );
    s.m(Mat::Stainless).cylinder(
        at + Vec3::new(0.0, h - 0.6, 0.0),
        at + Vec3::new(0.0, h - 1.2, 0.0),
        0.35,
        16,
        true,
    );
    // Specimen column, steel, mid-test.
    s.m(Mat::FrameSteel).i_beam(
        at + Vec3::new(0.0, 0.5, 0.0),
        at + Vec3::new(0.0, h - 1.2, 0.0),
        0.25,
        0.25,
        0.01,
        0.015,
        Vec3::Z,
    );
    s.m(Mat::IndicatorRed)
        .sphere(at + Vec3::new(1.25, 0.4, 1.25), 0.035, 8, false);
}

/// A small seismic platform on actuators.
fn shake_table(s: &mut Scenery, at: Vec3) {
    s.m(Mat::Concrete).aabb(
        at + Vec3::new(-2.2, 0.0, -2.2),
        at + Vec3::new(2.2, 0.25, 2.2),
    );
    s.m(Mat::MachineBlue).aabb(
        at + Vec3::new(-1.5, 0.55, -1.5),
        at + Vec3::new(1.5, 0.7, 1.5),
    );
    let mut x = -1.25;
    while x < 1.3 {
        let mut z = -1.25;
        while z < 1.3 {
            s.m(Mat::Dark)
                .disc(at + Vec3::new(x, 0.702, z), Vec3::Y, 0.03, 6);
            z += 0.25;
        }
        x += 0.25;
    }
    for (dx, dz) in [(-1.0f32, -1.0f32), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)] {
        s.m(Mat::Stainless).cylinder(
            at + Vec3::new(dx, 0.25, dz),
            at + Vec3::new(dx, 0.55, dz),
            0.1,
            10,
            false,
        );
    }
    for side in [-1.0f32, 1.0] {
        s.m(Mat::MachineOrange).cylinder(
            at + Vec3::new(side * 2.0, 0.45, 0.0),
            at + Vec3::new(side * 1.52, 0.6, 0.0),
            0.12,
            12,
            true,
        );
    }
    // A test model left on it: a little steel frame.
    for (dx, dz) in [(-0.6f32, -0.4f32), (0.6, -0.4), (-0.6, 0.4), (0.6, 0.4)] {
        s.m(Mat::LightSteel).bar(
            at + Vec3::new(dx, 0.7, dz),
            at + Vec3::new(dx, 2.0, dz),
            0.05,
            0.05,
            Vec3::X,
        );
    }
    for y in [1.35f32, 2.0] {
        s.m(Mat::Timber).aabb(
            at + Vec3::new(-0.7, y, -0.5),
            at + Vec3::new(0.7, y + 0.06, 0.5),
        );
    }
}

/// A hydraulic power unit: tank, motor-pump, filters, gauges.
fn power_unit(s: &mut Scenery, at: Vec3, rot: Quat) {
    let p = |v: Vec3| at + rot * v;
    s.m(Mat::MachineGreen)
        .cuboid(p(Vec3::new(0.0, 0.5, 0.0)), Vec3::new(1.1, 0.5, 0.6), rot);
    s.m(Mat::Dark).cuboid(
        p(Vec3::new(0.0, 0.04, 0.0)),
        Vec3::new(1.15, 0.04, 0.65),
        rot,
    );
    s.m(Mat::MachineBlue).cylinder(
        p(Vec3::new(-0.7, 1.35, 0.0)),
        p(Vec3::new(0.2, 1.35, 0.0)),
        0.3,
        16,
        true,
    );
    s.m(Mat::Dark).cylinder(
        p(Vec3::new(0.2, 1.35, 0.0)),
        p(Vec3::new(0.6, 1.35, 0.0)),
        0.18,
        12,
        true,
    );
    for k in 0..3 {
        s.m(Mat::Stainless).cylinder(
            p(Vec3::new(0.7 + k as f32 * 0.14, 1.0, 0.35)),
            p(Vec3::new(0.7 + k as f32 * 0.14, 1.45, 0.35)),
            0.05,
            10,
            true,
        );
    }
    // Control box with lamps.
    s.m(Mat::MachineGrey).cuboid(
        p(Vec3::new(-0.9, 1.4, 0.45)),
        Vec3::new(0.25, 0.3, 0.12),
        rot,
    );
    s.m(Mat::IndicatorGreen)
        .sphere(p(Vec3::new(-1.0, 1.55, 0.58)), 0.025, 8, false);
    s.m(Mat::IndicatorAmber)
        .sphere(p(Vec3::new(-0.85, 1.55, 0.58)), 0.025, 8, false);
    for k in 0..2 {
        s.m(Mat::Stainless).disc(
            p(Vec3::new(-0.2 + k as f32 * 0.3, 0.75, 0.61)),
            rot * Vec3::Z,
            0.07,
            12,
        );
    }
}

fn tank(s: &mut Scenery, at: Vec3, r: f32, h: f32) {
    for k in 0..4 {
        let a = k as f32 * std::f32::consts::FRAC_PI_2 + 0.785;
        let d = Vec3::new(a.cos(), 0.0, a.sin()) * r * 0.75;
        s.m(Mat::LightSteel)
            .cylinder(at + d, at + d + Vec3::Y * 0.8, 0.06, 8, false);
    }
    s.m(Mat::Stainless)
        .cylinder(at + Vec3::Y * 0.7, at + Vec3::Y * h, r, 24, false);
    s.m(Mat::Stainless).sphere(at + Vec3::Y * h, r, 24, true);
    s.m(Mat::Stainless)
        .disc(at + Vec3::Y * 0.7, -Vec3::Y, r, 24);
    for y in [1.6f32, h - 0.6] {
        s.m(Mat::LightSteel)
            .ring(at + Vec3::Y * y, Vec3::Y, r + 0.03, 0.04, 0.06, 24);
    }
    // Ladder.
    let lx = at + Vec3::new(0.0, 0.0, r + 0.25);
    for dx in [-0.22f32, 0.22] {
        s.m(Mat::Yellow).cylinder(
            lx + Vec3::new(dx, 0.0, 0.0),
            lx + Vec3::new(dx, h + 0.8, 0.0),
            0.02,
            6,
            false,
        );
    }
    let mut y = 0.3;
    while y < h + 0.6 {
        s.m(Mat::Yellow).cylinder(
            lx + Vec3::new(-0.22, y, 0.0),
            lx + Vec3::new(0.22, y, 0.0),
            0.015,
            5,
            false,
        );
        y += 0.3;
    }
}

fn pump(s: &mut Scenery, at: Vec3) {
    s.m(Mat::Dark).aabb(
        at + Vec3::new(-0.8, 0.0, -0.35),
        at + Vec3::new(0.8, 0.15, 0.35),
    );
    s.m(Mat::MachineBlue).cylinder(
        at + Vec3::new(-0.75, 0.45, 0.0),
        at + Vec3::new(0.05, 0.45, 0.0),
        0.26,
        16,
        true,
    );
    for k in 0..6 {
        let x = -0.7 + k as f32 * 0.12;
        s.m(Mat::MachineBlue)
            .ring(at + Vec3::new(x, 0.45, 0.0), Vec3::X, 0.29, 0.04, 0.03, 16);
    }
    s.m(Mat::PipeGreen).cylinder(
        at + Vec3::new(0.3, 0.45, -0.22),
        at + Vec3::new(0.3, 0.45, 0.22),
        0.3,
        18,
        true,
    );
    s.m(Mat::PipeGreen).cylinder(
        at + Vec3::new(0.3, 0.45, 0.0),
        at + Vec3::new(0.9, 0.45, 0.0),
        0.1,
        10,
        false,
    );
}

fn cabinet(s: &mut Scenery, rng: &mut Rng, at: Vec3, yaw: f32) {
    let rot = Quat::from_rotation_y(yaw);
    let h = rng.range(1.9, 2.1);
    s.m(Mat::MachineGrey).cuboid(
        at + Vec3::Y * (h * 0.5 + 0.1),
        Vec3::new(0.4, h * 0.5, 0.3),
        rot,
    );
    s.m(Mat::Dark)
        .cuboid(at + Vec3::Y * 0.05, Vec3::new(0.4, 0.05, 0.3), rot);
    let front = rot * Vec3::Z;
    let side = rot * Vec3::X;
    // Door seam and handle.
    s.m(Mat::Dark).cuboid(
        at + Vec3::Y * (h * 0.55) + front * 0.305 + side * 0.25,
        Vec3::new(0.015, 0.12, 0.01),
        rot,
    );
    let lamps = [
        Mat::IndicatorGreen,
        Mat::IndicatorGreen,
        Mat::IndicatorAmber,
        Mat::IndicatorRed,
    ];
    for k in 0..rng.range(1.0, 3.99) as i32 {
        let m = rng.pick(&lamps);
        s.m(m).sphere(
            at + Vec3::Y * (h - 0.15) + front * 0.31 + side * (-0.25 + k as f32 * 0.09),
            0.018,
            8,
            false,
        );
    }
}

fn rack(s: &mut Scenery, rng: &mut Rng, at: Vec3) {
    let (w, d) = (2.7, 1.1);
    let levels = [0.15f32, 1.8, 3.4, 5.0];
    for dx in [0.0f32, w] {
        for dz in [-d * 0.5, d * 0.5] {
            s.m(Mat::MachineBlue).bar(
                at + Vec3::new(dx, 0.0, dz),
                at + Vec3::new(dx, 6.0, dz),
                0.09,
                0.09,
                Vec3::X,
            );
        }
        let mut y = 0.3;
        while y < 5.8 {
            s.m(Mat::MachineBlue).cylinder(
                at + Vec3::new(dx, y, -d * 0.5),
                at + Vec3::new(dx, y + 0.5, d * 0.5),
                0.015,
                5,
                false,
            );
            y += 0.6;
        }
    }
    for &y in &levels[1..] {
        for dz in [-d * 0.5, d * 0.5] {
            s.m(Mat::MachineOrange).bar(
                at + Vec3::new(0.0, y, dz),
                at + Vec3::new(w, y, dz),
                0.05,
                0.12,
                Vec3::Y,
            );
        }
    }
    for &y in &levels {
        for k in 0..2 {
            if rng.chance(0.15) {
                continue;
            }
            let x = 0.15 + k as f32 * 1.3;
            let loaded = y < 1.0 || rng.chance(0.8);
            pallet(s, rng, at + Vec3::new(x + 0.6, y + 0.06, 0.0), loaded);
        }
    }
}

fn pallet(s: &mut Scenery, rng: &mut Rng, c: Vec3, loaded: bool) {
    s.m(Mat::Timber).aabb(
        c + Vec3::new(-0.6, 0.0, -0.5),
        c + Vec3::new(0.6, 0.14, 0.5),
    );
    if !loaded {
        return;
    }
    let mats = [
        Mat::Cardboard,
        Mat::Cardboard,
        Mat::MachineGrey,
        Mat::Timber,
        Mat::MachineBlue,
    ];
    let mat = rng.pick(&mats);
    let h = rng.range(0.4, 1.3);
    if mat == Mat::MachineBlue {
        // A drum or two.
        for dx in [-0.3f32, 0.3] {
            s.m(mat).cylinder(
                c + Vec3::new(dx, 0.14, 0.0),
                c + Vec3::new(dx, 1.0, 0.0),
                0.28,
                14,
                true,
            );
        }
        return;
    }
    let shrink = Vec3::new(rng.range(0.0, 0.15), 0.0, rng.range(0.0, 0.1));
    s.m(mat).aabb(
        c + Vec3::new(-0.58, 0.14, -0.48) + shrink,
        c + Vec3::new(0.58, 0.14 + h, 0.48) - shrink,
    );
}

fn bench(s: &mut Scenery, rng: &mut Rng, at: Vec3) {
    let (w, d, h) = (3.0, 0.8, 0.9);
    s.m(Mat::Timber).aabb(
        at + Vec3::new(-w * 0.5, h - 0.05, -d * 0.5),
        at + Vec3::new(w * 0.5, h, d * 0.5),
    );
    for (dx, dz) in [(-1.0f32, -1.0f32), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)] {
        let p = at + Vec3::new(dx * (w * 0.5 - 0.05), 0.0, dz * (d * 0.5 - 0.05));
        s.m(Mat::MachineGreen)
            .bar(p, p + Vec3::Y * (h - 0.05), 0.05, 0.05, Vec3::X);
    }
    s.m(Mat::MachineGreen).aabb(
        at + Vec3::new(-w * 0.5, 0.15, -d * 0.5),
        at + Vec3::new(w * 0.5, 0.18, d * 0.5),
    );
    // Pegboard and clutter.
    s.m(Mat::Cardboard).aabb(
        at + Vec3::new(-w * 0.5, h, d * 0.5 - 0.02),
        at + Vec3::new(w * 0.5, h + 1.0, d * 0.5),
    );
    for _ in 0..6 {
        let x = rng.range(-w * 0.45, w * 0.45);
        let z = rng.range(-d * 0.35, d * 0.2);
        let m = rng.pick(&[
            Mat::Dark,
            Mat::MachineOrange,
            Mat::Stainless,
            Mat::MachineBlue,
        ]);
        let sz = Vec3::new(
            rng.range(0.05, 0.25),
            rng.range(0.03, 0.2),
            rng.range(0.05, 0.2),
        );
        s.m(m).aabb(
            at + Vec3::new(x, h, z) - Vec3::new(sz.x, 0.0, sz.z) * 0.5,
            at + Vec3::new(x, h, z) + Vec3::new(sz.x * 0.5, sz.y, sz.z * 0.5),
        );
    }
    // A vice.
    s.m(Mat::MachineBlue).aabb(
        at + Vec3::new(w * 0.5 - 0.3, h, -d * 0.5),
        at + Vec3::new(w * 0.5 - 0.1, h + 0.18, -d * 0.5 + 0.25),
    );
}

fn tool_chest(s: &mut Scenery, at: Vec3) {
    s.m(Mat::MachineOrange).aabb(
        at + Vec3::new(-0.4, 0.1, -0.3),
        at + Vec3::new(0.4, 1.1, 0.3),
    );
    for k in 0..6 {
        let y = 0.2 + k as f32 * 0.15;
        s.m(Mat::Dark).aabb(
            at + Vec3::new(-0.36, y, -0.31),
            at + Vec3::new(0.36, y + 0.012, -0.3),
        );
    }
    for (dx, dz) in [
        (-0.33f32, -0.25f32),
        (0.33, -0.25),
        (-0.33, 0.25),
        (0.33, 0.25),
    ] {
        s.m(Mat::Dark).cylinder(
            at + Vec3::new(dx - 0.03, 0.05, dz),
            at + Vec3::new(dx + 0.03, 0.05, dz),
            0.05,
            8,
            true,
        );
    }
}

fn gas_bottles(s: &mut Scenery, at: Vec3) {
    s.m(Mat::LightSteel).aabb(
        at + Vec3::new(-0.2, 0.0, -1.2),
        at + Vec3::new(0.2, 1.2, -1.15),
    );
    let colours = [
        Mat::MachineGrey,
        Mat::MachineBlue,
        Mat::PipeRed,
        Mat::MachineGreen,
        Mat::Dark,
    ];
    for (k, m) in colours.into_iter().enumerate() {
        let c = at + Vec3::new(0.0, 0.0, -0.9 + k as f32 * 0.42);
        s.m(m).cylinder(c, c + Vec3::Y * 1.4, 0.115, 12, false);
        s.m(m).sphere(c + Vec3::Y * 1.4, 0.115, 12, true);
        s.m(Mat::Stainless)
            .cylinder(c + Vec3::Y * 1.5, c + Vec3::Y * 1.62, 0.03, 6, true);
    }
    s.m(Mat::Yellow).cylinder(
        at + Vec3::new(0.2, 0.9, -1.15),
        at + Vec3::new(0.2, 0.9, 1.2),
        0.015,
        5,
        false,
    );
}

fn cube_stack(s: &mut Scenery, at: Vec3) {
    let c = 0.15;
    for i in 0..4 {
        for j in 0..3 {
            for k in 0..(3 - j / 2) {
                let p = at + Vec3::new(i as f32 * 0.17, k as f32 * 0.155, j as f32 * 0.17);
                s.m(Mat::Concrete).aabb(p, p + Vec3::splat(c));
            }
        }
    }
    s.m(Mat::Timber).aabb(
        at + Vec3::new(-0.1, -0.0, -0.1),
        at + Vec3::new(0.75, 0.0001, 0.6),
    );
}

fn section_stillage(s: &mut Scenery, at: Vec3) {
    s.m(Mat::MachineOrange).aabb(
        at + Vec3::new(-2.0, 0.0, -0.5),
        at + Vec3::new(2.0, 0.15, 0.5),
    );
    for dx in [-1.9f32, 0.0, 1.9] {
        for dz in [-0.5f32, 0.5] {
            s.m(Mat::MachineOrange).bar(
                at + Vec3::new(dx, 0.15, dz),
                at + Vec3::new(dx, 1.0, dz),
                0.06,
                0.06,
                Vec3::X,
            );
        }
    }
    for layer in 0..3 {
        for k in 0..4 {
            let y = 0.25 + layer as f32 * 0.22;
            let z = -0.35 + k as f32 * 0.23;
            s.m(Mat::FrameSteel).i_beam(
                at + Vec3::new(-2.6, y, z),
                at + Vec3::new(2.6, y, z),
                0.2,
                0.1,
                0.008,
                0.012,
                Vec3::Y,
            );
        }
    }
}

fn timber_stack(s: &mut Scenery, at: Vec3) {
    for layer in 0..5 {
        s.m(Mat::Dark).aabb(
            at + Vec3::new(-1.8, layer as f32 * 0.2, -0.6),
            at + Vec3::new(-1.7, layer as f32 * 0.2 + 0.04, 0.6),
        );
        s.m(Mat::Dark).aabb(
            at + Vec3::new(1.7, layer as f32 * 0.2, -0.6),
            at + Vec3::new(1.8, layer as f32 * 0.2 + 0.04, 0.6),
        );
        for k in 0..8 {
            let z = -0.56 + k as f32 * 0.15;
            let y = layer as f32 * 0.2 + 0.04;
            s.m(Mat::Timber).aabb(
                at + Vec3::new(-2.2, y, z),
                at + Vec3::new(2.2, y + 0.15, z + 0.13),
            );
        }
    }
}

fn scissor_lift(s: &mut Scenery, at: Vec3) {
    s.m(Mat::Yellow).aabb(
        at + Vec3::new(-1.1, 0.12, -0.4),
        at + Vec3::new(1.1, 0.5, 0.4),
    );
    for (dx, dz) in [(-0.8f32, -0.42f32), (0.8, -0.42), (-0.8, 0.42), (0.8, 0.42)] {
        s.m(Mat::Dark).cylinder(
            at + Vec3::new(dx, 0.15, dz - 0.06),
            at + Vec3::new(dx, 0.15, dz + 0.06),
            0.15,
            12,
            true,
        );
    }
    for k in 0..3 {
        let y0 = 0.5 + k as f32 * 0.25;
        for dz in [-0.38f32, 0.38] {
            s.m(Mat::Dark).bar(
                at + Vec3::new(-0.9, y0, dz),
                at + Vec3::new(0.9, y0 + 0.25, dz),
                0.04,
                0.06,
                Vec3::Y,
            );
            s.m(Mat::Dark).bar(
                at + Vec3::new(0.9, y0, dz),
                at + Vec3::new(-0.9, y0 + 0.25, dz),
                0.04,
                0.06,
                Vec3::Y,
            );
        }
    }
    let deck = 1.25;
    s.m(Mat::Yellow).aabb(
        at + Vec3::new(-1.1, deck, -0.42),
        at + Vec3::new(1.1, deck + 0.08, 0.42),
    );
    for (dx, dz) in [
        (-1.05f32, -0.38f32),
        (1.05, -0.38),
        (-1.05, 0.38),
        (1.05, 0.38),
    ] {
        s.m(Mat::Yellow).cylinder(
            at + Vec3::new(dx, deck, dz),
            at + Vec3::new(dx, deck + 1.1, dz),
            0.02,
            6,
            false,
        );
    }
    for dz in [-0.38f32, 0.38] {
        s.m(Mat::Yellow).cylinder(
            at + Vec3::new(-1.05, deck + 1.1, dz),
            at + Vec3::new(1.05, deck + 1.1, dz),
            0.02,
            6,
            false,
        );
    }
}

fn pallet_truck(s: &mut Scenery, at: Vec3, yaw: f32) {
    let r = Quat::from_rotation_y(yaw);
    let p = |v: Vec3| at + r * v;
    for dz in [-0.2f32, 0.2] {
        s.m(Mat::MachineOrange)
            .cuboid(p(Vec3::new(0.0, 0.05, dz)), Vec3::new(0.6, 0.04, 0.08), r);
    }
    s.m(Mat::MachineOrange)
        .cuboid(p(Vec3::new(-0.7, 0.2, 0.0)), Vec3::new(0.12, 0.18, 0.3), r);
    s.m(Mat::Dark).cylinder(
        p(Vec3::new(-0.75, 0.35, 0.0)),
        p(Vec3::new(-1.3, 1.2, 0.0)),
        0.02,
        6,
        false,
    );
    s.m(Mat::Dark).cylinder(
        p(Vec3::new(-1.3, 1.2, -0.12)),
        p(Vec3::new(-1.3, 1.2, 0.12)),
        0.025,
        6,
        true,
    );
}

/// A yellow two-rail guard rail along a polyline on the floor.
fn railing(s: &mut Scenery, pts: &[Vec3]) {
    for w in pts.windows(2) {
        let (a, b) = (w[0], w[1]);
        let n = (a.distance(b) / 1.5).ceil().max(1.0) as i32;
        for k in 0..=n {
            let p = a.lerp(b, k as f32 / n as f32);
            s.m(Mat::Yellow)
                .cylinder(p, p + Vec3::Y * 1.1, 0.03, 8, false);
            s.m(Mat::Dark).aabb(
                p - Vec3::new(0.08, 0.0, 0.08),
                p + Vec3::new(0.08, 0.02, 0.08),
            );
        }
        for h in [0.5f32, 1.1] {
            s.m(Mat::Yellow)
                .cylinder(a + Vec3::Y * h, b + Vec3::Y * h, 0.025, 8, false);
        }
        // Kick plate.
        s.m(Mat::Yellow)
            .bar(a + Vec3::Y * 0.08, b + Vec3::Y * 0.08, 0.01, 0.1, Vec3::Y);
    }
}

/// A floor drain: a dark channel with a slatted grating, `a` to `b` corners.
fn floor_grating(s: &mut Scenery, a: Vec3, b: Vec3) {
    let (min, max) = (a.min(b), a.max(b));
    s.m(Mat::Dark).aabb(
        Vec3::new(min.x, -0.01, min.z),
        Vec3::new(max.x, 0.003, max.z),
    );
    let along_x = (max.x - min.x) > (max.z - min.z);
    let len = if along_x {
        max.x - min.x
    } else {
        max.z - min.z
    };
    let n = (len / 0.05) as i32;
    for k in 0..n {
        let t = min + (max - min) * (k as f32 + 0.5) / n as f32;
        let (p, q) = if along_x {
            (
                Vec3::new(t.x - 0.01, 0.0, min.z),
                Vec3::new(t.x + 0.01, 0.008, max.z),
            )
        } else {
            (
                Vec3::new(min.x, 0.0, t.z - 0.01),
                Vec3::new(max.x, 0.008, t.z + 0.01),
            )
        };
        s.m(Mat::LightSteel).aabb(p, q);
    }
}

/// A cantilever rack of long steel sections against a wall.
fn section_rack(s: &mut Scenery, at: Vec3) {
    for k in 0..3 {
        let z = at.z - 2.8 + k as f32 * 2.8;
        s.m(Mat::MachineBlue).i_beam(
            Vec3::new(at.x, 0.0, z),
            Vec3::new(at.x, 4.0, z),
            0.3,
            0.2,
            0.01,
            0.015,
            Vec3::X,
        );
        for y in [0.8f32, 1.8, 2.8] {
            s.m(Mat::MachineBlue).bar(
                Vec3::new(at.x, y, z),
                Vec3::new(at.x - 1.0, y + 0.05, z),
                0.1,
                0.12,
                Vec3::Y,
            );
        }
    }
    for (layer, y) in [0.86f32, 1.86, 2.86].into_iter().enumerate() {
        for k in 0..(5 - layer) {
            let x = at.x - 0.2 - k as f32 * 0.17;
            let mat = if layer == 1 {
                Mat::LightSteel
            } else {
                Mat::FrameSteel
            };
            s.m(mat).i_beam(
                Vec3::new(x, y + 0.1, at.z - 3.6),
                Vec3::new(x, y + 0.1, at.z + 3.6),
                0.16,
                0.08,
                0.006,
                0.009,
                Vec3::Y,
            );
        }
    }
}

fn cone(s: &mut Scenery, at: Vec3) {
    s.m(Mat::Dark).aabb(
        at + Vec3::new(-0.18, 0.0, -0.18),
        at + Vec3::new(0.18, 0.03, 0.18),
    );
    s.m(Mat::MachineOrange).frustum(
        at + Vec3::Y * 0.03,
        at + Vec3::Y * 0.7,
        0.14,
        0.02,
        12,
        false,
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dressing_is_deterministic() {
        let h = HallSpec::default();
        let a = build(&h, 7);
        let b = build(&h, 7);
        assert_eq!(a.triangles(), b.triangles());
        assert_eq!(
            a.bufs[&Mat::Cardboard].positions,
            b.bufs[&Mat::Cardboard].positions
        );
    }
}
