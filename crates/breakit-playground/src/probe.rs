//! What the time-series graph follows: one number from a picked member or
//! node, sampled every frame into a rolling window.

use std::collections::VecDeque;

use crate::Live;

/// A picked member or node and the quantity to plot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Probe {
    /// Utilisation, 1 = at capacity.
    Utilization(usize),
    /// Axial force, kN, tension positive.
    Axial(usize),
    /// Largest end moment, kN·m.
    Moment(usize),
    /// Displacement from the start, mm.
    Displacement(usize),
    /// Vertical displacement, mm (up positive).
    Sag(usize),
    /// Speed, m/s.
    Speed(usize),
}

impl Probe {
    /// The member probes in the order the graph cycles through them.
    pub fn member(m: usize) -> [Probe; 3] {
        [Probe::Utilization(m), Probe::Axial(m), Probe::Moment(m)]
    }

    /// The node probes in the order the graph cycles through them.
    pub fn node(n: usize) -> [Probe; 3] {
        [Probe::Displacement(n), Probe::Sag(n), Probe::Speed(n)]
    }

    /// The next quantity for the same member or node.
    pub fn next(self) -> Probe {
        let list = match self {
            Probe::Utilization(m) | Probe::Axial(m) | Probe::Moment(m) => Probe::member(m),
            Probe::Displacement(n) | Probe::Sag(n) | Probe::Speed(n) => Probe::node(n),
        };
        let i = list.iter().position(|p| *p == self).unwrap_or(0);
        list[(i + 1) % list.len()]
    }

    pub fn label(self) -> String {
        match self {
            Probe::Utilization(m) => format!("Member {m} utilisation"),
            Probe::Axial(m) => format!("Member {m} axial force"),
            Probe::Moment(m) => format!("Member {m} end moment"),
            Probe::Displacement(n) => format!("Node {n} displacement"),
            Probe::Sag(n) => format!("Node {n} vertical movement"),
            Probe::Speed(n) => format!("Node {n} speed"),
        }
    }

    pub fn unit(self) -> &'static str {
        match self {
            Probe::Utilization(_) => "%",
            Probe::Axial(_) => "kN",
            Probe::Moment(_) => "kNm",
            Probe::Displacement(_) | Probe::Sag(_) => "mm",
            Probe::Speed(_) => "m/s",
        }
    }

    /// The value now, in [`Self::unit`]. `None` if the index is out of range.
    pub fn read(self, live: &Live) -> Option<f64> {
        let sim = live.sim();
        let members = live.structure().members.len();
        let nodes = sim.node_count();
        Some(match self {
            Probe::Utilization(m) if m < members => {
                if sim.is_broken(m) {
                    0.0
                } else {
                    sim.utilization(m) * 100.0
                }
            }
            Probe::Axial(m) if m < members => sim.damage(m).axial / 1000.0,
            Probe::Moment(m) if m < members => {
                let mo = sim.damage(m).moment;
                mo.iter().flatten().map(|v| v.abs()).fold(0.0, f64::max) / 1000.0
            }
            Probe::Displacement(n) if n < nodes => sim.displacement(n).norm() * 1000.0,
            Probe::Sag(n) if n < nodes => sim.displacement(n).y * 1000.0,
            Probe::Speed(n) if n < nodes => sim.velocity(n).norm(),
            _ => return None,
        })
    }
}

/// Rolling samples of one probe.
#[derive(Clone, Debug)]
pub struct Trace {
    pub probe: Probe,
    /// Seconds of history kept.
    pub window: f64,
    samples: VecDeque<(f64, f64)>,
}

impl Trace {
    pub fn new(probe: Probe, window: f64) -> Self {
        Self {
            probe,
            window,
            samples: VecDeque::new(),
        }
    }

    /// Reads the probe and keeps the sample; drops samples older than the
    /// window, and everything if time went backwards (a reset).
    pub fn sample(&mut self, live: &Live) {
        if let Some(v) = self.probe.read(live) {
            self.push(live.time(), v);
        }
    }

    pub fn push(&mut self, t: f64, v: f64) {
        match self.samples.back() {
            Some(&(t0, _)) if t < t0 => self.samples.clear(),
            // Paused: the same instant again.
            Some(&(t0, _)) if t == t0 => {
                self.samples.pop_back();
            }
            _ => {}
        }
        self.samples.push_back((t, v));
        while self
            .samples
            .front()
            .is_some_and(|&(t0, _)| t - t0 > self.window)
        {
            self.samples.pop_front();
        }
    }

    pub fn samples(&self) -> impl ExactSizeIterator<Item = &(f64, f64)> {
        self.samples.iter()
    }

    pub fn last(&self) -> Option<f64> {
        self.samples.back().map(|s| s.1)
    }

    /// Smallest and largest value in the window, widened to include 0.
    pub fn range(&self) -> (f64, f64) {
        self.samples
            .iter()
            .fold((0.0f64, 0.0f64), |(lo, hi), &(_, v)| (lo.min(v), hi.max(v)))
    }

    /// `columns` values across the window ending now: for each column the
    /// sample with the largest magnitude in it, or `None` where there is no
    /// sample yet.
    pub fn columns(&self, columns: usize) -> Vec<Option<f64>> {
        let mut out = vec![None; columns];
        let Some(&(t1, _)) = self.samples.back() else {
            return out;
        };
        let t0 = t1 - self.window;
        for &(t, v) in &self.samples {
            let c = (((t - t0) / self.window) * columns as f64) as usize;
            let c = c.min(columns - 1);
            let cur: Option<f64> = out[c];
            if cur.is_none_or(|x| v.abs() > x.abs()) {
                out[c] = Some(v);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trace_keeps_its_window_and_clears_on_reset() {
        let mut t = Trace::new(Probe::Sag(0), 2.0);
        for i in 0..300 {
            t.push(i as f64 / 60.0, -(i as f64));
        }
        assert!(t.samples().len() <= 121);
        assert_eq!(t.range().0, -299.0);
        assert_eq!(t.range().1, 0.0);
        let cols = t.columns(10);
        assert!(cols.iter().all(|c| c.is_some()));
        t.push(0.0, 1.0);
        assert_eq!(t.samples().len(), 1);
    }

    #[test]
    fn probes_cycle_within_member_or_node() {
        assert_eq!(Probe::Utilization(4).next(), Probe::Axial(4));
        assert_eq!(Probe::Moment(4).next(), Probe::Utilization(4));
        assert_eq!(Probe::Speed(2).next(), Probe::Displacement(2));
    }
}
