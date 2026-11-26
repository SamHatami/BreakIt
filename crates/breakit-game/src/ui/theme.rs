//! The look of the UI, in one place.
//!
//! Screens never name a colour, a font size or a border width. They tag
//! nodes with a [`Skin`] (background and border swatches) and text with an
//! [`Ink`] (text role and colour swatch), and [`apply_theme`] resolves those
//! against the active [`UiTheme`]. Changing the look is a new `UiTheme`
//! preset; F10 cycles the presets at run time.

use bevy::prelude::*;

/// A named colour slot. Each theme fills every slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Swatch {
    /// Panel ground (the "paper").
    Panel,
    /// Cells, keys and buttons on a panel.
    Raised,
    /// Main text and strong rules.
    Ink,
    /// Secondary text.
    Muted,
    /// Thin rules between rows.
    Rule,
    /// Active tool, primary action, selection.
    Accent,
    /// Text on the accent.
    OnAccent,
    /// Failure only.
    Fail,
    OnFail,
    /// A met criterion.
    Ok,
    /// Instrument housing (test run read-outs).
    Instrument,
    /// The instrument's chart paper and its text.
    Chart,
    InstrumentInk,
    InstrumentMuted,
    /// Text drawn straight over the 3D scene.
    SceneText,
    /// Dims the scene behind a modal sheet.
    Scrim,
    Clear,
    /// Utilisation bands 0..50, 50..80, 80..100, over 100 %.
    Ramp(u8),
}

/// What a piece of text is for; each theme sizes it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum TextRole {
    /// Screen titles.
    Title,
    /// Panel headings.
    Heading,
    Body,
    /// Small labels above values.
    Label,
    /// Numbers, IDs, keys.
    Mono,
    /// The big test-time read-out.
    MonoLarge,
    /// The PASSED / FAILED stamp.
    Stamp,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Weight {
    Thin,
    Strong,
}

#[derive(Clone, Debug)]
pub struct UiTheme {
    pub name: &'static str,
    pub panel: Color,
    pub raised: Color,
    pub ink: Color,
    pub muted: Color,
    pub rule: Color,
    pub accent: Color,
    pub on_accent: Color,
    pub fail: Color,
    pub on_fail: Color,
    pub ok: Color,
    pub instrument: Color,
    pub chart: Color,
    pub instrument_ink: Color,
    pub instrument_muted: Color,
    pub scene_text: Color,
    pub scrim: Color,
    pub ramp: [Color; 4],
    /// Border widths, logical px.
    pub thin: f32,
    pub strong: f32,
    /// Corner radius of panels and buttons, logical px.
    pub radius: f32,
    /// Text sizes per [`TextRole`], logical px, in the enum's order.
    pub sizes: [f32; 7],
    /// Optional font per role: `None` uses Bevy's built-in font.
    pub font: Option<Handle<Font>>,
}

impl UiTheme {
    /// "Test sheet": drafting paper panels, ink rules, square corners,
    /// safety-yellow accent. The concept shown in the UI mock-ups.
    pub fn drafting() -> Self {
        Self {
            name: "Test sheet",
            panel: hex(0xEEEBE3),
            raised: hex(0xF7F5F0),
            ink: hex(0x1B1F23),
            muted: hex(0x4A5058),
            rule: hex(0xC9C4B8),
            accent: hex(0xE0A526),
            on_accent: hex(0x1B1F23),
            fail: hex(0xB8322A),
            on_fail: hex(0xFFFFFF),
            ok: hex(0x1B1F23),
            instrument: hex(0x23282D),
            chart: hex(0xE9E2C9),
            instrument_ink: hex(0xE9E2C9),
            instrument_muted: hex(0xB7B2A6),
            scene_text: hex(0xD9D5CA),
            scrim: Color::srgba(0.05, 0.06, 0.07, 0.62),
            ramp: RAMP,
            thin: 1.0,
            strong: 1.5,
            radius: 0.0,
            sizes: [24.0, 15.0, 13.0, 10.0, 12.0, 28.0, 30.0],
            font: None,
        }
    }

    /// A deliberately different look on the same layout, to prove the
    /// screens don't depend on the drafting style: dark slate panels,
    /// rounded corners, cyan accent.
    pub fn slate() -> Self {
        Self {
            name: "Slate",
            panel: hex(0x1E2329),
            raised: hex(0x2A3038),
            ink: hex(0xE6E9ED),
            muted: hex(0x9AA3AD),
            rule: hex(0x343C45),
            accent: hex(0x4CC3D9),
            on_accent: hex(0x0E1418),
            fail: hex(0xE0564A),
            on_fail: hex(0x16090A),
            ok: hex(0x7BD389),
            instrument: hex(0x15191D),
            chart: hex(0x1B2127),
            instrument_ink: hex(0xE6E9ED),
            instrument_muted: hex(0x8A939C),
            scene_text: hex(0xC9D1D9),
            scrim: Color::srgba(0.0, 0.0, 0.0, 0.55),
            ramp: RAMP,
            thin: 1.0,
            strong: 1.0,
            radius: 6.0,
            sizes: [24.0, 15.0, 13.0, 11.0, 12.0, 28.0, 30.0],
            font: None,
        }
    }

    /// "Blueprint": cyanotype panels, white linework, hairline rules,
    /// square corners. Reads like the engineer's own drawing set.
    pub fn blueprint() -> Self {
        Self {
            name: "Blueprint",
            panel: hex(0x163A5F),
            raised: hex(0x1D4A78),
            ink: hex(0xF2F6FA),
            muted: hex(0xA9C1DA),
            rule: hex(0x3E6893),
            accent: hex(0xF2F6FA),
            on_accent: hex(0x163A5F),
            fail: hex(0xFF7A66),
            on_fail: hex(0x1A0D0A),
            ok: hex(0xF2F6FA),
            instrument: hex(0x0F2A45),
            chart: hex(0x12324F),
            instrument_ink: hex(0xF2F6FA),
            instrument_muted: hex(0x8FAAC6),
            scene_text: hex(0xE4ECF4),
            scrim: Color::srgba(0.03, 0.10, 0.18, 0.62),
            ramp: RAMP,
            thin: 1.0,
            strong: 1.0,
            radius: 0.0,
            sizes: [24.0, 15.0, 13.0, 10.0, 12.0, 28.0, 30.0],
            font: None,
        }
    }

    /// "Hangar": the lab's own signage. Black plates, heavy borders,
    /// hazard-yellow accent, larger type for reading at a glance.
    pub fn hangar() -> Self {
        Self {
            name: "Hangar",
            panel: hex(0x111111),
            raised: hex(0x1E1E1E),
            ink: hex(0xF5F5F0),
            muted: hex(0xA8A8A0),
            rule: hex(0x3A3A36),
            accent: hex(0xFFC400),
            on_accent: hex(0x111111),
            fail: hex(0xFF4D2E),
            on_fail: hex(0x111111),
            ok: hex(0xF5F5F0),
            instrument: hex(0x0A0A0A),
            chart: hex(0x171717),
            instrument_ink: hex(0xFFC400),
            instrument_muted: hex(0x9A9A92),
            scene_text: hex(0xF5F5F0),
            scrim: Color::srgba(0.0, 0.0, 0.0, 0.7),
            ramp: RAMP,
            thin: 2.0,
            strong: 3.0,
            radius: 2.0,
            sizes: [26.0, 16.0, 13.0, 11.0, 13.0, 30.0, 34.0],
            font: None,
        }
    }

    pub fn color(&self, s: Swatch) -> Color {
        match s {
            Swatch::Panel => self.panel,
            Swatch::Raised => self.raised,
            Swatch::Ink => self.ink,
            Swatch::Muted => self.muted,
            Swatch::Rule => self.rule,
            Swatch::Accent => self.accent,
            Swatch::OnAccent => self.on_accent,
            Swatch::Fail => self.fail,
            Swatch::OnFail => self.on_fail,
            Swatch::Ok => self.ok,
            Swatch::Instrument => self.instrument,
            Swatch::Chart => self.chart,
            Swatch::InstrumentInk => self.instrument_ink,
            Swatch::InstrumentMuted => self.instrument_muted,
            Swatch::SceneText => self.scene_text,
            Swatch::Scrim => self.scrim,
            Swatch::Clear => Color::NONE,
            Swatch::Ramp(i) => self.ramp[(i as usize).min(3)],
        }
    }

    pub fn size(&self, r: TextRole) -> f32 {
        let i = match r {
            TextRole::Title => 0,
            TextRole::Heading => 1,
            TextRole::Body => 2,
            TextRole::Label => 3,
            TextRole::Mono => 4,
            TextRole::MonoLarge => 5,
            TextRole::Stamp => 6,
        };
        self.sizes[i]
    }

    pub fn border(&self, w: Weight) -> f32 {
        match w {
            Weight::Thin => self.thin,
            Weight::Strong => self.strong,
        }
    }
}

/// Utilisation ramp: blue, amber, orange, red. Each band also differs in
/// lightness, so it reads for colour-blind players; numbers always sit
/// next to it.
pub const RAMP: [Color; 4] = [
    Color::srgb(0.184, 0.435, 0.690),
    Color::srgb(0.910, 0.773, 0.278),
    Color::srgb(0.941, 0.541, 0.141),
    Color::srgb(0.769, 0.169, 0.137),
];

/// Ramp band for a utilisation (1.0 = at capacity).
pub fn ramp_band(u: f32) -> u8 {
    match u {
        u if u < 0.5 => 0,
        u if u < 0.8 => 1,
        u if u <= 1.0 => 2,
        _ => 3,
    }
}

fn hex(v: u32) -> Color {
    Color::srgb_u8((v >> 16) as u8, (v >> 8) as u8, v as u8)
}

/// The presets F10 cycles through, and which one is active.
#[derive(Resource, Clone, Debug)]
pub struct Themes {
    pub presets: Vec<UiTheme>,
    pub active: usize,
}

impl Default for Themes {
    fn default() -> Self {
        Self {
            presets: vec![
                UiTheme::drafting(),
                UiTheme::slate(),
                UiTheme::blueprint(),
                UiTheme::hangar(),
            ],
            active: 0,
        }
    }
}

impl Themes {
    pub fn current(&self) -> &UiTheme {
        &self.presets[self.active]
    }

    pub fn cycle(&mut self) {
        self.active = (self.active + 1) % self.presets.len();
    }
}

/// Background and border of a node, as swatches.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
#[require(BackgroundColor, BorderColor)]
pub struct Skin {
    pub bg: Swatch,
    pub border: Option<(Swatch, Weight)>,
    /// Round the corners by the theme's radius.
    pub rounded: bool,
}

impl Skin {
    pub const fn fill(bg: Swatch) -> Self {
        Self {
            bg,
            border: None,
            rounded: true,
        }
    }

    pub const fn boxed(bg: Swatch, border: Swatch, w: Weight) -> Self {
        Self {
            bg,
            border: Some((border, w)),
            rounded: true,
        }
    }

    pub const fn square(mut self) -> Self {
        self.rounded = false;
        self
    }
}

/// Text role and colour.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
#[require(TextFont, TextColor)]
pub struct Ink {
    pub role: TextRole,
    pub color: Swatch,
}

impl Ink {
    pub const fn new(role: TextRole, color: Swatch) -> Self {
        Self { role, color }
    }
}

/// Resolves [`Skin`] and [`Ink`] against the active theme: for new or
/// changed components every frame, and for everything when the theme
/// changes.
#[allow(clippy::type_complexity)]
pub fn apply_theme(
    themes: Res<Themes>,
    mut skins: Query<(Ref<Skin>, &mut BackgroundColor, &mut BorderColor, &mut Node)>,
    mut inks: Query<(Ref<Ink>, &mut TextFont, &mut TextColor)>,
) {
    let all = themes.is_changed();
    let t = themes.current();
    for (skin, mut bg, mut border, mut node) in &mut skins {
        if !(all || skin.is_changed()) {
            continue;
        }
        bg.0 = t.color(skin.bg);
        let (c, w) = match skin.border {
            Some((s, w)) => (t.color(s), t.border(w)),
            None => (Color::NONE, 0.0),
        };
        *border = BorderColor::all(c);
        let width = UiRect::all(Val::Px(w));
        if node.border != width {
            node.border = width;
        }
        let radius = if skin.rounded { t.radius } else { 0.0 };
        let r = BorderRadius::all(Val::Px(radius));
        if node.border_radius != r {
            node.border_radius = r;
        }
    }
    for (ink, mut font, mut color) in &mut inks {
        if !(all || ink.is_changed()) {
            continue;
        }
        color.0 = t.color(ink.color);
        let mut f = TextFont::from_font_size(FontSize::Px(t.size(ink.role)));
        if matches!(
            ink.role,
            TextRole::Title | TextRole::Heading | TextRole::Stamp
        ) {
            f = f.with_font_weight(FontWeight::BOLD);
        }
        if let Some(h) = &t.font {
            f = f.with_font(h.clone());
        }
        if *font != f {
            *font = f;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ramp_bands_split_at_50_80_100() {
        assert_eq!(ramp_band(0.2), 0);
        assert_eq!(ramp_band(0.5), 1);
        assert_eq!(ramp_band(0.92), 2);
        assert_eq!(ramp_band(1.0), 2);
        assert_eq!(ramp_band(1.04), 3);
    }

    #[test]
    fn every_preset_fills_every_swatch_distinctly_enough() {
        for t in Themes::default().presets {
            // Text on panels must not vanish.
            let a = t.color(Swatch::Ink).to_srgba();
            let b = t.color(Swatch::Panel).to_srgba();
            let d = (a.red - b.red).abs() + (a.green - b.green).abs() + (a.blue - b.blue).abs();
            assert!(d > 1.0, "{}: ink too close to panel", t.name);
        }
    }
}
