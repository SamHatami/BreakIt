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
    pub ok: Color,
    pub instrument: Color,
    pub chart: Color,
    pub instrument_ink: Color,
    pub instrument_muted: Color,
    pub scene_text: Color,
    pub scrim: Color,
    pub ramp: [Color; 4],
    /// Border width, logical px.
    pub thin: f32,
    /// Corner radius of panels and buttons, logical px.
    pub radius: f32,
    /// Text sizes per [`TextRole`], logical px, in the enum's order.
    pub sizes: [f32; 7],
    /// Font weight per [`TextRole`], 100..900, in the enum's order.
    pub weights: [u16; 7],
    /// Colour of the soft gradients along the top and bottom edges that
    /// keep text over the scene readable; `None` for no gradients.
    pub edge: Option<Color>,
    /// Text shadow colour, for text that sits on the scene.
    pub shadow: Option<Color>,
    /// Laid over the blurred scene behind every panel (`frost.rs`);
    /// alpha is how much it darkens.
    pub backdrop: Color,
    /// The hairline along a panel's edge.
    pub backdrop_edge: Color,
    /// The UI font (a variable-weight face). `None` uses Bevy's built-in
    /// font; [`Themes`] fills it with the bundled Inter Tight.
    pub font: Option<Handle<Font>>,
}

impl UiTheme {
    /// "Hairline": light type, a single warm accent for the active tool,
    /// the primary action and the playhead. Each group of read-outs sits on
    /// a frosted backdrop that darkens and blurs the scene behind it, so it
    /// reads over the bright hall.
    pub fn hairline() -> Self {
        let paper = Color::srgb_u8(0xF4, 0xF2, 0xEE);
        Self {
            name: "Hairline",
            panel: Color::NONE,
            raised: paper.with_alpha(0.08),
            ink: paper,
            muted: paper.with_alpha(0.70),
            rule: paper.with_alpha(0.22),
            accent: hex(0xFF8A3D),
            on_accent: hex(0x17120E),
            fail: hex(0xFF7A66),
            ok: hex(0x7BC67E),
            instrument: Color::NONE,
            chart: Color::NONE,
            instrument_ink: paper,
            instrument_muted: paper.with_alpha(0.68),
            scene_text: paper,
            scrim: Color::srgba(0.03, 0.035, 0.045, 0.55),
            ramp: RAMP,
            thin: 1.0,
            radius: 10.0,
            sizes: [26.0, 17.0, 15.0, 12.0, 14.0, 56.0, 30.0],
            weights: [400, 500, 400, 600, 400, 300, 500],
            edge: Some(Color::srgba(0.03, 0.035, 0.045, 0.35)),
            shadow: Some(Color::srgba(0.0, 0.0, 0.0, 0.45)),
            backdrop: Color::srgba(0.045, 0.05, 0.06, 0.62),
            backdrop_edge: paper.with_alpha(0.14),
            font: None,
        }
    }

    /// The same layout on opaque dark cards, to keep the screens honest
    /// about not depending on one look.
    pub fn slate() -> Self {
        Self {
            name: "Slate",
            panel: hex(0x1E2329).with_alpha(0.92),
            raised: hex(0x2A3038),
            ink: hex(0xE6E9ED),
            muted: hex(0x9AA3AD),
            rule: hex(0x343C45),
            accent: hex(0x4CC3D9),
            on_accent: hex(0x0E1418),
            fail: hex(0xE0564A),
            ok: hex(0x7BD389),
            instrument: hex(0x15191D).with_alpha(0.92),
            chart: hex(0x1B2127),
            instrument_ink: hex(0xE6E9ED),
            instrument_muted: hex(0x8A939C),
            scene_text: hex(0xC9D1D9),
            scrim: Color::srgba(0.0, 0.0, 0.0, 0.65),
            ramp: RAMP,
            thin: 1.0,
            radius: 6.0,
            sizes: [24.0, 15.0, 13.0, 11.0, 12.0, 44.0, 30.0],
            weights: [600, 600, 400, 500, 400, 400, 700],
            edge: None,
            shadow: None,
            backdrop: hex(0x1E2329).with_alpha(0.86),
            backdrop_edge: hex(0x343C45).with_alpha(1.0),
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
        self.sizes[role_index(r)]
    }

    pub fn weight(&self, r: TextRole) -> u16 {
        self.weights[role_index(r)]
    }

    pub fn border(&self, w: Weight) -> f32 {
        match w {
            Weight::Thin => self.thin,
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

fn role_index(r: TextRole) -> usize {
    match r {
        TextRole::Title => 0,
        TextRole::Heading => 1,
        TextRole::Body => 2,
        TextRole::Label => 3,
        TextRole::Mono => 4,
        TextRole::MonoLarge => 5,
        TextRole::Stamp => 6,
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
            presets: vec![UiTheme::hairline(), UiTheme::slate()],
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

/// Inter Tight (SIL Open Font License, see `fonts/OFL.txt`), a variable
/// face, so one file covers every weight the themes ask for.
const UI_FONT: &[u8] = include_bytes!("../../fonts/InterTight.ttf");

/// Loads the bundled UI font into every preset.
pub fn load_fonts(mut fonts: ResMut<Assets<Font>>, mut themes: ResMut<Themes>) {
    let h = fonts.add(Font::from_bytes(UI_FONT.to_vec()));
    for t in &mut themes.presets {
        t.font = Some(h.clone());
    }
}

/// How a node's corners are rounded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Corners {
    Square,
    /// The theme's radius.
    Theme,
    /// Fully round: pills and dots.
    Pill,
}

/// Background and border of a node, as swatches.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
#[require(BackgroundColor, BorderColor)]
pub struct Skin {
    pub bg: Swatch,
    pub border: Option<(Swatch, Weight)>,
    pub corners: Corners,
}

impl Skin {
    pub const fn fill(bg: Swatch) -> Self {
        Self {
            bg,
            border: None,
            corners: Corners::Theme,
        }
    }

    pub const fn boxed(bg: Swatch, border: Swatch, w: Weight) -> Self {
        Self {
            bg,
            border: Some((border, w)),
            corners: Corners::Theme,
        }
    }

    pub const fn square(mut self) -> Self {
        self.corners = Corners::Square;
        self
    }

    pub const fn pill(mut self) -> Self {
        self.corners = Corners::Pill;
        self
    }
}

/// Text role and colour.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
#[require(TextFont, TextColor, TextShadow)]
pub struct Ink {
    pub role: TextRole,
    pub color: Swatch,
}

impl Ink {
    pub const fn new(role: TextRole, color: Swatch) -> Self {
        Self { role, color }
    }
}

/// A soft gradient along the top or bottom edge of the screen, behind the
/// UI, so text over the scene stays readable. Themes without an edge
/// colour leave it clear.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
#[require(BackgroundGradient)]
pub enum EdgeScrim {
    Top,
    Bottom,
}

impl EdgeScrim {
    /// The scrim node: full width, `height` px, pinned to its edge.
    pub fn node(self, height: f32) -> Node {
        let (top, bottom) = match self {
            EdgeScrim::Top => (Val::Px(0.0), Val::Auto),
            EdgeScrim::Bottom => (Val::Auto, Val::Px(0.0)),
        };
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(0.0),
            right: Val::Px(0.0),
            top,
            bottom,
            height: Val::Px(height),
            ..default()
        }
    }
}

/// Resolves [`Skin`], [`Ink`] and [`EdgeScrim`] against the active theme:
/// for new or changed components every frame, and for everything when the
/// theme changes.
#[allow(clippy::type_complexity)]
pub fn apply_theme(
    themes: Res<Themes>,
    mut skins: Query<(Ref<Skin>, &mut BackgroundColor, &mut BorderColor, &mut Node)>,
    mut inks: Query<(Ref<Ink>, &mut TextFont, &mut TextColor, &mut TextShadow)>,
    mut edges: Query<(Ref<EdgeScrim>, &mut BackgroundGradient)>,
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
        let r = match skin.corners {
            Corners::Square => BorderRadius::ZERO,
            Corners::Theme => BorderRadius::all(Val::Px(t.radius)),
            Corners::Pill => BorderRadius::MAX,
        };
        if node.border_radius != r {
            node.border_radius = r;
        }
    }
    for (ink, mut font, mut color, mut shadow) in &mut inks {
        if !(all || ink.is_changed()) {
            continue;
        }
        color.0 = t.color(ink.color);
        let mut f = TextFont::from_font_size(FontSize::Px(t.size(ink.role)))
            .with_font_weight(FontWeight(t.weight(ink.role)));
        if let Some(h) = &t.font {
            f = f.with_font(h.clone());
        }
        if *font != f {
            *font = f;
        }
        let s = TextShadow {
            offset: Vec2::new(0.0, 1.0),
            color: t.shadow.unwrap_or(Color::NONE),
        };
        if *shadow != s {
            *shadow = s;
        }
    }
    for (edge, mut grad) in &mut edges {
        if !(all || edge.is_changed()) {
            continue;
        }
        let c = t.edge.unwrap_or(Color::NONE);
        let stops = vec![
            ColorStop::new(c, Val::Percent(0.0)),
            ColorStop::new(c.with_alpha(0.0), Val::Percent(100.0)),
        ];
        let g = match *edge {
            EdgeScrim::Top => LinearGradient::to_bottom(stops),
            EdgeScrim::Bottom => LinearGradient::to_top(stops),
        };
        *grad = BackgroundGradient::from(g);
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
