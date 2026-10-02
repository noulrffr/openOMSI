//! Shared settings for the telemetry FFB model and launcher.
//! Defaults reproduce OMSI2DirectInputFFB.ini from the supplied plugin archive.
use serde_json::{json, Value};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_tuning_round_trips_and_legacy_globals_survive_other_settings_saves() {
        let mut ui = crate::settings_from_text(None);
        for p in PARAMETERS {
            ui[p.key] = json!(p.max);
        }
        for c in CHOICES {
            ui[c.key] = json!(c.options.last().unwrap().0);
        }
        for (key, _, _) in SWITCHES {
            ui[*key] = json!(false);
        }
        let expected = Settings::from_json(&ui);
        let text = expected.to_text();
        assert_eq!(Settings::from_text(&text), expected);
        assert_eq!(
            Settings::from_json(&crate::settings_from_text(Some(&text))),
            expected
        );
        assert_eq!(text.matches("ffb_overall_gain=").count(), 1);
        let old = format!("{text}future_option=keep\n");
        let saved = crate::settings_to_text(&ui, Some(&old));
        assert_eq!(Settings::from_text(&saved), expected);
        assert!(saved.contains("future_option=keep"));
        assert!(!crate::settings_to_text(&ui, None).contains("ffb_"));
        assert_eq!(Settings::from_text(""), Settings::default());
    }

    #[test]
    fn bad_numbers_and_cross_field_ranges_are_normalized_without_losing_good_settings() {
        let text = "ffb_overall_gain=0.4\nffb_output_limit=NaN\nffb_damper_at_rest=inf\nffb_friction_model=wrong\nffb_force_direction=wrong\nffb_calculated_damping=off\nffb_align_speed_start_kmh=100\nffb_align_speed_full_kmh=2\nffb_front_slip_onset=0.5\nffb_front_slip_full=0.1\nffb_rolling_radius_minimum=1\nffb_rolling_radius_maximum=0.2\nffb_impact_rumble_limit=0\nffb_impact_rumble_minimum=1\nffb_rumble_min_hz=90\nffb_rumble_max_hz=2\n";
        let s = Settings::from_text(text);
        assert_eq!(s.overall_gain, 0.4);
        assert_eq!(s.output_limit, 1.0);
        assert_eq!(s.damper_at_rest, Settings::default().damper_at_rest);
        assert_eq!(s.friction_model, FrictionModel::Smooth);
        assert!(!s.calculated_damping);
        assert!(s.align_speed_full_kmh > s.align_speed_start_kmh);
        assert!(s.front_slip_full > s.front_slip_onset);
        assert!(s.rolling_radius_maximum > s.rolling_radius_minimum);
        assert_eq!(s.impact_rumble_minimum, 0.0);
        assert_eq!(s.rumble_max_hz, 90.0);
    }
}

pub const GROUPS: [&str; 8] = [
    "Output",
    "Centring",
    "Grip and load",
    "Damping",
    "Friction",
    "Road kicks",
    "Impacts",
    "Road vibration",
];
pub struct Parameter {
    pub key: &'static str,
    pub label: &'static str,
    pub group: usize,
    pub default: f32,
    pub min: f32,
    pub max: f32,
    pub step: f32,
}
macro_rules! parameters {
    ($( $field:ident: ($group:expr, $label:expr, $default:expr, $min:expr, $max:expr, $step:expr) ),* $(,)?) => {
        #[derive(Clone, Copy, Debug, PartialEq)]
        pub struct Settings {
            $(pub $field: f32,)*
            pub calculated_damping: bool, pub calculated_friction: bool,
            pub friction_model: FrictionModel,
            pub rumble_waveform: Waveform, pub impact_rumble_waveform: Waveform,
            pub force_direction: Direction, pub resistance_direction: Direction, pub road_kick_direction: Direction,
        }
        impl Default for Settings {
            fn default() -> Self {
                Self { $($field: $default,)*
                    calculated_damping: true, calculated_friction: true,
                    friction_model: FrictionModel::Smooth,
                    rumble_waveform: Waveform::Triangle, impact_rumble_waveform: Waveform::Sine,
                    // openOMSI already normalizes physical/steering axis signs.
                    force_direction: Direction::Normal, resistance_direction: Direction::Normal,
                    road_kick_direction: Direction::Reverse,
                }
            }
        }
        pub const PARAMETERS: &[Parameter] = &[
            $(Parameter { key: concat!("ffb_", stringify!($field)), label: $label, group: $group,
                default: $default, min: $min, max: $max, step: $step },)*
        ];
        impl Settings {
            fn set_number(&mut self, key: &str, value: &str) {
                match key {
                    $(concat!("ffb_", stringify!($field)) => {
                        if let Ok(x) = value.parse::<f32>() { if x.is_finite() { self.$field = x.clamp($min, $max); } }
                    },)*
                    _ => {}
                }
            }
            pub fn validated(mut self) -> Self {
                $(self.$field = if self.$field.is_finite() { self.$field.clamp($min, $max) } else { $default };)*
                self.align_speed_full_kmh = self.align_speed_full_kmh.max(self.align_speed_start_kmh + 0.1);
                self.front_slip_full = self.front_slip_full.max(self.front_slip_onset + 0.01);
                self.rolling_radius_maximum = self.rolling_radius_maximum.max(self.rolling_radius_minimum + 0.05);
                self.impact_rumble_minimum = self.impact_rumble_minimum.min(self.impact_rumble_limit);
                self.rumble_max_hz = self.rumble_max_hz.max(self.rumble_min_hz);
                self
            }
            pub fn write_json(&self, target: &mut Value) {
                $(target[concat!("ffb_", stringify!($field))] = json!(self.$field);)*
                target["ffb_calculated_damping"] = json!(self.calculated_damping);
                target["ffb_calculated_friction"] = json!(self.calculated_friction);
                target["ffb_friction_model"] = json!(self.friction_model.key());
                target["ffb_rumble_waveform"] = json!(self.rumble_waveform.key());
                target["ffb_impact_rumble_waveform"] = json!(self.impact_rumble_waveform.key());
                target["ffb_force_direction"] = json!(self.force_direction.key());
                target["ffb_resistance_direction"] = json!(self.resistance_direction.key());
                target["ffb_road_kick_direction"] = json!(self.road_kick_direction.key());
            }
        }
    }
}
parameters! {
    overall_gain: (0, "Overall gain", 0.8, 0.0, 1.0, 0.01),
    output_limit: (0, "Output limit", 1.0, 0.0, 1.0, 0.01),
    model_force_limit: (0, "Base torque limit", 1.0, 0.0, 1.0, 0.01),
    force_slew_per_second: (0, "Torque slew (per second)", 2.0, 0.1, 20.0, 0.1),
    steering_deadband: (1, "Steering deadband (rad)", 0.004, 0.0, 0.25, 0.001),
    steering_response_exponent: (1, "Steering response exponent", 0.85, 0.25, 2.5, 0.01),
    align_at_rest: (1, "Centring at rest", 0.0, 0.0, 1.0, 0.01),
    align_at_speed: (1, "Centring at speed", 1.0, 0.0, 1.0, 0.01),
    align_speed_start_kmh: (1, "Centring starts (km/h)", 1.5, 0.0, 100.0, 0.1),
    align_speed_full_kmh: (1, "Full centring (km/h)", 35.0, 0.1, 150.0, 0.1),
    align_angle_saturation_rad: (1, "Angle saturation (rad)", 0.45, 0.05, 1.0, 0.005),
    align_tyre_onset_rad: (1, "Tyre force onset (rad)", 0.05, 0.005, 0.5, 0.005),
    align_response_time: (1, "Centring response (s)", 0.04, 0.0, 0.5, 0.001),
    lateral_accel_full: (2, "Lateral load peak (m/s2)", 3.5, 0.1, 20.0, 0.1),
    lateral_load_gain: (2, "Lateral load gain", 0.12, 0.0, 1.0, 0.01),
    longitudinal_accel_full: (2, "Longitudinal peak (m/s2)", 4.0, 0.5, 12.0, 0.1),
    longitudinal_load_gain: (2, "Longitudinal load gain", 0.16, 0.0, 0.5, 0.01),
    brake_load_gain: (2, "Brake pedal load gain", 0.02, 0.0, 1.0, 0.01),
    wet_grip_loss_per_level: (2, "Wet grip loss per level", 0.12, 0.0, 0.25, 0.01),
    front_slip_minimum_speed_kmh: (2, "Slip detection (km/h)", 8.0, 2.0, 30.0, 0.5),
    front_slip_onset: (2, "Slip onset ratio", 0.08, 0.01, 0.5, 0.01),
    front_slip_full: (2, "Full slip ratio", 0.35, 0.02, 1.5, 0.01),
    front_slip_align_loss: (2, "Centring loss on slip", 0.75, 0.0, 0.95, 0.01),
    rolling_radius_minimum: (2, "Minimum rolling radius (m)", 0.3, 0.15, 1.0, 0.01),
    rolling_radius_maximum: (2, "Maximum rolling radius (m)", 0.65, 0.2, 1.5, 0.01),
    rolling_radius_filter_time: (2, "Rolling radius filter (s)", 1.5, 0.1, 10.0, 0.1),
    damper_at_rest: (3, "Damping at rest", 0.053, 0.0, 1.0, 0.001),
    damper_at_speed: (3, "Damping at speed", 0.257, 0.0, 1.0, 0.001),
    calculated_damping_limit: (3, "Damping force limit", 0.5, 0.0, 1.0, 0.01),
    physical_wheel_rate_filter_time: (3, "Wheel rate filter (s)", 0.035, 0.0, 1.0, 0.001),
    physical_wheel_rate_limit: (3, "Wheel rate limit", 12.0, 0.1, 100.0, 0.1),
    damping_load_gain: (3, "Damping load gain", 0.25, 0.0, 2.0, 0.01),
    damping_high_rate_gain: (3, "Fast movement damping", 0.01, 0.0, 0.25, 0.001),
    friction_at_rest: (4, "Friction at rest", 0.14, 0.0, 1.0, 0.001),
    friction_at_speed: (4, "Friction at speed", 0.063, 0.0, 1.0, 0.001),
    calculated_friction_limit: (4, "Friction force limit", 0.3, 0.0, 1.0, 0.01),
    friction_velocity_scale: (4, "Friction crossing speed", 0.05, 0.001, 2.0, 0.001),
    friction_static_ratio: (4, "Static friction ratio", 1.3, 1.0, 3.0, 0.01),
    friction_stribeck_velocity: (4, "Stribeck speed", 0.1, 0.001, 2.0, 0.001),
    friction_viscous_gain: (4, "Viscous friction gain", 0.0, 0.0, 1.0, 0.001),
    friction_lugre_stiffness: (4, "LuGre stiffness", 12.0, 0.1, 100.0, 0.1),
    friction_lugre_damping: (4, "LuGre damping", 0.015, 0.0, 1.0, 0.001),
    road_kick_gain: (5, "Directional kick gain", 0.4, 0.0, 2.0, 0.01),
    road_kick_limit: (5, "Directional kick limit", 1.0, 0.0, 1.0, 0.01),
    road_kick_low_speed_weight: (5, "Low speed kick weight", 0.2, 0.0, 1.0, 0.01),
    road_kick_speed_weight: (5, "Road speed kick weight", 0.8, 0.0, 1.0, 0.01),
    suspension_rate_filter_time: (5, "Suspension rate filter (s)", 0.03, 0.0, 1.0, 0.001),
    road_kick_displacement_gain: (5, "Displacement kick gain", 1.5, 0.0, 20.0, 0.1),
    road_kick_position_filter_time: (5, "Camber decay (s)", 0.45, 0.05, 5.0, 0.01),
    vertical_filter_time: (6, "Vertical filter (s)", 0.18, 0.01, 2.0, 0.01),
    vertical_rumble_threshold: (6, "Vertical threshold (m/s2)", 0.45, 0.0, 20.0, 0.01),
    vertical_rumble_gain: (6, "Vertical impact gain", 1.0, 0.0, 1.0, 0.01),
    suspension_impact_threshold: (6, "Suspension threshold (m/s)", 0.03, 0.0, 5.0, 0.001),
    suspension_impact_gain: (6, "Suspension impact gain", 0.35, 0.0, 2.0, 0.01),
    impact_rumble_minimum: (6, "Minimum impact strength", 0.08, 0.0, 1.0, 0.01),
    impact_rumble_limit: (6, "Impact strength limit", 0.22, 0.0, 1.0, 0.01),
    impact_rumble_release_time: (6, "Impact release (s)", 0.12, 0.02, 1.0, 0.01),
    impact_rumble_frequency_hz: (6, "Impact frequency (Hz)", 12.0, 4.0, 40.0, 0.5),
    impact_low_speed_floor: (6, "Low speed impact weight", 0.6, 0.0, 1.0, 0.01),
    rear_impact_rejection: (6, "Rear impact rejection", 0.75, 0.0, 1.0, 0.01),
    unknown_impact_front_weight: (6, "Unlocalized impact weight", 0.7, 0.0, 1.0, 0.01),
    surface_rumble_gain: (7, "Surface vibration gain", 0.1, 0.0, 1.0, 0.01),
    rumble_limit: (7, "Surface vibration limit", 0.12, 0.0, 1.0, 0.01),
    surface_rumble_response_time: (7, "Surface response (s)", 0.06, 0.0, 1.0, 0.001),
    rumble_min_hz: (7, "Minimum frequency (Hz)", 16.0, 1.0, 100.0, 0.5),
    rumble_max_hz: (7, "Maximum frequency (Hz)", 30.0, 1.0, 150.0, 0.5),
}
macro_rules! choice {
    ($name:ident { $($variant:ident => $key:literal),+ }) => {
        #[derive(Clone, Copy, Debug, PartialEq)]
        pub enum $name { $($variant,)+ }
        impl $name {
            pub fn key(self) -> &'static str { match self { $(Self::$variant => $key,)+ } }
            fn parse(value: &str) -> Option<Self> { match value { $($key => Some(Self::$variant),)+ _ => None } }
        }
    }
}
choice!(FrictionModel { Smooth => "smooth", Stribeck => "stribeck", LuGre => "lugre" });
choice!(Waveform { Sine => "sine", Triangle => "triangle", Square => "square" });
choice!(Direction { Normal => "1", Reverse => "-1" });
impl Direction {
    pub fn sign(self) -> f32 {
        if self == Self::Normal {
            1.0
        } else {
            -1.0
        }
    }
}
pub struct Choice {
    pub key: &'static str,
    pub label: &'static str,
    pub group: usize,
    pub options: &'static [(&'static str, &'static str)],
}
pub const CHOICES: &[Choice] = &[
    Choice {
        key: "ffb_force_direction",
        label: "Base force direction",
        group: 0,
        options: &[("1", "Normal"), ("-1", "Reversed")],
    },
    Choice {
        key: "ffb_resistance_direction",
        label: "Resistance direction",
        group: 0,
        options: &[("1", "Normal"), ("-1", "Reversed")],
    },
    Choice {
        key: "ffb_friction_model",
        label: "Friction model",
        group: 4,
        options: &[
            ("smooth", "Smooth"),
            ("stribeck", "Stribeck"),
            ("lugre", "LuGre"),
        ],
    },
    Choice {
        key: "ffb_road_kick_direction",
        label: "Road kick direction",
        group: 5,
        options: &[("-1", "Plugin preset"), ("1", "Reversed")],
    },
    Choice {
        key: "ffb_impact_rumble_waveform",
        label: "Impact waveform",
        group: 6,
        options: &[
            ("sine", "Sine"),
            ("triangle", "Triangle"),
            ("square", "Square"),
        ],
    },
    Choice {
        key: "ffb_rumble_waveform",
        label: "Surface waveform",
        group: 7,
        options: &[
            ("triangle", "Triangle"),
            ("sine", "Sine"),
            ("square", "Square"),
        ],
    },
];
pub const SWITCHES: &[(&str, &str, usize)] = &[
    ("ffb_calculated_damping", "Software damping", 3),
    ("ffb_calculated_friction", "Software friction", 4),
];
impl Settings {
    fn set(&mut self, key: &str, value: &str) {
        let value = value.trim().to_ascii_lowercase();
        let boolean = || match value.as_str() {
            "1" | "true" | "on" | "yes" => Some(true),
            "0" | "false" | "off" | "no" => Some(false),
            _ => None,
        };
        match key {
            "ffb_calculated_damping" => {
                if let Some(v) = boolean() {
                    self.calculated_damping = v;
                }
            }
            "ffb_calculated_friction" => {
                if let Some(v) = boolean() {
                    self.calculated_friction = v;
                }
            }
            "ffb_friction_model" => {
                if let Some(v) = FrictionModel::parse(&value) {
                    self.friction_model = v;
                }
            }
            "ffb_rumble_waveform" => {
                if let Some(v) = Waveform::parse(&value) {
                    self.rumble_waveform = v;
                }
            }
            "ffb_impact_rumble_waveform" => {
                if let Some(v) = Waveform::parse(&value) {
                    self.impact_rumble_waveform = v;
                }
            }
            "ffb_force_direction" => {
                if let Some(v) = Direction::parse(&value) {
                    self.force_direction = v;
                }
            }
            "ffb_resistance_direction" => {
                if let Some(v) = Direction::parse(&value) {
                    self.resistance_direction = v;
                }
            }
            "ffb_road_kick_direction" => {
                if let Some(v) = Direction::parse(&value) {
                    self.road_kick_direction = v;
                }
            }
            _ => self.set_number(key, &value),
        }
    }
    pub fn from_text(text: &str) -> Self {
        let mut settings = Self::default();
        for line in text.lines() {
            let line = line.trim();
            if line.starts_with(['#', ';']) {
                continue;
            }
            if let Some((key, value)) = line.split_once('=') {
                settings.set(&key.trim().to_ascii_lowercase(), value);
            }
        }
        settings.validated()
    }
    pub fn from_json(value: &Value) -> Self {
        let mut settings = Self::default();
        if let Some(object) = value.as_object() {
            for (key, value) in object {
                settings.set(
                    key,
                    &value
                        .as_str()
                        .map(str::to_owned)
                        .unwrap_or_else(|| value.to_string()),
                );
            }
        }
        settings.validated()
    }
    pub fn to_text(&self) -> String {
        let mut value = json!({});
        self.validated().write_json(&mut value);
        let mut text = String::new();
        for (key, value) in value.as_object().unwrap() {
            let value = value
                .as_str()
                .map(str::to_owned)
                .or_else(|| value.as_f64().map(|v| (v as f32).to_string()))
                .unwrap_or_else(|| value.to_string());
            text.push_str(&format!("{key}={}\n", value));
        }
        text
    }
}
