//! Telemetry steering model adapted from the user's OMSI2 DirectInput FFB
//! Telemetry Physics Update (`force_model.hpp` and its supplied INI preset).
//! All forces use openOMSI's convention: positive pushes the wheel right.
//! Driver polarity belongs in the controller backend, not in this model.

use omsi_launcher_lib::ffb::{FrictionModel, Settings, Waveform};
use std::f32::consts::TAU;

/// Road-wheel angles are radians, suspension travel is metres (negative when
/// compressed), accelerations are m/s² and wheel speeds are RPM, as in OMSI.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Telemetry {
    pub speed_kmh: f32,
    pub steering: [f32; 2],
    pub suspension: [f32; 2],
    pub rear_suspension: [f32; 2],
    pub acceleration: [f32; 3],
    pub street_condition: f32,
    pub brake: f32,
    pub surface: [f32; 2],
    pub wheel_rpm: [f32; 2],
    pub script_amplitude: f32,
    /// OMSI FF_Vib_Period, in hundredths of a second.
    pub script_period: f32,
}

#[derive(Default)]
pub(crate) struct ForceFeedback {
    settings: Settings,
    friction_state: f32,
    previous: Option<[f32; 3]>,
    suspension_rate: [f32; 3],
    suspension_difference: f32,
    vertical: f32,
    aligning: f32,
    rolling_radius: f32,
    surface_energy: f32,
    surface_magnitude: f32,
    impact_envelope: f32,
    impact_active: bool,
    previous_wheel: Option<f32>,
    wheel_rate: f32,
    base_force: f32,
    surface_wave: SurfaceTexture,
    impact_wave: Wave,
    script_wave: Wave,
}

#[derive(Debug, Default)]
struct Forces {
    aligning: f32,
    road_kick: f32,
    damper: f32,
    friction: f32,
    surface: f32,
    surface_hz: f32,
    impact: f32,
    new_impact: bool,
}

impl ForceFeedback {
    pub fn configure(&mut self, settings: Settings) {
        let settings = settings.validated();
        if settings != self.settings {
            self.settings = settings;
            self.reset();
        }
    }

    pub fn reset(&mut self) {
        *self = Self {
            settings: self.settings,
            ..Self::default()
        };
    }

    /// The physical position is read before deadzone and steering range scaling,
    /// but after axis reversal so it shares the road-wheel coordinate system.
    pub fn update(
        &mut self,
        input: &Telemetry,
        physical_position: f32,
        wheel_degrees: f32,
        dt: f32,
        steering_scale: f32,
        vibration_scale: f32,
    ) -> f32 {
        // Do not turn a pause, stale frame or broken axis into a derivative kick.
        if !dt.is_finite() || dt <= 0.0 || dt > 0.3 || !physical_position.is_finite() {
            self.reset();
            return 0.0;
        }
        // At e.g. 144 FPS the 10 ms backend throttle sends every second frame.
        let output_period = dt * (0.01 / dt).ceil().max(1.0);
        let dt = dt.clamp(0.001, 0.1);
        let s = self.settings;
        let f = self.telemetry(input, dt);
        let position = physical_position.clamp(-1.0, 1.0);
        let mut resistance = 0.0;
        if let Some(previous) = self.previous_wheel.replace(position) {
            let rotation = (finite(wheel_degrees, 900.0) / 900.0).clamp(0.25, 5.0);
            let rate = ((position - previous) / dt * rotation)
                .clamp(-s.physical_wheel_rate_limit, s.physical_wheel_rate_limit);
            self.wheel_rate =
                low_pass(self.wheel_rate, rate, s.physical_wheel_rate_filter_time, dt);
            let load = 1.0
                + s.damping_load_gain
                    * (f.aligning.abs() / s.align_at_speed.max(0.05)).clamp(0.0, 1.0);
            if s.calculated_damping {
                resistance += (-self.wheel_rate
                    * (f.damper * load + s.damping_high_rate_gain * self.wheel_rate.abs()))
                .clamp(-s.calculated_damping_limit, s.calculated_damping_limit);
            }
            if s.calculated_friction {
                resistance += self.friction(f.friction, dt);
            } else {
                self.friction_state = 0.0;
            }
        }
        let ks = finite(steering_scale, 0.0).clamp(0.0, 2.0);
        let ke = finite(vibration_scale, 0.0).clamp(0.0, 2.0);
        // Slew the base torque (preset: 2 full-scale units/s); keep detail
        // waves outside that limiter. A zero channel scale must mute immediately.
        let limit = s.model_force_limit.min(s.output_limit);
        let target =
            (f.aligning * ks + f.road_kick * ke).clamp(-limit, limit) * s.force_direction.sign();
        self.base_force += (target - self.base_force)
            .clamp(-s.force_slew_per_second * dt, s.force_slew_per_second * dt);
        if ks == 0.0 && ke == 0.0 {
            self.base_force = 0.0;
        }
        // openOMSI sends at most 100 Hz and may render more slowly. Keep all
        // texture bands below 40% of the effective output sample rate.
        let sample_hz = 1.0 / output_period;
        let surface = self.surface_wave.update(
            f.surface,
            f.surface_hz.min(sample_hz * 0.24),
            dt,
            s.rumble_waveform,
        );
        if f.new_impact {
            self.impact_wave = Wave::default();
        }
        let impact = self.impact_wave.update(
            f.impact,
            s.impact_rumble_frequency_hz.min(sample_hz * 0.4),
            dt,
            s.impact_rumble_waveform,
        );
        let script_hz = (100.0 / finite(input.script_period, 2.0).max(2.0)).min(sample_hz * 0.4);
        let script = self.script_wave.update(
            finite(input.script_amplitude, 0.0).clamp(0.0, 1.0) * 0.25,
            script_hz,
            dt,
            Waveform::Sine,
        );
        s.overall_gain
            * (self.base_force
                + resistance * ks * s.resistance_direction.sign()
                + (surface + impact + script) * ke)
                .clamp(-s.output_limit, s.output_limit)
    }

    fn friction(&mut self, dynamic: f32, dt: f32) -> f32 {
        let s = self.settings;
        let rate = self.wheel_rate;
        let static_level = (dynamic * s.friction_static_ratio).clamp(dynamic, 1.0);
        let speed_ratio = rate.abs() / s.friction_stribeck_velocity;
        let stribeck = dynamic + (static_level - dynamic) * (-speed_ratio * speed_ratio).exp();
        let force = match s.friction_model {
            FrictionModel::Smooth => {
                self.friction_state = 0.0;
                -(rate / s.friction_velocity_scale).tanh() * dynamic
            }
            FrictionModel::Stribeck => {
                self.friction_state = 0.0;
                -(rate / s.friction_velocity_scale).tanh() * stribeck
                    - rate * s.friction_viscous_gain
            }
            FrictionModel::LuGre => {
                if static_level <= 0.000001 {
                    self.friction_state = 0.0;
                    return 0.0;
                }
                let displacement = (stribeck / s.friction_lugre_stiffness).max(0.000001);
                let previous = self.friction_state;
                if rate.abs() > 0.0000001 {
                    let target = displacement.copysign(rate);
                    self.friction_state =
                        target + (previous - target) * (-rate.abs() * dt / displacement).exp();
                }
                let limit = static_level / s.friction_lugre_stiffness;
                self.friction_state = self.friction_state.clamp(-limit, limit);
                -(s.friction_lugre_stiffness * self.friction_state
                    + s.friction_lugre_damping * (self.friction_state - previous) / dt
                    + s.friction_viscous_gain * rate)
            }
        };
        force.clamp(-s.calculated_friction_limit, s.calculated_friction_limit)
    }

    fn telemetry(&mut self, input: &Telemetry, dt: f32) -> Forces {
        let s = self.settings;
        let speed = finite(input.speed_kmh, 0.0).abs();
        let speed_factor = smooth_step(s.align_speed_start_kmh, s.align_speed_full_kmh, speed);
        let steering = input.steering.map(|x| finite(x, 0.0).clamp(-1.0, 1.0));
        let steering = (steering[0] + steering[1]) * 0.5;
        let shaped = (((steering.abs() - s.steering_deadband).max(0.0)
            / (1.0 - s.steering_deadband))
            .powf(s.steering_response_exponent))
        .copysign(steering);
        let front = input.suspension.map(|x| finite(x, 0.0).clamp(-1.0, 1.0));
        let rear = input
            .rear_suspension
            .map(|x| finite(x, 0.0).clamp(-1.0, 1.0));
        let sample = [
            front[0] - front[1],
            0.5 * (front[0] + front[1]),
            0.5 * (rear[0] + rear[1]),
        ];
        let acceleration = input
            .acceleration
            .map(|x| finite(x, 0.0).clamp(-100.0, 100.0));
        if let Some(previous) = self.previous.replace(sample) {
            for i in 0..3 {
                self.suspension_rate[i] = low_pass(
                    self.suspension_rate[i],
                    (sample[i] - previous[i]) / dt,
                    s.suspension_rate_filter_time,
                    dt,
                );
            }
        } else {
            self.suspension_difference = sample[0];
            self.vertical = acceleration[2];
        }
        self.suspension_difference = low_pass(
            self.suspension_difference,
            sample[0],
            s.road_kick_position_filter_time,
            dt,
        );
        self.vertical = low_pass(self.vertical, acceleration[2], s.vertical_filter_time, dt);

        let direction = if input.speed_kmh < 0.0 { -1.0 } else { 1.0 };
        let load_ratio = if speed > 1.0 {
            (-direction * acceleration[1] / s.longitudinal_accel_full).clamp(-1.0, 1.0)
        } else {
            0.0
        };
        let brake = finite(input.brake, 0.0).clamp(0.0, 1.0);
        let load = (1.0
            + s.longitudinal_load_gain * load_ratio
            + s.brake_load_gain * brake * smooth_step(1.0, 8.0, speed))
        .clamp(0.75, 1.35);
        let grip = (1.0
            - s.wet_grip_loss_per_level * finite(input.street_condition, 0.0).clamp(0.0, 2.0))
        .clamp(0.45, 1.0);
        let rpm = input.wheel_rpm.map(|x| finite(x, 0.0).abs());
        let mean_rpm = 0.5 * (rpm[0] + rpm[1]);
        let mismatch = if mean_rpm > 1.0 {
            (rpm[0] - rpm[1]).abs() / mean_rpm
        } else {
            1.0
        };
        let omega = mean_rpm * TAU / 60.0;
        let speed_ms = speed / 3.6;
        if speed >= s.front_slip_minimum_speed_kmh
            && omega > 1.0
            && mismatch < 0.08
            && acceleration[1].abs() < 0.75
            && brake < 0.05
        {
            let radius = speed_ms / omega;
            if (s.rolling_radius_minimum..=s.rolling_radius_maximum).contains(&radius) {
                self.rolling_radius = if self.rolling_radius > 0.0 {
                    low_pass(
                        self.rolling_radius,
                        radius,
                        s.rolling_radius_filter_time,
                        dt,
                    )
                } else {
                    radius
                };
            }
        }
        let slip = if self.rolling_radius > 0.0 && speed >= s.front_slip_minimum_speed_kmh {
            let wheel_speed = rpm.map(|x| x * TAU / 60.0 * self.rolling_radius);
            (0.5 * ((wheel_speed[0] - speed_ms).abs() + (wheel_speed[1] - speed_ms).abs())
                / speed_ms.max(2.0))
            .clamp(0.0, 2.0)
        } else {
            0.0
        };
        let front_grip = (1.0
            - s.front_slip_align_loss * smooth_step(s.front_slip_onset, s.front_slip_full, slip))
        .clamp(0.1, 1.0);
        let align_strength = lerp(s.align_at_rest, s.align_at_speed, speed_factor);
        let caster = align_strength
            * s.align_angle_saturation_rad
            * (shaped.abs() / s.align_angle_saturation_rad).tanh();
        let lateral_ratio =
            (acceleration[0].abs() / (s.lateral_accel_full * grip).max(0.1)).clamp(0.0, 4.0);
        let tyre = align_strength
            * s.lateral_load_gain
            * (shaped.abs() / s.align_tyre_onset_rad).tanh()
            * lateral_ratio
            * (1.0 - lateral_ratio).exp();
        let target = if shaped == 0.0 {
            0.0
        } else {
            -((caster * (0.30 + 0.70 * front_grip) + tyre * front_grip) * load * grip)
                .copysign(shaped)
        };
        self.aligning = low_pass(self.aligning, target, s.align_response_time, dt);
        // Suspension signs match OMSI's Axle_Suspension convention.
        let road_kick = (-(self.suspension_rate[0] * s.road_kick_gain
            + (sample[0] - self.suspension_difference) * s.road_kick_displacement_gain)
            * (s.road_kick_low_speed_weight + s.road_kick_speed_weight * speed_factor)
            * s.road_kick_direction.sign())
        .clamp(-s.road_kick_limit, s.road_kick_limit);

        let vertical_high_pass = acceleration[2] - self.vertical;
        let front_evidence = self.suspension_rate[1].abs();
        let rear_evidence = self.suspension_rate[2].abs();
        let evidence = front_evidence + rear_evidence;
        let front_weight = if evidence > (s.suspension_impact_threshold * 0.5).max(0.002) {
            1.0 - s.rear_impact_rejection * (1.0 - front_evidence / evidence)
        } else {
            s.unknown_impact_front_weight
        };
        let vertical_impact = (vertical_high_pass.abs() - s.vertical_rumble_threshold).max(0.0)
            * s.vertical_rumble_gain
            * front_weight;
        let suspension_impact =
            (front_evidence - s.suspension_impact_threshold).max(0.0) * s.suspension_impact_gain;
        let mut impact_target = vertical_impact
            .hypot(suspension_impact)
            .clamp(0.0, s.impact_rumble_limit);
        if impact_target > 0.0 {
            impact_target = s.impact_rumble_minimum
                + (s.impact_rumble_limit - s.impact_rumble_minimum)
                    * (impact_target / s.impact_rumble_limit);
        }
        let new_impact = impact_target > 0.0 && !self.impact_active;
        self.impact_active = impact_target > 0.0;
        self.impact_envelope = if impact_target >= self.impact_envelope {
            impact_target
        } else {
            low_pass(
                self.impact_envelope,
                impact_target,
                s.impact_rumble_release_time,
                dt,
            )
        };
        if self.impact_envelope < 0.00001 {
            self.impact_envelope = 0.0;
        }

        let surface = input.surface.map(surface_properties);
        let roughness = 0.5 * (surface[0].0 + surface[1].0);
        let spatial_frequency = (surface[0].0 / surface[0].1 + surface[1].0 / surface[1].1)
            / (surface[0].0 + surface[1].0).max(0.001);
        let raw_frequency = speed_ms * spatial_frequency;
        let measured = (vertical_high_pass.abs() / s.vertical_rumble_threshold.max(0.50)
            * front_weight)
            .clamp(0.0, 1.0)
            .hypot(
                ((front_evidence + 0.5 * self.suspension_rate[0].abs())
                    / s.suspension_impact_threshold.max(0.030))
                .clamp(0.0, 1.0),
            );
        self.surface_energy = low_pass(
            self.surface_energy,
            measured * measured,
            (s.surface_rumble_response_time * 4.0).max(0.15),
            dt,
        );
        let response = 1.0 + 0.35 * self.surface_energy.sqrt().clamp(0.0, 1.0);
        let surface_target = (smooth_step(0.25, 8.0, speed)
            * smooth_step(s.rumble_min_hz * 0.35, s.rumble_min_hz, raw_frequency)
            * roughness
            * s.surface_rumble_gain
            * response)
            .clamp(0.0, s.rumble_limit);
        self.surface_magnitude = low_pass(
            self.surface_magnitude,
            surface_target,
            s.surface_rumble_response_time,
            dt,
        );
        Forces {
            aligning: self.aligning,
            road_kick,
            damper: lerp(s.damper_at_rest, s.damper_at_speed, speed_factor),
            friction: lerp(s.friction_at_rest, s.friction_at_speed, speed_factor),
            surface: self.surface_magnitude,
            surface_hz: raw_frequency.clamp(s.rumble_min_hz, s.rumble_max_hz),
            impact: self.impact_envelope * lerp(s.impact_low_speed_floor, 1.0, speed_factor),
            new_impact,
        }
    }
}

fn finite(value: f32, fallback: f32) -> f32 {
    if value.is_finite() {
        value
    } else {
        fallback
    }
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

fn low_pass(previous: f32, input: f32, time: f32, dt: f32) -> f32 {
    if time <= 0.0 {
        input
    } else {
        lerp(previous, input, 1.0 - (-dt / time).exp())
    }
}

fn smooth_step(start: f32, end: f32, value: f32) -> f32 {
    let x = ((value - start) / (end - start)).clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

fn surface_properties(surface: f32) -> (f32, f32) {
    // Plugin's material priors: roughness and dominant wavelength in metres.
    match finite(surface, 0.0).round() as i32 {
        0 => (0.02, 0.10),
        1 => (0.16, 0.45),
        2 => (1.00, 0.22),
        3 => (0.45, 0.30),
        4 => (0.35, 0.35),
        5 => (0.65, 0.12),
        6 => (0.15, 0.28),
        7 => (0.10, 0.38),
        8 => (0.20, 0.24),
        9 => (0.02, 0.12),
        10 => (0.10, 0.30),
        11 => (0.25, 0.34),
        12 => (0.30, 0.26),
        13 => (0.15, 0.16),
        14 => (0.25, 0.20),
        15 => (0.75, 0.14),
        16 => (0.50, 0.20),
        _ => (0.05, 0.20),
    }
}

#[derive(Default)]
struct Wave {
    phase: Option<f32>,
}

fn wave(phase: f32, waveform: Waveform) -> f32 {
    let sine = (phase * TAU).sin();
    match waveform {
        Waveform::Sine => sine,
        Waveform::Triangle => (2.0 / std::f32::consts::PI) * sine.asin(),
        Waveform::Square => {
            if sine >= 0.0 {
                1.0
            } else {
                -1.0
            }
        }
    }
}

impl Wave {
    fn update(&mut self, amplitude: f32, frequency: f32, dt: f32, waveform: Waveform) -> f32 {
        if amplitude <= 0.00001 {
            self.phase = None;
            return 0.0;
        }
        let phase = self.phase.map_or(0.25, |p| (p + frequency * dt).fract());
        self.phase = Some(phase);
        amplitude * wave(phase, waveform)
    }
}

#[derive(Default)]
struct SurfaceTexture {
    phases: Option<[f32; 4]>,
}

impl SurfaceTexture {
    fn update(&mut self, amplitude: f32, frequency: f32, dt: f32, waveform: Waveform) -> f32 {
        if amplitude <= 0.00001 {
            self.phases = None;
            return 0.0;
        }
        let ratios = [0.61, 0.93, 1.27, 1.67];
        let weights = [0.30, 0.31, 0.23, 0.16];
        let phases = self.phases.map_or([0.07, 0.29, 0.53, 0.81], |mut p| {
            for i in 0..4 {
                p[i] = (p[i] + frequency * ratios[i] * dt).fract();
            }
            p
        });
        self.phases = Some(phases);
        let carrier: f32 = (0..4).map(|i| weights[i] * wave(phases[i], waveform)).sum();
        amplitude * (carrier * 1.75).clamp(-1.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn configured_gain_limits_slew_and_direction_reach_the_wheel_output() {
        let output = |settings: Settings| {
            let mut model = ForceFeedback::default();
            model.configure(settings);
            (0..200)
                .map(|_| model.update(&corner(35.0), 0.3, 900.0, 0.02, 1.0, 0.0))
                .last()
                .unwrap()
        };
        let normal = output(Settings::default());
        assert!(normal < -0.1);
        assert_eq!(
            output(Settings {
                overall_gain: 0.4,
                ..Settings::default()
            }),
            normal * 0.5
        );
        assert_eq!(
            output(Settings {
                force_direction: omsi_launcher_lib::ffb::Direction::Reverse,
                ..Settings::default()
            }),
            -normal
        );
        assert!(
            (output(Settings {
                output_limit: 0.1,
                ..Settings::default()
            }) + 0.08)
                .abs()
                < 1e-6
        );
        assert!(
            (output(Settings {
                model_force_limit: 0.05,
                ..Settings::default()
            }) + 0.04)
                .abs()
                < 1e-6
        );
        assert_eq!(
            output(Settings {
                output_limit: 0.0,
                ..Settings::default()
            }),
            0.0
        );
        let mut model = ForceFeedback::default();
        model.configure(Settings {
            force_slew_per_second: 0.1,
            ..Settings::default()
        });
        assert!((model.update(&corner(35.0), 0.3, 900.0, 0.02, 1.0, 0.0) + 0.0016).abs() < 1e-6);
    }

    #[test]
    fn centring_and_resistance_can_be_tuned_independently() {
        let mut model = ForceFeedback::default();
        model.configure(Settings {
            align_at_rest: 0.7,
            align_response_time: 0.0,
            ..Settings::default()
        });
        assert!(model.telemetry(&corner(0.0), 0.02).aligning < -0.1);
        model.configure(Settings {
            align_at_rest: 0.0,
            align_at_speed: 0.0,
            ..Settings::default()
        });
        assert_eq!(model.telemetry(&corner(35.0), 0.02).aligning, 0.0);
        let resistance = |damping, friction, reverse| {
            let mut model = ForceFeedback::default();
            let mut settings = Settings {
                calculated_damping: damping,
                calculated_friction: friction,
                ..Settings::default()
            };
            if reverse {
                settings.resistance_direction = omsi_launcher_lib::ffb::Direction::Reverse;
            }
            model.configure(settings);
            model.update(&Telemetry::default(), 0.0, 900.0, 0.02, 1.0, 0.0);
            model.update(&Telemetry::default(), 0.001, 900.0, 0.02, 1.0, 0.0)
        };
        let damping = resistance(true, false, false);
        let friction = resistance(false, true, false);
        assert!(damping < 0.0 && friction < 0.0);
        assert_eq!(resistance(false, false, false), 0.0);
        assert!((resistance(true, true, false) - damping - friction).abs() < 1e-6);
        assert_eq!(resistance(true, true, true), -resistance(true, true, false));
    }

    #[test]
    fn stribeck_breakaway_and_lugre_memory_are_bounded_and_reset() {
        let mut model = ForceFeedback::default();
        model.wheel_rate = 0.03;
        let smooth = model.friction(0.14, 0.01);
        model.configure(Settings {
            friction_model: FrictionModel::Stribeck,
            ..Settings::default()
        });
        model.wheel_rate = 0.03;
        assert!(model.friction(0.14, 0.01) < smooth);
        let settings = Settings {
            friction_model: FrictionModel::LuGre,
            calculated_friction_limit: 0.1,
            ..Settings::default()
        };
        model.configure(settings);
        model.wheel_rate = 0.2;
        for _ in 0..200 {
            assert!(model.friction(0.14, 0.01).abs() <= 0.1);
        }
        model.wheel_rate = 0.0;
        assert!(
            model.friction(0.14, 0.01) < -0.09,
            "LuGre retains elastic friction at rest"
        );
        model.reset();
        assert_eq!(model.settings, settings);
        assert_eq!(model.friction_state, 0.0);
        assert_eq!(
            model.update(&Telemetry::default(), -0.8, 900.0, 0.02, 1.0, 0.0),
            0.0
        );
        model.configure(settings);
        assert!(
            model.previous_wheel.is_some(),
            "unchanged settings must not reset every frame"
        );
        model.configure(Settings {
            friction_model: FrictionModel::Smooth,
            ..settings
        });
        assert!(model.previous_wheel.is_none());
    }

    #[test]
    fn waveform_settings_change_surface_and_impact_output() {
        for impacts in [false, true] {
            let trace = |waveform| {
                let mut model = ForceFeedback::default();
                model.configure(Settings {
                    rumble_waveform: waveform,
                    impact_rumble_waveform: waveform,
                    surface_rumble_gain: if impacts { 0.0 } else { 0.1 },
                    ..Settings::default()
                });
                (0..80)
                    .map(|i| {
                        let mut input = Telemetry {
                            surface: [2.0; 2],
                            ..corner(35.0)
                        };
                        if impacts && i > 0 {
                            input.acceleration[2] = 3.0;
                        }
                        model.update(&input, 0.0, 900.0, 0.01, 0.0, 1.0)
                    })
                    .collect::<Vec<_>>()
            };
            let sine = trace(Waveform::Sine);
            let triangle = trace(Waveform::Triangle);
            let square = trace(Waveform::Square);
            assert_ne!(sine, triangle);
            assert_ne!(sine, square);
            assert!(sine
                .iter()
                .chain(&triangle)
                .chain(&square)
                .all(|f| f.is_finite() && f.abs() <= 0.8));
        }
    }

    #[test]
    fn every_tuning_endpoint_stays_finite_for_all_friction_models() {
        for maximum in [false, true] {
            for friction_model in [
                FrictionModel::Smooth,
                FrictionModel::Stribeck,
                FrictionModel::LuGre,
            ] {
                let mut values = serde_json::json!({});
                for parameter in omsi_launcher_lib::ffb::PARAMETERS {
                    values[parameter.key] = serde_json::json!(if maximum {
                        parameter.max
                    } else {
                        parameter.min
                    });
                }
                let settings = Settings {
                    friction_model,
                    ..Settings::from_json(&values)
                };
                let mut model = ForceFeedback::default();
                model.configure(settings);
                for i in 0..1000 {
                    let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
                    let input = Telemetry {
                        steering: [sign; 2],
                        suspension: [sign, -sign],
                        rear_suspension: [-sign; 2],
                        acceleration: [100.0 * sign; 3],
                        script_amplitude: 1.0,
                        surface: [2.0; 2],
                        ..corner(100.0)
                    };
                    let output = model.update(&input, sign, 2880.0, 0.01, 2.0, 2.0);
                    assert!(
                        output.is_finite()
                            && output.abs() <= settings.overall_gain * settings.output_limit
                    );
                }
            }
        }
    }

    #[test]
    fn matches_original_cpp_telemetry_trace_with_supplied_ini() {
        // Generated by compiling the ZIP's force_model.hpp with the numeric
        // ModelSettings from OMSI2DirectInputFFB.ini. This trace covers rolling,
        // calibration, lock-up, two asymmetric impacts, then stopping.
        let expected = [
            (0, [0.0, 0.0, 0.053, 0.14, 0.0, 16.0, 0.0]),
            (1, [-0.164020810, 0.0, 0.257, 0.063, 0.028346869, 30.0, 0.0]),
            (80, [-0.416857919, 0.0, 0.257, 0.063, 0.1, 30.0, 0.0]),
            (81, [-0.329543407, 0.0, 0.257, 0.063, 0.1, 30.0, 0.0]),
            (
                100,
                [
                    -0.194958677,
                    -0.446658029,
                    0.257,
                    0.063,
                    0.103967460,
                    30.0,
                    0.22,
                ],
            ),
            (
                101,
                [
                    -0.194954713,
                    0.638563240,
                    0.257,
                    0.063,
                    0.108340346,
                    30.0,
                    0.22,
                ],
            ),
            (250, [0.0, -0.000000030, 0.053, 0.14, 0.0, 16.0, 0.0]),
        ];
        let mut model = ForceFeedback::default();
        for i in 0..=250 {
            let mut t = Telemetry::default();
            if i > 0 && i < 120 {
                t.speed_kmh = 36.0;
                t.steering = [0.3; 2];
                t.acceleration[0] = 3.5;
                t.surface = [2.0; 2];
                t.wheel_rpm = [10.0 / 0.5 * 60.0 / TAU; 2];
            }
            if (81..120).contains(&i) {
                t.wheel_rpm = [0.0; 2];
                t.brake = 1.0;
                t.acceleration[1] = -3.0;
            }
            if i == 100 {
                t.suspension[0] = -0.04;
                t.acceleration[2] = 0.8;
            }
            if i == 101 {
                t.suspension[1] = -0.04;
                t.acceleration[2] = -0.8;
            }
            let f = model.telemetry(&t, 0.02);
            if let Some((_, values)) = expected.iter().find(|(frame, _)| *frame == i) {
                let actual = [
                    f.aligning,
                    f.road_kick,
                    f.damper,
                    f.friction,
                    f.surface,
                    f.surface_hz,
                    f.impact,
                ];
                for (a, b) in actual.into_iter().zip(values) {
                    assert!((a - b).abs() < 2e-6, "frame {i}: Rust {a}, C++ {b}");
                }
            }
        }
    }

    fn corner(speed: f32) -> Telemetry {
        Telemetry {
            speed_kmh: speed,
            steering: [0.3; 2],
            ..Default::default()
        }
    }

    fn settled(input: &Telemetry) -> Forces {
        let mut model = ForceFeedback::default();
        for _ in 0..200 {
            model.telemetry(input, 0.02);
        }
        model.telemetry(input, 0.02)
    }

    #[test]
    fn centring_uses_axle_radians_builds_with_speed_and_saturates() {
        assert_eq!(settled(&corner(0.0)).aligning, 0.0);
        let town = settled(&corner(10.0)).aligning;
        let road = settled(&corner(35.0)).aligning;
        assert!(road < town && town < 0.0);
        let left = settled(&Telemetry {
            steering: [-0.3; 2],
            ..corner(35.0)
        })
        .aligning;
        assert!((road + left).abs() < 1e-6);
        let lock = settled(&Telemetry {
            steering: [0.9; 2],
            ..corner(35.0)
        })
        .aligning;
        assert!(lock < road && lock.abs() < road.abs() * 2.0);
        assert_eq!(
            settled(&Telemetry {
                steering: [0.003; 2],
                ..corner(35.0)
            })
            .aligning,
            0.0
        );
    }

    #[test]
    fn braking_loads_the_front_in_both_travel_directions() {
        let coast = settled(&corner(35.0)).aligning.abs();
        let braking = settled(&Telemetry {
            acceleration: [0.0, -3.0, 0.0],
            brake: 0.7,
            ..corner(35.0)
        })
        .aligning
        .abs();
        let reverse = settled(&Telemetry {
            acceleration: [0.0, 3.0, 0.0],
            brake: 0.7,
            ..corner(-35.0)
        })
        .aligning
        .abs();
        let accelerating = settled(&Telemetry {
            acceleration: [0.0, 3.0, 0.0],
            ..corner(35.0)
        })
        .aligning
        .abs();
        assert!(braking > coast && coast > accelerating);
        assert!((braking - reverse).abs() < 1e-6);
    }

    #[test]
    fn saturated_tyres_and_wet_roads_lighten_steering() {
        let peak = settled(&Telemetry {
            acceleration: [3.5, 0.0, 0.0],
            ..corner(35.0)
        })
        .aligning
        .abs();
        let skid = settled(&Telemetry {
            acceleration: [14.0, 0.0, 0.0],
            ..corner(35.0)
        })
        .aligning
        .abs();
        let wet = settled(&Telemetry {
            street_condition: 2.0,
            acceleration: [3.5, 0.0, 0.0],
            ..corner(35.0)
        })
        .aligning
        .abs();
        assert!(skid < peak && wet < peak);
    }

    #[test]
    fn locked_front_wheels_lighten_only_after_valid_rolling_calibration() {
        let mut model = ForceFeedback::default();
        let mut t = corner(36.0);
        t.wheel_rpm = [10.0 / 0.5 * 60.0 / TAU; 2];
        for _ in 0..200 {
            model.telemetry(&t, 0.02);
        }
        let rolling = model.telemetry(&t, 0.02).aligning.abs();
        t.wheel_rpm = [0.0; 2];
        t.brake = 1.0;
        for _ in 0..200 {
            model.telemetry(&t, 0.02);
        }
        let locked = model.telemetry(&t, 0.02).aligning.abs();
        assert!(locked < rolling * 0.6);
        assert!(
            settled(&t).aligning.abs() > rolling,
            "missing RPM must not imply lock-up without calibration"
        );
    }

    #[test]
    fn one_wheel_kicks_are_signed_and_static_camber_decays() {
        let mut left = ForceFeedback::default();
        let mut right = ForceFeedback::default();
        let mut t = Telemetry {
            speed_kmh: 20.0,
            ..Default::default()
        };
        left.telemetry(&t, 0.02);
        right.telemetry(&t, 0.02);
        t.suspension = [-0.04, 0.0];
        let l = left.telemetry(&t, 0.02).road_kick;
        let r = right
            .telemetry(
                &Telemetry {
                    suspension: [0.0, -0.04],
                    ..t
                },
                0.02,
            )
            .road_kick;
        assert!(l.abs() > 0.01 && (l + r).abs() < 1e-6);
        for _ in 0..400 {
            left.telemetry(&t, 0.02);
        }
        assert!(left.telemetry(&t, 0.02).road_kick.abs() < 1e-5);
    }

    #[test]
    fn front_impacts_are_stronger_than_rear_and_have_a_release_tail() {
        let mut front = ForceFeedback::default();
        let mut rear = ForceFeedback::default();
        let mut t = Telemetry {
            speed_kmh: 3.0,
            ..Default::default()
        };
        front.telemetry(&t, 0.02);
        rear.telemetry(&t, 0.02);
        t.acceleration[2] = 0.8;
        let f = front.telemetry(
            &Telemetry {
                suspension: [-0.005; 2],
                ..t
            },
            0.02,
        );
        let r = rear.telemetry(
            &Telemetry {
                rear_suspension: [-0.005; 2],
                ..t
            },
            0.02,
        );
        assert!(f.impact > r.impact && r.impact > 0.0);
        assert!(f.new_impact);
        t.acceleration[2] = 0.0;
        t.suspension = [-0.005; 2];
        assert!(front.telemetry(&t, 0.02).impact > 0.0);
        for _ in 0..400 {
            front.telemetry(&t, 0.02);
        }
        assert_eq!(front.telemetry(&t, 0.02).impact, 0.0);
        t.suspension = [-0.01; 2];
        assert!(front.telemetry(&t, 0.02).new_impact);
    }

    #[test]
    fn texture_depends_on_surface_and_stops_at_rest() {
        let asphalt = settled(&corner(35.0));
        let cobble = settled(&Telemetry {
            surface: [2.0; 2],
            ..corner(35.0)
        });
        assert!(cobble.surface > asphalt.surface * 10.0);
        assert_eq!(
            settled(&Telemetry {
                surface: [2.0; 2],
                ..corner(0.0)
            })
            .surface,
            0.0
        );
        let mut texture = SurfaceTexture::default();
        let values: Vec<_> = (0..100)
            .map(|_| texture.update(0.1, 20.0, 0.01, Waveform::Triangle))
            .collect();
        assert!(values.iter().all(|v| v.abs() <= 0.1));
        assert!(values.iter().any(|v| *v < 0.0) && values.iter().any(|v| *v > 0.0));
    }

    #[test]
    fn resistance_opposes_raw_motion_without_an_initial_kick() {
        let t = Telemetry::default();
        let mut right = ForceFeedback::default();
        let mut left = ForceFeedback::default();
        assert_eq!(right.update(&t, 0.5, 900.0, 0.02, 1.0, 0.0), 0.0);
        assert_eq!(left.update(&t, -0.5, 900.0, 0.02, 1.0, 0.0), 0.0);
        let r = right.update(&t, 0.501, 900.0, 0.02, 1.0, 0.0);
        let l = left.update(&t, -0.501, 900.0, 0.02, 1.0, 0.0);
        assert!(r < 0.0 && l > 0.0 && (r + l).abs() < 1e-6);
        for _ in 0..200 {
            right.update(&t, 0.501, 900.0, 0.02, 1.0, 0.0);
        }
        assert!(right.update(&t, 0.501, 900.0, 0.02, 1.0, 0.0).abs() < 1e-6);
    }

    #[test]
    fn mute_reset_stalls_and_invalid_inputs_do_not_leave_old_forces() {
        let mut model = ForceFeedback::default();
        let t = corner(35.0);
        for _ in 0..200 {
            model.update(&t, 0.5, 900.0, 0.02, 1.0, 1.0);
        }
        assert_eq!(model.update(&t, 0.5, 900.0, 0.02, 0.0, 0.0), 0.0);
        for dt in [0.0, -1.0, f32::NAN, f32::INFINITY, 0.5] {
            assert_eq!(model.update(&t, 0.5, 900.0, dt, 1.0, 1.0), 0.0);
        }
        model.reset();
        assert_eq!(
            model.update(&Telemetry::default(), -0.8, 900.0, 0.02, 1.0, 1.0),
            0.0
        );
        let broken = Telemetry {
            speed_kmh: f32::NAN,
            steering: [f32::INFINITY; 2],
            suspension: [f32::NAN; 2],
            acceleration: [f32::NAN; 3],
            wheel_rpm: [f32::INFINITY; 2],
            script_amplitude: f32::NAN,
            ..Default::default()
        };
        assert!(model
            .update(&broken, 0.0, f32::NAN, 0.02, 1.0, 1.0)
            .is_finite());
    }

    #[test]
    fn low_vr_frame_rates_keep_script_vibration_bounded_and_mutable() {
        // FF_Vib_Period=2 requests 50 Hz, above what a frame-driven wheel can
        // reproduce at 25 or 30 FPS. The limiter must still yield a changing
        // signal, and a saved zero vibration scale must mute it immediately.
        let input = Telemetry { script_amplitude: 1.0, script_period: 2.0, ..Default::default() };
        for fps in [25.0, 30.0] {
            let mut model = ForceFeedback::default();
            let samples: Vec<f32> = (0..60)
                .map(|_| model.update(&input, 0.0, 900.0, 1.0 / fps, 1.0, 1.0))
                .collect();
            assert!(samples.iter().all(|f| f.is_finite() && f.abs() <= 0.8));
            assert!(samples.iter().any(|f| *f > 0.05));
            assert!(samples.iter().any(|f| *f < -0.05));
            assert_eq!(model.update(&input, 0.0, 900.0, 1.0 / fps, 1.0, 0.0), 0.0);
        }
    }

    #[test]
    fn output_is_bounded_and_channels_scale_independently() {
        let t = Telemetry {
            script_amplitude: 1.0,
            script_period: 5.0,
            ..Default::default()
        };
        assert_eq!(
            ForceFeedback::default().update(&t, 0.0, 900.0, 0.02, 1.0, 0.0),
            0.0
        );
        assert!(ForceFeedback::default().update(&t, 0.0, 900.0, 0.02, 0.0, 1.0) > 0.1);
        let mut model = ForceFeedback::default();
        for i in 0..1000 {
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            let t = Telemetry {
                steering: [sign; 2],
                suspension: [sign, -sign],
                acceleration: [100.0 * sign; 3],
                script_amplitude: 1.0,
                ..corner(100.0)
            };
            let output = model.update(&t, sign, 2880.0, 0.01, 2.0, 2.0);
            assert!(output.is_finite() && output.abs() <= 0.8);
        }
    }
}
