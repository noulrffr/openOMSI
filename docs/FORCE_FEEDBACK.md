# Telemetry force feedback

The native wheel feedback includes a Rust port of the force model in
`OMSI2_DirectInput_FFB_Telemetry_Physics_Update.zip`, using the tuning in its
`OMSI2DirectInputFFB.ini`. It is enabled by default in this build.

In **Settings → Driving → Game controllers**, leave **Force feedback and vibration**
and **Telemetry force feedback** on. Switch **Telemetry force feedback** off to
compare with the previous openOMSI model. The saved setting is `ff_telemetry=1`
(ported model) or `ff_telemetry=0` (previous model).

Set **Wheel rotation** to the physical rotation configured in your wheel driver.
Select your wheel under **Controls → Game controllers → device → Force feedback**
to adjust its own tuning. The old Steering force and Vibration sliders are replaced
by these controls; their `[FFScale]` values no longer multiply telemetry feedback.
**Invert force feedback** still reverses the final wheel force. The port uses
openOMSI's steering coordinates; the plugin's hardware-specific
`force_direction` and `resistance_direction` defaults are normalized to these
coordinates. Both can be changed separately on the Force feedback tab.
The device's **Invert force feedback** switch is on that tab too. The setup
wizard can detect the direction with a brief motor pulse; its choice is saved
for that wheel and overrides the global default.

This runs inside openOMSI; installing the OPL or running its DirectInput worker is
unnecessary. If the original FFB plugin is enabled in openOMSI's plugin list,
disable that plugin before using native wheel feedback so the two implementations
do not compete for the same device. No OMSI bus files need patching.

## What was migrated

* Nonlinear caster return based on road-wheel **radians**, building between
  1.5 and 35 km/h, with no standing centring spring in the supplied preset.
* Lateral-load saturation, measured braking/acceleration load transfer,
  wet-road grip reduction and front-wheel lock/slip lightening after rolling-radius
  calibration from RPM.
* Software damping and selectable smooth, Stribeck or LuGre friction from physical
  wheel movement, before controller deadzone and steering-range scaling.
* Signed front suspension kicks, with a decaying displacement term to prevent
  permanent camber pull; front/rear impact separation and a short release tail.
* Four-band road texture and separately triggered impacts, with sine, triangle or
  square waveforms (the preset uses triangle texture and 12 Hz sine impacts).
  Existing `FF_Vib_Amp` / `FF_Vib_Period` script vibration is also retained.

The supplied preset's master gain is **0.8**, with a final output limit of **1**
before that gain, a base-force slew limit of **2 units/s**, alignment strength
**0 → 1**, damping **0.053 → 0.257**, and friction **0.14 → 0.063**.
Each device has independent overall gain, centring/resistance and vibration tuning.

## Tuning in openOMSI

Open **Controls → Game controllers**, select a device, then its **Force feedback**
tab. **Axes and buttons** stays next to it. The **74 controls** cover:

| Group | Controls |
| --- | --- |
| Output | Master gain, output/base-torque limits, torque slew, base/resistance direction |
| Centring | Rest/speed strength, speed thresholds, deadband, steering response, angle saturation, tyre onset, response time |
| Grip and load | Lateral/braking load, wet grip, slip thresholds and force loss, rolling-radius calibration |
| Damping | Enable software damping, rest/speed strength, force limit, wheel-rate filter/limit, load and fast-movement gains |
| Friction | Enable software friction, smooth/Stribeck/LuGre model, rest/speed strength, force limit, crossing speed, static ratio, Stribeck speed, viscous gain and LuGre coefficients |
| Road kicks | Direction, gain/limit, speed weighting, suspension filter, displacement gain and camber decay |
| Impacts | Vertical/suspension thresholds and gains, minimum/maximum strength, release, frequency/waveform, low-speed weighting and rear-axle rejection |
| Road vibration | Surface gain/limit, response, frequency range and waveform |

Scroll to reach the lower groups. Press **Save** in the device list, then **restart
a running game to use the changes**. These controls affect the telemetry model; keep **Telemetry
force feedback** enabled under Driving. **Restore plugin tuning** restores just
these 74 values for the selected device to the supplied INI preset. Other devices,
axis/button bindings and the global feedback switches keep their values. Saved
devices can be tuned while disconnected.

Smooth friction gives resistance that rises smoothly around zero wheel speed.
Stribeck adds stronger breakaway friction at low speed. LuGre also retains a small
elastic friction force when the wheel stops; its state clears on pause/reset.
Turning off **Software damping** or **Software friction** disables that component;
it does not select a native driver condition effect.

Values are stored in the content folder's `Inputs/gamectrler.cfg`, in an
`[openomsi_ffb]` block within each device's `[ctrl]` entry. Keys use `ffb_` before
the original plugin parameter name, for example:

```ini
[openomsi_ffb]
ffb_overall_gain=0.8
ffb_align_at_rest=0
ffb_damper_at_rest=0.053
ffb_calculated_friction=true
ffb_friction_model=smooth
ffb_rumble_waveform=triangle
ffb_impact_rumble_frequency_hz=12
```

Edit the file with the launcher closed so its next save does not replace your edits. Invalid numbers
fall back to defaults; numeric values and conflicting minimum/maximum pairs are
bounded consistently by the launcher and game. The original plugin INI is not
loaded automatically. The shared defaults and full key/range list are in
`crates/omsi-launcher-core/src/ffb.rs`.

For migration from the earlier build, devices without a saved profile start from
the old `ffb_*` values in `~/.openomsi/settings.cfg` (or the plugin defaults when
absent). Saving controller settings gives each device its own complete profile.
After that, changes to the old global values do not override those profiles.
Old `[FFScale]` values remain in the file for compatibility with the previous
openOMSI feedback model, selected by turning Telemetry force feedback off.

## Differences and limits

openOMSI supplies suspension, acceleration, wheel RPM and Ackermann steering angles
directly from its physics, including buses whose scripts omit those variables.
Axle selection uses geometry rather than assuming axle 0 is always the front.
The physics itself has not been replaced, so the feel can differ from OMSI 2 even
when the force equations match.

The port reads `Axle_SurfaceID_<front axle>_L/R` when available. openOMSI currently
does not populate those material variables from ground contacts; missing/default
IDs use asphalt. Suspension and acceleration feedback still work, but material
texture cannot reliably distinguish cobblestones from asphalt yet.

Force updates use openOMSI's existing frame-driven Windows DirectInput / Linux
evdev output, capped at 100 Hz, rather than the plugin's separate 120 Hz worker.
Vibration frequencies are limited to suit the available sample rate; low FPS can
therefore change the vibration feel. The model resets after a frame gap over
300 ms. Device effects expire after 300 ms without refresh, and stopping feedback
bypasses normal output throttling. Pause, focus loss and controller changes reset
the model. Gamepads apply their own profile's overall gain and output limit to
rumble; steering-specific effects need a force-feedback wheel. macOS gains no
native wheel-torque backend from this change.

Native DirectInput condition/periodic effect fallbacks, the plugin's device and
cooperative-mode options, worker timing, live tuning window, hotkeys and
visual-rotation patcher are not part of this port. Device selection and output
timing remain managed by openOMSI. The feedback uses software constant force.

## Verification

Tests cover a telemetry trace compared with the original C++ model and supplied
INI, centring direction and saturation, braking/reversing, wet grip, wheel lock-up,
front/rear impacts, surface texture, raw wheel resistance, output limits,
invalid/stale frames, telemetry units, all settings saving/reloading, launcher
controls/reset, nondefault force limits and directions, friction modes, waveform
selection, and finite output at the tuning range extremes. Device tests cover
separate profiles, migration defaults, saving/reloading every parameter, binding
preservation, device-specific torque/rumble and removal of the old multipliers.

```text
cargo test --workspace --locked
cargo build --release --locked
```

Wheel-driver polarity, physical feel and device timeout behavior require a real
wheel test. Compare both modes on the same bus, wheel settings and route: parking,
10–35 km/h turns, firm braking, and an isolated front/rear axle bump.
