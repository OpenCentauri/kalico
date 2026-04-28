# CANVAS Motor And Odometer Notes

This file documents the motor-drive and odometer behavior recovered from the closed firmware in canvas_firm.c and canvas_firm.asm.

Pin assignments are intentionally omitted. This note is about behavior, sequencing, and math.

## Scope

- There are four logical motor channels.
- Each channel has both motor-hall feedback and odometer feedback.
- Channel numbers in this note refer to the firmware's logical channels 0 through 3.

## Power Sequencing

- The DRV8833 supply is split into two separately enabled rails in firmware: a 24 V rail and a 9 V rail.
- The startup task waits until the 24 V rail is reported on, delays about 2 seconds, then enables the 9 V rail.
- In practice, the firmware behavior and hardware testing both point to the same conclusion: motion is not expected to work correctly with only the 24 V rail enabled.
- If you are bringing the hardware up externally, the stock order is:

1. Turn on the 24 V rail.
2. Wait for the rail to be stable.
3. Wait about 2 seconds, matching the firmware startup delay.
4. Turn on the 9 V rail.
5. Only then issue motor drive commands.

Relevant functions:

- sub_802b6fc: DRV8833_24VPowerOn
- sub_802b754: DRV8833_9VPowerOn
- sub_8017598: periodic startup task that waits for 24 V, delays, then enables 9 V

## Motor Drive Model

- Each motor is driven as a two-input H-bridge, not as a single direction pin plus an enable pin.
- The normal motion path drives one H-bridge leg with PWM and forces the opposite leg to 0.
- The firmware does not show the usual DRV8833 "PWM one side, opposite side forced high" slow-decay drive pattern in the normal motion path.
- Two wrapper functions exist for different channel layouts in memory, but the direction semantics are the same:

1. Mode 1: one leg gets PWM, the other leg is driven to 0.
2. Mode 2: the opposite leg gets PWM, the first leg is driven to 0.
3. Mode 3: stop path, both legs go through the stop helper.

Relevant functions:

- sub_802ba8c: bridge-drive helper for one motor-group layout
- sub_802bb40: bridge-drive helper for the other motor-group layout
- sub_802b830: converts duty percent into a timer compare value
- sub_801c3ac: motor_stop, which clears the run flag and routes through the stop mode

## PWM Behavior

- The stock firmware uses a fixed 20 kHz PWM carrier.
- The firmware changes speed by changing duty cycle, not by changing PWM frequency.
- The stored duty value is a percentage in the range 0 to 100.
- Duty initializes to 50% when the motor-control state is created.
- Duty is clamped to 0 through 100 during control updates.

Practical consequence:

- If you drive the motors yourself at low PWM frequency, for example around 1 kHz, the sound will be much worse than stock.
- To match stock behavior, keep the carrier high and vary duty only.

## Motion Commands

The firmware exposes multiple motor-control surfaces:

- sub_801bff4: motor_start_load(channel, speed_mm_s)
- sub_801c0b8: motor_start_unload(channel, speed_mm_s)
- sub_801c17c: motor_move_to_position(channel, pulses, pulses_per_second)
- sub_801c2cc: motor_move_relative_circles(channel, circles, speed)
- sub_801c3ac: motor_stop(channel)
- sub_801c564: motor_clear_material_distance(channel)

Important details:

- Continuous forward and reverse motion are commanded in mm/s.
- Position-domain moves are commanded in pulses.
- Relative circle moves are converted to pulses using the configured motor pulses-per-revolution value.
- Clearing material distance only resets the odometer-derived distance accumulator. It does not reset the duty-control state.

## Preload And Pressure Relief

- Preload starts by calling motor_start_load with the configured preload speed.
- The preload-speed getter is sub_802e978 and the setter is sub_802e990.
- The factory default preload speed is 80 mm/s.

Pressure relief appears to be motion-based rather than actuator-based:

- No separate clutch, solenoid, or dedicated "pressure release" output was identified in the motion path.
- The firmware behavior is consistent with pressure relief being achieved by reversing or relaxing the feed train through pre-unload and unload motion.
- That matches the observed geared mechanism behavior: the assembly relieves pressure by motor motion, not by a separate hardware output.

## Control Loop Overview

The main runtime update is sub_801c824, which runs once per control-cycle iteration.

Per channel, it does the following:

1. Read the current motor-hall pulse count.
2. Read the current odometer pulse count.
3. Compute the elapsed time since the previous sample.
4. Compute a motor-derived linear speed estimate.
5. Compute an odometer-derived linear speed estimate.
6. Integrate odometer distance into a material-distance accumulator.
7. If the channel is running in continuous speed mode, update PWM duty through a PID-like correction term.

Important: the visible speed-control branch uses the motor-derived linear speed as its direct feedback term, not the odometer-derived linear speed.

The odometer speed is still important because it is used for actual material-distance bookkeeping.

## Default Calibration And Controller Values

Recovered factory defaults:

- Motor pulses per revolution: 6
- Odometer pulses per revolution: 6
- Odometer distance per revolution: 29.516 mm
- Motor revolutions per odometer revolution: 28.380
- Preload speed: 80 mm/s
- Kp: 2.0
- Ki: 0.0
- Kd: 0.1
- Initial duty: 50

Derived scales from those defaults:

- Motor linear travel per motor-hall pulse: about 0.17334 mm/pulse
- Odometer linear travel per odometer pulse: about 4.91933 mm/pulse

## Motor-Hall Based Speed Estimate

The firmware's motor-side linear speed estimate is:

```text
dt_s = delta_ms / 1000.0

v_motor_mm_s =
    (delta_motor_pulses / motor_pulses_per_revolution)
    * (odometer_distance_per_revolution_mm / motor_revolutions_per_odometer_revolution)
    / dt_s
```

With stock defaults:

```text
v_motor_mm_s = (delta_motor_pulses / 6) * (29.516 / 28.380) / dt_s
```

This is the feedback term used in the normal duty-control branch.

## Odometer Behavior

- The odometer inputs are treated as pulse counters.
- One firmware "pulse" means one odometer interrupt event counted by the shared EXTI callback.
- The odometer side is not treated as quadrature in the recovered code.
- The code examined here proves the count-increment behavior, but this note does not claim 100% certainty on whether hardware EXTI is configured for one edge or both. External code should match the actual sensor behavior seen on the board.

Relevant functions:

- sub_801e08c: shared hall/motor/odometer EXTI callback
- sub_801e20c: returns odometer pulse count for a logical channel
- sub_802b4e4: reads the packed odometer GPIO state

## Odometer Distance And Speed Math

Distance per pulse is determined by the configured odometer geometry:

```text
mm_per_odo_pulse = odometer_distance_per_revolution_mm / odometer_pulses_per_revolution
```

With stock defaults:

```text
mm_per_odo_pulse = 29.516 / 6 = 4.91933 mm
```

The odometer-based linear speed estimate is:

```text
v_odo_mm_s =
    (delta_odo_pulses / odometer_pulses_per_revolution)
    * odometer_distance_per_revolution_mm
    / dt_s
```

With stock defaults:

```text
v_odo_mm_s = (delta_odo_pulses / 6) * 29.516 / dt_s
```

The material-distance accumulator uses the same geometry without the time division:

```text
delta_material_mm =
    (delta_odo_pulses / odometer_pulses_per_revolution)
    * odometer_distance_per_revolution_mm

material_distance_mm += delta_material_mm
```

With stock defaults, every counted odometer pulse corresponds to about 4.919 mm of material travel.

## Duty Control Law

In continuous speed mode, the visible control branch behaves like this:

```text
error = target_mm_s - v_motor_mm_s
integral = clamp(integral + error, 0, 100)
derivative = error - previous_error

delta_duty = Kp * error + Ki * integral + Kd * derivative
duty = clamp(duty + delta_duty, 0, 100)
```

With stock defaults:

```text
Kp = 2.0
Ki = 0.0
Kd = 0.1
```

So the stock controller is effectively PD by default, even though the integral accumulator exists.

Additional runtime behaviors worth preserving if you want stock-like response:

- If no new motor-hall pulses arrive, the stored motor-speed estimate is decayed by 0.5 each update until it approaches zero.
- If a pulse delta appears negative, the firmware treats it as overflow and keeps the previous speed estimate for that update.
- The integral accumulator is clamped to 0 through 100 before it is used.
- The final duty is also clamped to 0 through 100.

## What The Odometer Is For

The odometer is best understood as the firmware's estimate of actual filament travel.

That means:

- Odometer pulses represent traveled material distance.
- Material-distance tracking is odometer-derived, not motor-derived.
- Remaining-distance / additional-push / additional-unload features are grounded in odometer mm, which helps when motor rotation and actual material travel are not perfectly matched.
- The normal visible speed loop still closes on motor-derived speed, so odometer speed is tracked but not used as the direct duty feedback term in that branch.

## Practical Reimplementation Guidance

If you want to mimic the stock firmware behavior externally:

1. Bring up both power rails in the stock order before commanding motion.
2. Treat each motor as a two-input H-bridge.
3. Use a fixed 20 kHz PWM carrier.
4. For normal forward or reverse drive, PWM one bridge leg and hold the opposite leg low.
5. Command continuous motion in mm/s.
6. Use motor-hall pulses for the direct speed-feedback loop if you want the stock controller behavior.
7. Use odometer pulses for actual distance traveled and any slip-aware material limiter.
8. Keep the geometry values configurable: motor PPR, odometer PPR, odometer mm/rev, and motor rev per odometer rev.

## Key Firmware Entry Points

- sub_8017598: startup sequence and periodic task setup
- sub_801bff4: start load
- sub_801c0b8: start unload
- sub_801c2cc: relative move helper
- sub_801c3ac: stop
- sub_801c564: clear material distance
- sub_801c824: main motor-control update loop
- sub_801e08c: shared EXTI pulse callback
- sub_801e1c8: motor-hall pulse getter
- sub_801e20c: odometer pulse getter
- sub_802b6fc: 24 V enable helper
- sub_802b754: 9 V enable helper
- sub_802ba8c / sub_802bb40: H-bridge direction and duty helpers
- sub_802e978 / sub_802e990: preload speed getter and setter
