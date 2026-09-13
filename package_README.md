# CarMap Beta (Brake Map)

**Beta fork of CarMap Thermal Throttle.** A configurable 441-cell
throttle/duty map drives relative propulsion current and relative brake
current. Native VESC current, voltage, temperature and speed limits
remain active.

The grid is **ragged**, on purpose:

- **Traction rows**: throttle 0..100% in 5% steps (the same resolution as
  the stable package), duty 0..100%.
- **Brake rows**: lever 0..-100% in 10% steps, duty **-100%..+100%**. The
  right half is braking against speed, the left half is reverse.

Negative duty under a positive throttle is not stored at all - it only
ever means "full forward torque" - and that saved space is what pays for
the traction half keeping its 5% steps.

Each half has its own generator and its own regenerate flag, so shaping
one never discards hand edits made to the other. The brake generator
defaults to -10% lever = -0.10 at any forward speed, backing up to 10%
duty before it stops pulling.

**Brake type** (lever only):

- *Regen only* - braking never produces torque against travel.
- *Current, no reverse* - regen while rolling, negative current below the
  ERPM threshold so it pulls to a stop, then latched back to regen so it
  can never shunt backwards.
- *Current, bidirectional* - as above, then on into reverse once stopped.

Engine braking and overrun regen always stay pure regen regardless, so a
released throttle can never reverse the vehicle. Only a bidirectional
brake ever reads the reverse columns.

An existing 21x21 EEPROM image is converted on read (old rows become the
traction half, every other duty column kept). Earlier beta formats are
rejected and fall back to defaults.

## Setup

1. Keep the ADC/PPM input app enabled and set its **Control Type** to **Off**,
   so it decodes the input without also commanding the motor.
2. Open **CarMap**, select and calibrate the input, then apply throttle settings.
3. Select a preset or adjust the generator and press **Apply parameters**.
4. Use **Save to VESC** while stopped. This uploads the displayed map and
   parameters, writes EEPROM and verifies the result. Wait for “saved and
   verified”; output is paused during saving.
5. Use **Load from VESC** to check the stored data. Restart the controller
   and check again before relying on the new settings.

Version 0.1.52 fixes persistence, mutable-map flash placement, custom generator
application, Direct Electric behavior and imported-map overwrites. Valid
20260912 EEPROM saves migrate automatically on the next save; corrupt or
older formats require reconfiguration. An interrupted save is detected and
may require saving again.

Test mode has a STOP button and a 500 ms communication watchdog. UART and
PPM also stop commanding current when input updates expire. Test with the
wheel unloaded after installation; automated tests use simulated hardware,
not a connected VESC.

Source documentation includes the audit report and repeatable 32-bit LispBM tests.
