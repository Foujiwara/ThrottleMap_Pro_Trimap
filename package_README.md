# CarMap Beta (Brake Map)

**Beta fork of CarMap Thermal Throttle.** A configurable 31 x 11
throttle/duty map drives relative propulsion current and relative brake
current. Native VESC current, voltage, temperature and speed limits
remain active.

The vertical axis now spans **-100% to +100% throttle**: rows above zero
are traction (5% steps, the same resolution as before), rows below zero
are the brake half (10% steps), reached when the optional **brake map**
is enabled and the brake lever is pulled. The duty axis moved to 10%
steps to make room in the EEPROM; the lookup interpolates duty, so
nothing about the feel changes.

Each half has its own generator and its own regenerate flag, so shaping
one never discards hand edits made to the other. The brake generator
defaults to a negative current **proportional to both lever travel and
duty** - the mirror of the traction side, no bite at a standstill, full
bite at full speed - and exposes four settings: brake strength at full
lever, lever response, speed dependence (0 makes it flat across speed,
i.e. the plain proportional lever brake) and the speed curve.

**Brake type** (lever only):

- *Regen only* - braking never produces torque against travel.
- *Current, no reverse* - regen while rolling, negative current below the
  ERPM threshold so it pulls to a stop and holds, and regen again rather
  than driving if it ever rolls backwards.
- *Current, bidirectional* - as above, then on into reverse once stopped.

Engine braking and overrun regen always stay pure regen regardless, so a
released throttle can never reverse the vehicle.

An existing 21x21 EEPROM image is converted on read (old rows become the
traction half, every other duty column kept). Nothing is written back
until you Save; going back to the stable package means reconfiguring.

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
