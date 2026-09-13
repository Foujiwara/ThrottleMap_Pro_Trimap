# Map and EEPROM format

## Runtime map

31 throttle points by 11 absolute-duty points.
Index = throttle_index * 11 + duty_index. Each cell is a signed byte
in -100..100, representing -1.00..1.00 at 1% resolution.
The buffer is mutable RAM, never constant flash.

The throttle axis is signed and has two slopes. Row 10 is zero throttle.
Rows 10..30 are traction at 5% intervals, so the traction half keeps the
resolution the 21x21 format had. Rows 0..9 are the brake half at 10%
intervals, row 0 being -100%. The duty axis is 10% intervals: 21 duty
columns plus a brake half does not fit 128 EEPROM slots, and the lookup
interpolates duty anyway.

Lookup accepts throttle in -1000..1000 and duty in 0..1000 and clamps at
the edges. The row position is `10000 + throttle * (throttle < 0 ? 10 :
20)`, kept positive so integer `mod` yields the interpolation weight
directly. It interpolates the four adjacent cells using integer
arithmetic; the result is scaled by 1000. Positive values command
relative propulsion current. Negative values command relative brake
current.

The brake half is only reached when the brake map is enabled and the
brake channel is above its deadband; otherwise the lever keeps the plain
proportional behaviour and only rows 10..30 are ever read.

The generator computes peak = throttle ^ torque_response, then blends
towards throttle above 60% according to high_hold. An exponent below 1
increases low-throttle response; above 1 softens it. With nonzero speed
coupling, balance duty = clamp(throttle * coupling), with a shaped taper
over transition_width before balance and overrun regen after balance.
The released row uses -engine_brake * duty ^ (1 + regen_curve).
Zero speed coupling selects duty-independent torque (Direct Electric),
while retaining the released-row brake. QML previews the same formula.

**The generator writes rows 10..30 only.** The brake half is hand
territory: no slider, preset or regeneration overwrites it. Only Reset to
defaults refills it, with a straight proportional brake that is flat
across duty.

## Brake type

Independent of the map, and applied to the brake lever only:

| Value | Behaviour |
| --- | --- |
| 0 | Regen only: `set-brake-rel` throughout, no torque against travel |
| 1 | Current, no reverse: regen while rolling, negative current below the ERPM threshold, then latched back to regen as soon as it stops (below 50 ERPM) until the lever is released. Without that latch the torque reverses the motor, regen catches it, the speed re-enters the torque band and it shunts backwards repeatedly |
| 2 | Current, bidirectional: as above, and it drives on into reverse once stopped |

Engine braking and overrun regen, which come from the map itself rather
than from the lever, always stay pure regen whatever this is set to. A
bidirectional brake type must never reverse a vehicle whose throttle is
merely released.

## EEPROM layout

There are 128 persistent 32-bit slots. Format marker 20260914. The header
is packed into 9 slots (i16 scaled by 1000 instead of float32) so that
the taller map still fits.

| Offset | Content |
| --- | --- |
| 0 | Format marker; 0 means incomplete/invalid |
| 4..7 | Throttle source, invert, brake mode, preset id (u8 each) |
| 8, 10 | Throttle min/max, i16 scaled by 1000 |
| 12, 14 | Deadband, filter alpha |
| 16, 18 | Torque response, speed coupling |
| 20, 22 | Transition width, high hold |
| 24, 26 | Engine brake, overrun regen |
| 28..31 | Transition shape, regen curve, brake map on, brake type (u8 each) |
| 32 | Reverse-threshold ERPM, i16 |
| 36..379 | Four map bytes per word; first cell in least significant byte |
| Slot 127 | CRC16 of slots 0..126 encoded as big-endian words |

That is 95 of the 127 data slots; the remaining ones are written as zero
and are free for later use.

Each cell is stored as signed_value + 128. Padding cells in the last slot
represent zero. Packing promotes to u32 **before** shifts; ordinary Lisp
integers cannot hold all four bytes on a 32-bit VESC. Reading arbitrary
words uses bufget-u32 then to-i32, avoiding versions of bufget-i32 which
return a narrowed fixnum.

Save snapshots the current map/configuration, pauses output until the operation
finishes, invalidates slot 0, writes changed slots only, checks every written
word by re-reading it, writes the checksum, and commits the marker last.
An identical save performs no flash writes. eeprom-store-i already writes
flash; conf-store is unnecessary and would also persist unrelated app/motor
settings.

Load checks marker, field ranges, map byte ranges, completeness and checksum
before applying anything. Invalid data leaves the active configuration alone;
at boot only, failed load generates the default map.

## Migrating a 21x21 image

Markers 20260913 and 20260912 are read, validated against the old layout
and converted in memory: the 21 old throttle rows become rows 10..30
unchanged, the duty axis keeps every other column (old index `2*d`, which
lands exactly on the new 10% grid), and the brake half is filled with the
default proportional brake. Brake map off, brake type regen-only.

Nothing is written back until the next explicit Save, which stores the
new format. The conversion is one-way: a stable-package install reading
back a 20260914 image rejects it as invalid and falls back to defaults
rather than misreading it.

This is a single-image store: a power cut during an update can invalidate
the previous save. It detects partial updates, but does not promise rollback.
EEPROM is shared with other Lisp packages and is not preserved across every
firmware update. Save only while stopped; motor output is paused until the
operation completes. Errors produce a failure status, never "saved".
