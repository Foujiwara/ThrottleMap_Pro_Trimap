# Map and EEPROM format

## Runtime map

The grid is deliberately **ragged**, because a rectangular one would spend
a quarter of the EEPROM on a region holding no independent information:
for traction, duty is speed and its sign carries nothing extra, so the
negative side is the positive side mirrored.

| Rows | Throttle | Columns | Duty |
| --- | --- | --- | --- |
| 0..10 | brake lever -100%..0, 10% steps | 21 | -100%..+100%, 10% steps |
| 11..30 | throttle 5..100%, 5% steps | 11 | 0..100%, 10% steps, read mirrored for negative duty |

`11*21 + 20*11 = 451` cells. Each is a signed byte in -100..100,
representing -1.00..1.00 at 1% resolution. The buffer is mutable RAM,
never constant flash.

Row 10 is both zero throttle and zero lever, and stores the full 21
columns. Its right half is engine braking against forward speed; its left
half belongs to the reverse block and is read by **both** lookups - a
light pull on the lever fades in from it, and a released throttle while
rolling backwards reads it directly. With the brake map off it is inert
and the UI shows it as a mirror like any other throttle row.

Those cells are positive by default: forward torque against a backwards
roll, which holds the vehicle on a slope instead of letting it run away.
Set the reverse runaway hold to 0 to freewheel there instead.
Being a real row rather than an implicit zero also means a light pull on
the lever fades in from whatever engine braking is doing, not from
nothing.

Every view draws 21 display columns per row. On a throttle row (11..30)
the left half is only a mirror of the right, so it is drawn dimmed and
editing either side edits the same cell.

## Lookup

Two lookups, each bilinear on its own uniform grid; nothing ever blends
across the seam, since throttle and brake are separate inputs.

- **throttle >= 0**: rows 11..30 against `|duty|`, so rolling backwards
  reads the same curve as rolling forwards at that speed. Row 10 is the
  exception: with the brake map on, negative duty reads its own left-half
  cells (column `10 - d`) rather than the mirror. Engine braking
  and the balance point therefore work in both directions, and nothing is
  discontinuous through zero. Row 10 stores 21 columns, so its forward
  duty sits at `10 + d` - `map-drive-get` handles that.
- **throttle < 0**: the brake rows against signed duty -1000..1000.

Positive values command relative propulsion current, negative values
relative brake current. All intermediates fit the VESC signed 28-bit
inline integer.

## Generators

Traction rows come from the thermal law: peak = throttle ^ torque_response
blended towards throttle above 60% by high_hold, a balance duty of
clamp(throttle * coupling), a shaped taper over transition_width before
it and overrun regen after. The released row uses
`-engine_brake * duty ^ (1 + regen_curve)`. Zero speed coupling selects
duty-independent torque (Direct Electric). QML previews the same formula.

There are **three generators**, one per region of the graph, each with its
own regenerate flag so shaping one never discards hand edits made to
another:

1. **Traction** - rows 10..30, the thermal law above, forward duty only.
2. **Brake** - brake rows, duty > 0. A plateau by default:

```
-(brake_strength * lever^brake_response
  * ((1 - duty_dep) + duty_dep * duty^(1 + brake_curve)))
```

3. **Reverse** - brake rows *and the released row*, duty <= 0, since that
   whole region belongs to this editor. The traction law **negated**, with
   its own eight settings, over reverse duty. Row 10 is its zero-lever
   edge: peak and balance are both zero there, so every column falls in the
   runaway branch, `+rev_overrun * rev^(1 + rev_curve)`, continuous with
   the -10% row above it. `gen-rev-half` literally
   calls `thermal-cell` and negates the result, so the two regions cannot
   drift apart:

```
peak    = rev_strength * thermal_peak(lever, rev_response, rev_hold)
balance = clamp01(lever * rev_coupling)
cell    = -thermal_cell(lever, rev, peak, balance, rev_coupling,
                        rev_width, rev_shape, 0, rev_overrun, rev_curve)
```

Brake defaults 1.0 / 1.0 / 0.0 / 0 give -10% lever = -0.10 at any forward
speed. Reverse defaults 1.0 / 1.0 / 0.0 / 1.0 / 0.10 / squared / 0.12 /
progressive give -30% lever backing up to 30% duty and then holding
there. `duty_dep` fades braking out towards a standstill;
`rev_coupling = 0` removes the reverse balance and leaves a plateau.

The positive cells past the reverse balance are forward torque, which
holds a reverse that is running away. Only a bidirectional brake reaches
them - for the other two types the control loop clamps the lever result to
zero or below, so the brake half is pure braking.

## Brake type

Applied to the brake lever only:

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

There are 128 persistent 32-bit slots. Format marker 20260920. The header
is packed into 9 slots (i16 scaled by 1000 instead of float32) and the
generator settings sit in a tail after the map.

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
| 34, 35 | Reverse transition shape, reverse runaway curve (u8 each) |
| 36..487 | Four map bytes per word; first cell in least significant byte |
| 488..494 | Brake strength, response, speed dependence (i16 x1000), speed curve (u8) |
| 496..507 | Reverse coupling, width, overrun, strength, response, hold (i16 x1000) |
| Slot 127 | CRC16 of slots 0..126 encoded as big-endian words |

That is **all 126 usable data slots**, with slot 127 holding the CRC.
The store is full: another setting would have to replace an existing one,
or cost map cells.

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
lands exactly on the new 10% grid), and the brake half is generated.
Brake map off, brake type regen-only.

Earlier development markers are rejected outright rather than migrated: their
geometry differs, and a wrong reading would be worse than falling back to
defaults.

Nothing is written back until the next explicit Save, which stores the
new format. The conversion is one-way: a stable-package install reading
back a 20260914 image rejects it as invalid and falls back to defaults
rather than misreading it.

This is a single-image store: a power cut during an update can invalidate
the previous save. It detects partial updates, but does not promise rollback.
EEPROM is shared with other Lisp packages and is not preserved across every
firmware update. Save only while stopped; motor output is paused until the
operation completes. Errors produce a failure status, never "saved".
