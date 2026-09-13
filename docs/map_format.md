# Map and EEPROM format

## Runtime map

The grid is deliberately **ragged**, because a rectangular one would spend
a quarter of the EEPROM on a region holding no independent information:
for traction, duty is speed and its sign carries nothing extra, so the
negative side is the positive side mirrored.

| Rows | Throttle | Columns | Duty |
| --- | --- | --- | --- |
| 0..9 | brake lever -100%..-10%, 10% steps | 21 | -100%..+100%, 10% steps |
| 10..30 | throttle 0..100%, 5% steps | 11 | 0..100%, 10% steps, read mirrored for negative duty |

`10*21 + 21*11 = 441` cells - the same budget a 21x21 grid would have
used, but the traction half keeps its 5% throttle steps and the brake half
gains a full reverse region. Each cell is a signed byte in -100..100,
representing -1.00..1.00 at 1% resolution. The buffer is mutable RAM,
never constant flash.

Row 10 is zero throttle. Zero lever is not stored: it is zero by
definition, and having it as an implicit row lets a light pull fade in
from nothing instead of jumping to the -10% row.

Every view draws 21 display columns per row; on a traction row the two
halves are the same cells, so editing one edits the other.

## Lookup

Two lookups, each bilinear on its own uniform grid; nothing ever blends
across the seam, since throttle and brake are separate inputs.

- **throttle >= 0**: rows 10..30 against `|duty|`, so rolling backwards
  reads the same curve as rolling forwards at that speed. Engine braking
  and the balance point therefore work in both directions, and nothing is
  discontinuous through zero.
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

Brake rows have their own generator and their own regenerate flag, so
neither half can overwrite the other:

```
peak    = brake_strength * lever ^ brake_response

duty > 0  (braking against speed)
    -(peak * ((1 - duty_dep) + duty_dep * duty ^ (1 + brake_curve)))

duty <= 0 (reverse)
    balance = clamp01(lever * rev_coupling)
    start   = clamp01(balance - rev_width)
    rev <= start    : -peak
    rev <= balance  : -peak * (1 - p^2)
    rev >  balance  : +rev_overrun * over^2
```

Defaults 1.0 / 1.0 / 0.0 / 0 and 1.0 / 0.10 / 0.12: -10% lever is -0.10 at
any forward speed, and backs up to 10% duty before it stops pulling.
`duty_dep` fades braking out towards a standstill; `rev_coupling = 0`
removes the reverse balance.

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

There are 128 persistent 32-bit slots. Format marker 20260918. The header
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
| 36..479 | Four map bytes per word; first cell in least significant byte |
| 480..492 | Brake strength, response, speed dependence (i16 x1000), speed curve (u8), then reverse coupling, width, overrun (i16 x1000) |
| Slot 127 | CRC16 of slots 0..126 encoded as big-endian words |

That is 124 of the 127 data slots; the remaining ones are written as zero.

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

Earlier **beta** markers are rejected outright rather than migrated: their
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
