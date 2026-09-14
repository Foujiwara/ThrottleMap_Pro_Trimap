# Architecture

ThrottleMap Pro runs three LispBM processes: the input/control loop (nominal 200 Hz,
150-word stack), telemetry (20 Hz, 80 words), and the serial command handler
(256 words). Actual frequency depends on the firmware scheduler and GC.

The entry point imports five Lisp sources and evaluates them incrementally.
Immutable functions and constants live in flash via `@const-start`.
Configuration globals and byte buffers are defined outside these blocks.
Moving the mutable map into a constant block makes writes fail; separate
source files do not require private copies of shared arithmetic helpers.

The 441-cell map occupies 441 RAM bytes. Control calculations use signed
28-bit fixnums scaled by 1000. This reduces boxed numeric allocations; local
environments and function calls still consume heap cells. The ADC voltage
is sampled once per tick in bidirectional mode and converted to millivolts.
RPM is read only by telemetry. The EMA retains a fractional accumulator so
small deltas cannot leave a permanent positive throttle after release.

Each sender owns a reusable packet buffer: live 15 bytes, row 44 bytes,
configuration 27 bytes, status 3 bytes. EEPROM operations temporarily allocate
a 508-byte image, reclaimed by GC. Buffers must not be shared between the
telemetry and command processes.

Positive map output uses `set-current-rel`; negative output uses
`set-brake-rel` with a positive magnitude. Signed current alone would request
reverse torque rather than direction-independent braking. Native firmware
limits remain active. There is no extra `timeout-reset` call because both
motor commands already reset the firmware timeout.

Sources: ADC, PPM, UART, and USB Test. For ADC modes, keep App to Use set to
ADC and its control type Off: VESC Tool then supplies calibrated ADC1/ADC2
values without commanding the motor. The package does not duplicate ADC
Start/End calibration. Bidirectional mode uses VESC Tool's Start/End plus a
package-owned neutral voltage, defaulting to 1.650 V; the UI shows live ADC1
voltage and CALIBRER captures the present neutral point. That point is stored
in the package EEPROM.

UART accepts A5 / percent (0..200) / XOR checksum, preserving partial frames.
UART, Test and PPM input expires after 500 ms without a valid update; expired
input clears the filter and requests zero current. The UI refreshes an active
bench command every 100 ms; STOP clears both the bench value and filter.

Commands validate length, indices and all fields before mutation. Map generation,
configuration replacement and EEPROM operations pause motor output while they
run. A recoverable command error returns an explicit failure status; a fatal
interpreter error still requires restarting the script.

QML sends one queued command at a time and waits for its correlated status.
This prevents a 31-row upload overflowing the receiver mailbox. Bench commands
bypass this queue so STOP remains responsive; their acknowledgements cannot
complete another command. Full-map reads verify all 31 rows arrived (counted
in an array, not a bit mask - 31 bits would overflow a JS bitwise integer).

## Brake half and brake type

The map's vertical axis is signed. The brake half is only read when the
brake map is enabled and the brake lever is above its deadband, so the
default behaviour is byte-for-byte the stable package's.

Brake type applies to the brake lever alone. Type 0 is regen and calls
`set-brake-rel`. Type 1 calls only `set-current-rel`: it applies negative
current while moving forward and zero current once stopped so it cannot
reverse. Type 2 calls only `set-current-rel` and keeps negative current
through zero into reverse. Released-throttle engine braking and overrun regen
remain separate map behaviours and continue to use regen.

See [protocol](protocol.md), [storage layout](map_format.md),
[audit](audit-2026-09-12.md) and [tests](../tests/README.md).
