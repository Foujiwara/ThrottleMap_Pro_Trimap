# Regression tests

Three checks, in increasing order of cost.

| What | Needs | State |
|---|---|---|
| `check-structure.py` | Python | runs in every build |
| `regression.lisp` | WSL, gcc `-m32`, make, git | **passing** |
| `ui.test.mjs` | Node.js | updated, not yet executed |

## Structure check — runs on every build

```sh
python tests/check-structure.py
```

Parses the lisp sources and fails on any `let`, `lambda`, `defun` or loop
form carrying more than one body form. LispBM evaluates only the first; the
rest is dropped silently, and the parentheses balance perfectly either way.
That is exactly how the position lock came to publish a current request it
never sent to the motor. Both `build.sh` and `build.ps1` run this before
packaging, so a build cannot ship that class of fault again.

## Lisp regression suite

The application runs in the real LispBM interpreter. Only the hardware
boundary is mocked — sensors, motor output, the custom transport, EEPROM and
time (`vesc-mocks.lisp`). The program is loaded with its flash directives
intact, so the const regions are exercised the way they are on hardware.
Module bodies are concatenated for the harness; the `.vescpkg` uses real
imported modules. `package.lisp` is cut at its `; ---- boot ----` marker,
below which everything spawns threads or touches hardware.

Coverage: the ragged index function (every cell reachable, none shared),
map cold reload and every quantized byte value, CRC corruption, missing
slots, silently failed writes, injected write errors, unchanged-save write
count, rejection of foreign image formats, lookup mirroring and
interpolation, throttle filtering, inversion, deadband, bench and PPM
watchdogs, fragmented UART frames, brake routing in all three brake types,
malformed commands, packet range validation, boot-time packet refusal, and
the position lock end to end — that it commands the motor at all, that it is
symmetric, proportional, damped, that dead travel is genuinely free, and that
every release path records why. Two 10,000-tick workers run on the production
150-word control stack, one with the lock engaged.

Tested against 32-bit LispBM commit
`48f9259cb5aa6315ca37aa80fa814914c3fac1f1`. Needs a C compiler with 32-bit
libc (Ubuntu: `gcc-multilib`), make and git. No VESC connection is used.

```sh
git clone https://github.com/svenssonjoel/lispBM.git .test-runtime/lispBM
git -C .test-runtime/lispBM checkout 48f9259cb5aa6315ca37aa80fa814914c3fac1f1
make -C .test-runtime/lispBM/tests test_lisp_code_cps_time
python tests/prepare-lisp.py
.test-runtime/lispBM/tests/test_lisp_code_cps_time -i -h 2048 -t 180 .test-runtime/regression.lisp
```

On Windows, run the clone, make and runner lines inside WSL; `prepare-lisp.py`
runs on either side.

The upstream runner prints `SUCCESS` on success. An "injected-write-error"
trapped diagnostic is expected — it tests cleanup after an EEPROM failure.
The closing memory report includes the mocks and test globals and is not a
measurement of the package on a physical controller.

## Interface tests

```sh
node tests/ui.test.mjs
```

Extracts the JavaScript from `ui.qml.in` and runs the real functions with
only Qt and the transport mocked. It covers the receive path and the command
queue — where every interface fault in this project has actually been —
including the three found in the 0.1.10-0.1.14 range: the unthrottled
read-back retry that saturated the link, the 20-second timeout on a dropped
read-back, and the boot guard that returned above the decode which sets the
value it tests. It does not emulate Qt rendering.

This half has **not been executed** since it was rewritten: Node.js is not
installed here. Treat it as reviewed, not as passing.

## Building the package

`./build.ps1 -VescTool 'C:/path/to/vesc_tool.exe'` on Windows, or
`./build.sh /path/to/vesc_tool` on Linux. Never call
`vesc_tool --buildPkgFromDesc` directly: `pkgdesc.qml` points at the
*generated* `ui.qml` and `package_README-gen.md`.
