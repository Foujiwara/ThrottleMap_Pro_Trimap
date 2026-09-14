# RFP-Performance — ThrottleMap Pro Trimap

A [VESC Package](https://github.com/vedderb/vesc_pkg) (LispBM + QML) that
replaces plain throttle-to-current control with a configurable
**Throttle x Duty -> Current Relative** map, tuned by default to feel like
a thermal-engine powertrain instead of a flat electric current curve.

*Trimap* because the map is three independently tunable regions sharing
one grid: **traction**, **braking against speed**, and **reverse**.

- `+1.0` current-rel = 100% of the VESC's configured positive motor current
- `0.0` = freewheel
- negative = regen / motor braking, `-1.0` = 100% of configured negative current

The map is independent of the actual amps configured on the VESC, and the
motor commands use `set-current-rel` for propulsion and `set-brake-rel`
for braking. Every native VESC protection (current/ERPM/duty/temperature/
voltage limits) stays fully in control.

![license](https://img.shields.io/badge/license-GPLv3-blue)

## Features

- Throttle source: ADC, PPM or UART, with min/max/deadband/invert/filter
  calibration and a clean seam for adding more sources later.
- 451-cell ragged Throttle x Duty grid: throttle rows at 5% steps over
  0..100% duty, brake rows at 10% steps over **-100%..+100%** duty.
  Bilinear interpolation, ~200 Hz control loop, entirely in LispBM
  (see [docs/architecture.md](docs/architecture.md)).
- **Three generators**, one per region of the graph, each with its own
  regenerate flag so shaping one never discards hand edits made to
  another: traction (4 presets plus 8 free parameters), braking against
  speed, and reverse (the traction law mirrored, with its own 8
  parameters) - see [docs/map_format.md](docs/map_format.md).
- Three brake types: regen only, current without reverse, or fully
  bidirectional with a configurable reverse balance speed.
- QML UI: live heatmap with a real-time throttle/duty dot, 2D cross-section
  curves per throttle row, a pseudo-3D isometric view, manual cell/row/
  column editing (bump %, smooth, copy/paste, interpolate a selection),
  and text-based export/import.
- Persisted on the VESC itself (emulated eeprom), survives VESC Tool
  disconnects and controller reboots - see [docs/map_format.md](docs/map_format.md#eeprom-layout).
- A compact, package-private binary protocol between QML and LispBM - see
  [docs/protocol.md](docs/protocol.md).

## Repository layout

```
pkgdesc.qml, ui.qml.in, package_README.md, Makefile, version, package_name
lisp/        package.lisp, util.lisp, map.lisp, throttle.lisp,
             storage.lisp, protocol.lisp
docs/        architecture.md, protocol.md, map_format.md
```

See [docs/architecture.md](docs/architecture.md) for the full breakdown.

## Building the `.vescpkg`

Windows: `./build.ps1 -VescTool 'C:/path/to/vesc_tool.exe'`.

See [tests/README.md](tests/README.md) for the regression suite. **It has
not been updated for this package's map geometry** and is pending work,
not passing coverage.

This repo follows the same build convention as the official packages in
[vedderb/vesc_pkg](https://github.com/vedderb/vesc_pkg) (e.g. `refloat/`):
the actual `.vescpkg` is produced by **VESC Tool itself**, via its
command-line packaging mode.

```bash
make                              # -> throttlemap_pro_trimap.vescpkg
make VESC_TOOL=/path/to/vesc_tool # if vesc_tool isn't on PATH
```

This runs `vesc_tool --buildPkgFromDesc pkgdesc.qml` (desktop VESC Tool
6.05+). If your VESC Tool build only exposes the GUI **Pack** button and
not a working `--buildPkgFromDesc` flag, you can pack manually instead:
open VESC Tool -> Developer/Custom Config page -> point it at `ui.qml`
and `lisp/package.lisp` from this repo -> **Pack** -> save as
`throttlemap_pro_trimap.vescpkg`.

## Installing

1. VESC Tool -> **VESC Packages** -> install the built/downloaded
   `throttlemap_pro_trimap.vescpkg` onto your VESC.
2. In **App Settings**, keep the app for the input you intend to use
   (ADC / PPM / UART) **enabled/active** - just change its **Control
   Type** dropdown to **Off** (`ADC_CTRL_TYPE_NONE` / equivalent). This
   is two different settings: disabling the app entirely also stops it
   decoding the signal at all, so this package would see no input
   either; leaving Control Type on anything other than Off means the
   app *and* this package both try to drive the motor from the same
   signal at once, which shows up as jerky/stuttering acceleration
   (confirmed on real hardware - see the LIVE feedback: if `Throttle`
   never moves, the app is decoding nothing; if the motor stutters
   in bursts, the app's own control type is still active). This is the
   one existing setting this package expects you to change; it does
   not touch anything else in your motor/app config.
3. Open the new **ThrottleMap Pro** tab, pick a throttle source +
   calibration, pick a preset (or tune the three generators by hand),
   regenerate each region you changed, then **Save to VESC**.

## Status / testing

Built and reviewed against the real LispBM extension set (`vedderb/bldc`
lispBM docs) and the real VESC Package conventions (`vedderb/vesc_pkg`,
notably `refloat/`), but **not yet run on physical hardware** - there is
no VESC connected to this development environment. Before riding:

- Bench-test with the wheel off the ground first, confirm the live dot on
  the Map tab tracks your throttle/duty as expected, and confirm the
  default Thermal Street map's engine-braking feel at 0% throttle before
  trusting it at speed.
- If you enable the brake map, check the reverse region deliberately: with
  a bidirectional brake type, a released throttle while rolling backwards
  commands forward torque to hold the vehicle (12% by default). Set the
  reverse runaway hold to 0 to freewheel there instead.
- Start with conservative motor current limits in the standard VESC motor
  config regardless of this package's map, since this package only ever
  requests a *fraction* of whatever those limits allow.

## Versioning

`major.release.build`. Only the last number moves between builds; the
middle one is bumped for a release.

## License

GPL-3.0-or-later, matching the license used by the official VESC Packages
repository. See [LICENSE](LICENSE).

---

*Conçu par RFP-Performance.*
