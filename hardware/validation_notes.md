# Hardware starter: software validation record

Checked **12 September 2026**, against the completed Step 7 baseline `bb043f04dae480672da95980d6d27eb50152b36f`. This is a build guide and starter implementation. **No physical board was connected, no firmware was uploaded, and no motor was moved during these checks.**

## Python and frozen ANN

The existing Python 3.11.5 ANN environment was reused, with TensorFlow 2.16.1 and the pinned `requirements/requirements-ann.txt` stack. The only added runtime dependency is **pySerial 3.5**, declared in `requirements/requirements-hardware.txt`.

```sh
python -m unittest discover -s python -v
python python/python_serial_control.py --target 10 10
```

**33 tests passed**: 7 existing pickup tests, 14 existing planner tests, and 12 new hardware-bridge tests. The new tests use the original notebook robot class and a fake serial transport. They cover input/workspace rejection before inference, invalid ANN results, robot and hardware limits without clipping, nearby configuration jumps, FK error at the exact transmitted precision, controller-profile parsing, refusal to arm unverified calibration, acknowledgement/completion IDs, reset/error/partial replies, missing replies, heartbeat failure and incomplete writes. A failed reply leaves the last confirmed commanded state unchanged; session cleanup requests STOP.

The offline command loaded the **real frozen ANN and saved scalers** through `python/robot_pipeline.py` and reported:

| Quantity | Result |
|---|---|
| Requested XY | (10, 10) cm |
| Protocol-rounded ANN angles | (0.506°, 90.176°) |
| Original FK endpoint at those angles | (9.88058139, 10.08760410) cm |
| Cartesian ANN error | 0.148 cm |
| Serial connection | None opened |

This result checks the software prediction and geometry, not a measured robot position. Offline mode does not know the actual calibrated motor range.

## Arduino compilation

The sketches were compiled for **Arduino Uno R3 / `arduino:avr:uno`**, using Arduino CLI **1.5.1**, Arduino AVR Boards **1.8.8**, Servo **1.3.0**, and optional hd44780 **1.3.2**. Toolchain downloads/build files were kept outside the repository. No upload command was run.

| Sketch / build variant | Flash, of 32,256 bytes | Static SRAM, of 2,048 bytes |
|---|---:|---:|
| `servo_controller`: shipped calibration lock, LCD off | 8,958 (27%) | 386 (18%) |
| `servo_controller`: temporary copy, calibration enabled and LCD on | 17,974 (55%) | 805 (39%) |
| `servo_neutral_test` | 3,232 (10%) | 231 (11%) |

The enabled test copy checks compilation of the motion and LCD paths, which the compiler can eliminate when calibration is a constant false. It is **not** evidence of a calibrated arm. The committed sketch retains `CALIBRATION_VERIFIED = false` and `ENABLE_LCD = 0`. SRAM figures are static allocation; runtime stack and library behavior still require bench testing.

For reproducible default builds from the repository root:

```sh
arduino-cli core update-index
arduino-cli core install arduino:avr@1.8.8
arduino-cli lib install Servo@1.3.0
arduino-cli lib install hd44780@1.3.2
arduino-cli compile --fqbn arduino:avr:uno arduino/servo_controller
arduino-cli compile --fqbn arduino:avr:uno arduino/servo_neutral_test
```

To reproduce the enabled/LCD compilation, use a separate temporary sketch folder with the same sketch filename and change only those two flags in that copy. Do not upload an unmeasured example calibration. The fake serial tests do not execute the AVR firmware; compilation does not establish parser, watchdog, timing or physical-stop behavior on a real board.

## Preservation and visual review

SHA256 comparisons against the pre-hardware snapshot confirmed that every pre-existing file except the intentionally updated README is unchanged, including all notebooks, datasets, ANN/scalers, evaluation reports and simulation code/artifacts. The bridge calls the existing robot pipeline and workspace/validation helpers; it contains no replacement FK or IK implementation.

The architecture, wiring and mechanical PNGs were visually inspected. Editable SVG copies accompany them. The mechanical top view uses coordinates returned by the original `RobotArm2DOF`; the side concept is schematic, not a fabrication drawing. Local Markdown links and the final Git diff were checked.

## Required physical evidence

Follow the [calibration checklist](calibration_checklist.md) and [first-test procedure](../hardware_2d_robot_plan.md#9-first-physical-test-plan). Actual servo variant/travel, pulse direction and offset, dimensions, current/voltage under motion, bracket clearance, stopping behavior, backlash, heating, repeatability and Cartesian accuracy remain unmeasured. `DONE` and LCD angles are commanded values, never encoder feedback.

The 2D project stops at this guide/starter-code milestone. Physical construction and a separate 3D arm project are future work.
