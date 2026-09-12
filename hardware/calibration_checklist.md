# Calibration and first-movement checklist

Use with the [build guide](../hardware_2d_robot_plan.md). This sheet records **your actual build**; the repository contains no measured motor calibration. Initially keep horns/links off the motors and test only one servo at a time.

## A. Identify and inspect

- [ ] Both labels/order specifications confirm **DS3218 270° positional**, not continuous rotation.
- [ ] Adapter is the selected **5 V / 6 A** unit; voltage and polarity checked with a multimeter.
- [ ] External motor power, common ground, fuse, switch and insulated connections match the wiring diagram.
- [ ] Physical cutoff is reachable outside the arm's sweep.
- [ ] Each shaft is supported by suitable aligned hardware; nothing binds or rubs.
- [ ] Shoulder-to-elbow distance is **100 mm**; elbow-to-pointer XY distance is **100 mm**. Measured between rotation centers, not link ends or servo cases.
- [ ] Motor axes are parallel and vertical; the first build moves horizontally over the grid.
- [ ] Origin and +X/+Y directions on the paper match the model. Verify print scale using a ruler.

If link lengths are wrong, adjust the mechanics. Changing the FK link lengths alone would not fix the frozen ANN. Record all pointer offsets; the first pointer should have no horizontal offset from the modeled endpoint.

## B. Bench-test one motor

1. Disconnect servo power and remove its link/horn. Connect only this motor, signal D9, to the external supply/common-ground arrangement.
2. Upload `arduino/servo_neutral_test/servo_neutral_test.ino` with motor power off. Open Serial Monitor at 115200. Power the motor; it should remain without commanded pulses until `N`.
3. Send `N`: nominal pulse 1500 µs. Send `+` or `-` for individual 20 µs changes. The initial test window is only 1100–1900 µs. Send `X` to detach; cut power before modifying mechanics.
4. Fit a light temporary indicator and measure output angle with a protractor. Record the angle at 1500 µs and at several nearby pulses, looking from the same top-view direction used by the model.
5. Confirm motion direction. Do not assume the datasheet's view direction or another motor's horn mounting determines the sign in your robot.
6. Gradually test the pulse range required for your intended limits **without driving into hard stops**. Only expand `MIN_TEST_US`/`MAX_TEST_US` after checking the exact motor specification and clearance. The final example elbow range requires pulses beyond the initial bench window. Never exceed the controller's 544–2400 µs supported envelope in this starter.
7. Repeat with the other motor. Treat each motor's calibration independently, even when the model names match.

The neutral-test sketch intentionally holds its last pulse until `X` or power removal. It is for a supervised loose-servo bench test, not the assembled-arm safety controller.

## C. Fit the angle-to-pulse map

For two measured robot-relative angles `q_a`, `q_b` at pulses `p_a`, `p_b`:

```text
signed_us_per_degree = (p_b - p_a) / (q_b - q_a)
neutral_us = p_a - signed_us_per_degree × (q_a - neutral_robot_angle)

pulse(q) = neutral_us + signed_us_per_degree × (q - neutral_robot_angle)
```

Use a meaningful angular separation and verify at a third point. If the linear fit is inaccurate across a broad range, first check measurement, play and mounting; narrow the usable range rather than pretending one slope is exact everywhere.

| Calibration item | Shoulder | Elbow |
|---|---|---|
| Exact motor / ordered variant | | |
| Robot angle observed at 1500 µs | | |
| Pulse/angle measurement A | | |
| Pulse/angle measurement B | | |
| Third-point validation error | | |
| Signed µs/degree | | |
| Safe robot-angle minimum / maximum | | |
| Safe pulse minimum / maximum | | |
| Supply voltage under motion | | |

The example firmware has `NEUTRAL_ANGLE = {45, 0}`, `NEUTRAL_US = {1500, 1500}` and positive `US_PER_DEG ≈ 7.407`. That assumes shoulder neutral is mounted at +45° and elbow neutral makes the links straight. You may instead record the actual mounting angles and adjust the constants. **Robot zero is (0°,0°), with both links along +X; servo neutral does not automatically equal robot zero.**

Mount the elbow motor on Link 1 and measure elbow angle **relative to Link 1**. A change in shoulder angle must not be mistaken for a change in elbow zero.

## D. Set safe limits and assemble

- [ ] `NEUTRAL_ANGLE`, `NEUTRAL_US` and signed `US_PER_DEG` match the measurements.
- [ ] `LOWER`/`UPPER` remain inside the model's joint ranges and the measured mechanical range.
- [ ] Initial assembled limits are no wider than the example **q1 0…120°, q2 0…110°**, and narrower if necessary.
- [ ] `MIN_US`/`MAX_US` cover only verified pulses and stay inside 544–2400 µs. Example values 800/2350 are limits, not instructions to blindly visit those pulses.
- [ ] Both endpoint pulses and the HOME pulse lie within the verified range.
- [ ] HOME `(90°,0°)` is physically clear; cables remain loose throughout every intended movement.
- [ ] Loose-servo pulse/direction tests are complete; with power off, HOME and the intended small initial sweep are mechanically clear.
- [ ] Only after these checks, set `CALIBRATION_VERIFIED = true` and upload the controller. The first powered assembled-arm checks follow in section E; expand the tested motion gradually.

The pulse conversion is affine, so interpolating between two pulse-valid joint angles stays inside their pulse interval. This does **not** prove bracket clearance, exact servo tracking or safe load throughout the motion; inspect the whole swept area.

## E. Establish position at every session

1. Servo power off; controller outputs detached. Align the physical links with HOME `(90°,0°)` using the calibrated marks. Do not force a gearbox that resists back-driving; loosen/reseat the horn correctly if needed, then recheck calibration and screw retention.
2. Keep hands clear and turn on external motor power. Start the Python bridge, confirm the reported profile and type `ARM` only when alignment is correct.
3. Observe the first holding command. Unexpected movement means stop and inspect; no software interpolation can guarantee a gentle startup from an unknown physical angle without feedback.
4. Use small direct calibration commands, then the known poses below. Expand the tested area gradually.
5. On stop, USB reset/disconnection, lost power or unknown movement, repeat alignment. Do not trust the previous software setpoint as a measured physical position.

## F. Known poses, ANN targets and measured error

These expected coordinates follow the repository's existing FK:

| Robot angles (degrees) | Expected endpoint (cm) | Measured endpoint | Notes |
|---|---|---|---|
| (90, 0) | (0, 20) | | HOME |
| (0, 0) | (20, 0) | | Straight along +X |
| (0, 90) | (10, 10) | | Elbow relative to Link 1 |
| (45, 90) | (0, 14.142) | | Two-link geometry check |

In hardware mode, `angles a b` deliberately bypasses ANN **for calibration**, but still checks joint/hardware limits and the nearby-jump guard. For normal reaching, enter only `x y`; the saved ANN supplies the angles. Validate simple poses before evaluating ANN performance on the physical arm.

| Trial | XY target | Predicted angles | FK endpoint | Measured tip XY | ANN error | Physical tracking error | Total target error |
|---|---|---|---|---|---|---|---|
| 1 | (10, 10) | | | | | | |
| 2 | (0, 15) | | | | | | |
| 3 | | | | | | | |
| 4 | | | | | | | |
| 5 | | | | | | | |

- ANN error: Euclidean distance from FK-predicted tip to target.
- Physical tracking error: distance from measured tip to FK-predicted tip.
- Total error: distance from measured tip to target. Do not add the two scalar errors and assume that equals total error.
- Repeat an accepted target five times from both approach directions. Record spread, backlash, heat and voltage sag.
- A practical first debugging target is total error around 1 cm or less with repeatability spread around 0.5 cm or less; these are provisional goals, not achieved or guaranteed specifications.

## G. Stop checks before regular use

- [ ] D2 button stops advancing motion and disables pulses; release does not restart it.
- [ ] PC Ctrl+C/quit requests STOP; the pointer's final physical position is inspected.
- [ ] Loss of USB/heartbeats during an unloaded test disables pulses after the timeout.
- [ ] The independent servo-power switch really removes motor power.
- [ ] Actual behavior after pulse loss is known for these motors; detach is not assumed to be a brake.
- [ ] No full-arm test uses a gripper, payload or unattended motion.

**Builder/date:** ____________________   **Reviewed by:** ____________________
