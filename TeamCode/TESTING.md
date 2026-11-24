# Our first robot unit tests

These tests check calculations used by the `Drivetrain` subsystem and the camera. They run on a
computer without a Control Hub, Limelight, motors, or Android emulator.

From the robot repository, with the Android/JDK toolchain installed:

```sh
./gradlew :TeamCode:testDebugUnitTest
```

There are 17 tests: 12 drive-math cases and 5 camera-distance cases.

The test source lives in `src/test/java/`, separate from the OpModes under
`src/main/java/.../testing`. Those OpModes still need a robot and are not unit
tests. JUnit is a test-only dependency and is not packaged into the robot app.

## What we check

| Calculation | Examples |
| --- | --- |
| Mecanum wheel mixing | Stop, forward/backward, left/right strafe, rotation, diagonals |
| Power normalization | Preserve wheel ratios and keep every power between -1 and 1 |
| Camera distance | Meter-to-inch conversion, known triangles, negative coordinates, all three axes |

`DriveMath` and `VisionMath` contain the actual calculations used by
`Subsystems.Drivetrain` and `mechanisms.Camera`; the tests do not use a separate copy of the formulas.
Their extraction preserves the existing formulas, conversion factor, and tuning.

## Add a useful test

1. Name the situation, such as `releasedSticksStopAllFourWheels`.
2. Choose inputs with an expected answer you can explain on paper.
3. Call the real calculation and compare its answer to that expectation.
4. Run the test task. A failed assertion should help explain what changed.

For a bug fix, add a case that fails before the fix and passes afterward.
Do not change a test's expected answer just to make the test green.

These tests do not verify motor wiring/directions on the real robot, camera
target validity, aiming windows, mechanism timing, autonomous movement deadlines,
or game-rule compliance. Those need additional tests and supervised robot checks.
Existing aiming and hardware-setup concerns are not fixed by this first suite.

Android Studio can also run a test from the green arrow beside its method or
class. Full results are in `build/reports/tests/testDebugUnitTest/index.html`.
