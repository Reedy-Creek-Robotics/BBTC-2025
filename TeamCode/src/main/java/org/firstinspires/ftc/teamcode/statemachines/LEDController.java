package org.firstinspires.ftc.teamcode.statemachines;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Non-blocking state machine for the LED indicator.
 *
 * States:
 *   IDLE           → LED off.
 *   TARGET_LOCKED  → LED shows solid green (target acquired).
 *   CYCLING        → LED cycles through colors on a timer.
 *
 * Extracted from TeleOpRed/TeleOpBlue to eliminate duplicated LED logic.
 */
public class LEDController {

    public enum State {
        IDLE,
        TARGET_LOCKED,
        CYCLING
    }

    private final Servo led;
    private State state = State.CYCLING;
    private long lastToggleTime;
    private double ledColor = 0;

    // Timing: 2 seconds of color, then 3 seconds off, then repeat
    private static final long CYCLE_ON_DURATION_MS  = 2000;
    private static final long CYCLE_TOTAL_MS        = 5000;
    private static final double TARGET_LOCKED_COLOR = 0.666;

    public LEDController(Servo led) {
        this.led = led;
        this.lastToggleTime = System.currentTimeMillis();
    }

    /**
     * Call this each loop with the result of your target-lock check.
     * Automatically switches between TARGET_LOCKED and CYCLING states.
     */
    public void setTargetLocked(boolean locked) {
        if (locked) {
            state = State.TARGET_LOCKED;
        } else if (state == State.TARGET_LOCKED) {
            // Just lost lock — start cycling again
            state = State.CYCLING;
            lastToggleTime = System.currentTimeMillis();
        }
    }

    /** Force LED off. */
    public void setIdle() {
        state = State.IDLE;
        led.setPosition(0);
    }

    /**
     * Call once per loop cycle. Updates the LED based on current state.
     * Never blocks.
     */
    public void update() {
        long now = System.currentTimeMillis();

        switch (state) {
            case IDLE:
                led.setPosition(0);
                break;

            case TARGET_LOCKED:
                led.setPosition(TARGET_LOCKED_COLOR);
                lastToggleTime = now; // Keep timer fresh so cycling starts clean
                break;

            case CYCLING:
                long elapsed = now - lastToggleTime;

                if (elapsed >= CYCLE_TOTAL_MS) {
                    // Reset the cycle
                    lastToggleTime = now;
                    elapsed = 0;
                }

                if (elapsed >= CYCLE_ON_DURATION_MS) {
                    // Off phase
                    led.setPosition(0);
                } else {
                    // On phase — advance color at the start of each cycle
                    if (elapsed < 30) {
                        ledColor += 0.1;
                        if (ledColor > 1.0) ledColor = 0.1;
                        // Skip the green range (reserved for target lock)
                        if (ledColor > 0.55 && ledColor < 0.69) ledColor += 0.2;
                    }
                    led.setPosition(ledColor);
                }
                break;
        }
    }

    // --- Getters ---

    public State getState() { return state; }
}
