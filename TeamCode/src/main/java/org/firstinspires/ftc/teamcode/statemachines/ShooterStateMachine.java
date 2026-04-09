package org.firstinspires.ftc.teamcode.statemachines;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants.ShotType;

/**
 * Non-blocking state machine for the shooter motor.
 *
 * States:
 *   OFF         → Motor stopped, no target.
 *   SPINNING_UP → Motor accelerating toward target velocity.
 *   READY       → Motor at target velocity, safe to feed balls.
 *
 * Usage:
 *   shooterSM.start(ShotType.LONG);   // Begins spin-up
 *   // ... in loop ...
 *   shooterSM.update();                // Call every cycle
 *   if (shooterSM.isReady()) { ... }   // Check before feeding
 *   shooterSM.stop();                  // Shuts down
 */
public class ShooterStateMachine {

    public enum State {
        OFF,
        SPINNING_UP,
        READY
    }

    private final DcMotorEx shooter;
    private State state = State.OFF;
    private ShotType currentShotType = ShotType.NONE;

    public ShooterStateMachine(DcMotorEx shooter) {
        this.shooter = shooter;
    }

    /**
     * Begin spinning up the shooter for the given shot type.
     * Configures PIDF coefficients and starts the motor.
     */
    public void start(ShotType type) {
        if (type == ShotType.NONE) {
            stop();
            return;
        }
        // Only reconfigure PIDF when the shot type actually changes
        if (type != currentShotType) {
            currentShotType = type;
            shooter.setVelocityPIDFCoefficients(type.p, type.i, type.d, type.f);
        }
        if (state == State.OFF) {
            state = State.SPINNING_UP;
        }
    }

    /** Stop the shooter completely. */
    public void stop() {
        state = State.OFF;
        currentShotType = ShotType.NONE;
        shooter.setVelocity(0);
    }

    /**
     * Call once per loop cycle. Drives the motor and manages state transitions.
     * This method never blocks.
     */
    public void update() {
        switch (state) {
            case OFF:
                // Motor is off — do nothing
                break;

            case SPINNING_UP:
                shooter.setVelocity(currentShotType.tps);
                if (Math.abs(shooter.getVelocity() - currentShotType.tps)
                        < ShooterConstants.VELOCITY_TOLERANCE) {
                    state = State.READY;
                }
                break;

            case READY:
                // Maintain target velocity
                shooter.setVelocity(currentShotType.tps);
                // If velocity drifts too far, go back to spinning up
                if (Math.abs(shooter.getVelocity() - currentShotType.tps)
                        > ShooterConstants.VELOCITY_TOLERANCE * 2) {
                    state = State.SPINNING_UP;
                }
                break;
        }
    }

    // --- Getters ---

    public State    getState()           { return state; }
    public boolean  isReady()            { return state == State.READY; }
    public boolean  isOff()              { return state == State.OFF; }
    public boolean  isRunning()          { return state != State.OFF; }
    public ShotType getCurrentShotType() { return currentShotType; }
    public double   getCurrentVelocity() { return shooter.getVelocity(); }

    public double getTargetVelocity() {
        return currentShotType != null ? currentShotType.tps : 0;
    }

    public double getError() {
        return getTargetVelocity() - getCurrentVelocity();
    }
}
