package org.firstinspires.ftc.teamcode.statemachines;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

/**
 * Non-blocking state machine for the intake system (transfer belt + intake servo).
 *
 * States:
 *   OFF     → Everything stopped.
 *   RUNNING → Transfer belt pulls balls in (normal intake).
 *   REVERSE → Both belt and servo reverse to eject jams.
 *   FEEDING → Both belt and servo push balls into the shooter at full power.
 *
 * Usage:
 *   intakeSM.toggle(State.RUNNING);  // Toggle intake on/off
 *   intakeSM.setState(State.FEEDING); // Force into feeding mode
 *   intakeSM.update();                // Call every loop cycle
 */
public class IntakeStateMachine {

    public enum State {
        OFF,
        RUNNING,
        REVERSE,
        FEEDING
    }

    private final DcMotor intakeTransfer;
    private final CRServo intakeServo;
    private State state = State.OFF;

    public IntakeStateMachine(DcMotor intakeTransfer, CRServo intakeServo) {
        this.intakeTransfer = intakeTransfer;
        this.intakeServo    = intakeServo;
    }

    /** Force a specific state. */
    public void setState(State newState) {
        this.state = newState;
    }

    /** Toggle between the given state and OFF. */
    public void toggle(State targetState) {
        state = (state == targetState) ? State.OFF : targetState;
    }

    /** Force stop. */
    public void stop() {
        state = State.OFF;
    }

    /**
     * Call once per loop cycle. Applies the correct motor/servo powers
     * for the current state. Never blocks.
     */
    public void update() {
        switch (state) {
            case OFF:
                intakeTransfer.setPower(0);
                intakeServo.setPower(0);
                break;

            case RUNNING:
                intakeTransfer.setPower(IntakeConstants.INTAKE_TRANSFER_POWER);
                intakeServo.setPower(IntakeConstants.INTAKE_SERVO_POWER);
                break;

            case REVERSE:
                intakeTransfer.setPower(IntakeConstants.REVERSE_TRANSFER_POWER);
                intakeServo.setPower(IntakeConstants.REVERSE_SERVO_POWER);
                break;

            case FEEDING:
                intakeTransfer.setPower(IntakeConstants.FEED_TRANSFER_POWER);
                intakeServo.setPower(IntakeConstants.FEED_SERVO_POWER);
                break;
        }
    }

    // --- Getters ---

    public State   getState()   { return state; }
    public boolean isOff()      { return state == State.OFF; }
    public boolean isFeeding()  { return state == State.FEEDING; }
    public boolean isRunning()  { return state == State.RUNNING; }
}
