// Getting revolutions from encoders
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBench {
    private DcMotor frwheel;
    private DcMotor flwheel;
    private DcMotor brwheel;
    private DcMotor blwheel;

    // --- IMPORTANT: SET THIS TO YOUR MOTOR'S SPECIFIC VALUE ---
    public static final double TICKS_PER_REVOLUTION = 537.6; // Example value, e.g., for GoBILDA 5203 series

    public void init(HardwareMap hwMap) {
        frwheel = hwMap.get(DcMotor.class, "frwheel");
        if (frwheel != null) {
            frwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frwheel.setDirection(DcMotor.Direction.FORWARD); // Set direction if needed
        }

        flwheel = hwMap.get(DcMotor.class, "flwheel");
        if (flwheel != null) {
            flwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // flwheel.setDirection(DcMotor.Direction.REVERSE); // Set direction if needed
        }

        brwheel = hwMap.get(DcMotor.class, "brwheel");
        if (brwheel != null) {
            brwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // brwheel.setDirection(DcMotor.Direction.FORWARD); // Set direction if needed
        }

        blwheel = hwMap.get(DcMotor.class, "blwheel");
        if (blwheel != null) {
            blwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            blwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // blwheel.setDirection(DcMotor.Direction.REVERSE); // Set direction if needed
        }
    }

    // --- Methods to control motor power ---
    public void setAllWheelSpeeds(double speed) {
        if (frwheel != null) frwheel.setPower(speed);
        if (flwheel != null) flwheel.setPower(speed);
        if (brwheel != null) brwheel.setPower(speed);
        if (blwheel != null) blwheel.setPower(speed);
    }

    public void stopAllMotors() {
        setAllWheelSpeeds(0.0);
    }

    // --- Methods to GET ENCODER TICKS (raw data) ---
    public int getFrWheelTicks() {
        return (frwheel != null) ? frwheel.getCurrentPosition() : 0;
    }

    public int getFlWheelTicks() {
        return (flwheel != null) ? flwheel.getCurrentPosition() : 0;
    }

    public int getBrWheelTicks() {
        return (brwheel != null) ? brwheel.getCurrentPosition() : 0;
    }

    public int getBlWheelTicks() {
        return (blwheel != null) ? blwheel.getCurrentPosition() : 0;
    }

    // --- Methods to GET REVOLUTIONS (calculated) ---
    public double getFrWheelRevolutions() {
        return getFrWheelTicks() / TICKS_PER_REVOLUTION;
    }

    public double getFlWheelRevolutions() {
        return getFlWheelTicks() / TICKS_PER_REVOLUTION;
    }

    public double getBrWheelRevolutions() {
        return getBrWheelTicks() / TICKS_PER_REVOLUTION;
    }

    public double getBlWheelRevolutions() {
        return getBlWheelTicks() / TICKS_PER_REVOLUTION;
    }
}

