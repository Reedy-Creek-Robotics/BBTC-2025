package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class motorRpm {

    private DcMotorEx motor1;
    private Telemetry telemetry;

    // This value needs to be set to the ticks per revolution for your specific motor.
    // For example, a REV HD Hex Motor has 28 ticks/rev, a NeverRest 20 has 560.
    // Check your motor's datasheet.
    private static final double TICKS_PER_REVOLUTION = 28; // Example for a REV HD Hex Motor

    /**
     * Initializes the motor.
     * @param hwMap the hardware map from the OpMode.
     * @param telemetry the telemetry object from the OpMode.
     */
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        // The name "motor1" must match the name in your robot's configuration file.
        motor1 = hwMap.get(DcMotorEx.class, "motor1");
        this.telemetry = telemetry;
    }

    /**
     * Sets the power for the motor.
     * @param power The power to set, from -1.0 to 1.0.
     */
    public void setPower(double power) {
        motor1.setPower(power);
    }

    /**
     * Gets the current speed of the motor in Revolutions Per Minute (RPM).
     * @return The current RPM of the motor.
     */
    public double getRpm() {
        // getVelocity() returns ticks per second.
        // To convert ticks/sec to revolutions/minute:
        // (ticks/sec) * (60 sec/min) / (ticks/rev) = rev/min
        double velocity = motor1.getVelocity();
        return (velocity * 60) / TICKS_PER_REVOLUTION;
    }

    /**
     * Displays the current RPM of the motor on the driver station telemetry.
     */
    public void printRpm() {
        telemetry.addData("Motor 1 RPM", "%.2f", getRpm());
        telemetry.update();
    }
}
