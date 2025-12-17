package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor RPM Testing")
public class motorRpm extends LinearOpMode {

    private DcMotorEx flmotor;
    private DcMotorEx frmotor;
    private DcMotorEx blmotor;
    private DcMotorEx brmotor;
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    //private DcMotorEx intake;

    // This value needs to be set to the ticks per revolution for your specific motor.
    // For example, a REV HD Hex Motor has 28 ticks/rev, a NeverRest 20 has 560.
    // Check your motor's datasheet.
    private static final double TICKS_PER_REVOLUTION = 28; // Example for a REV HD Hex Motor

    @Override
    public void runOpMode() {
        // The names must match the names in your robot's configuration file.
        flmotor = hardwareMap.get(DcMotorEx.class, "flmotor");
        frmotor = hardwareMap.get(DcMotorEx.class, "frmotor");
        blmotor = hardwareMap.get(DcMotorEx.class, "blmotor");
        brmotor = hardwareMap.get(DcMotorEx.class, "brmotor");
       shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        //intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Set power for all motors
        flmotor.setPower(1);
        frmotor.setPower(1);
        blmotor.setPower(1);
        brmotor.setPower(1);
        shooter1.setPower(0.5);
        shooter2.setPower(0.5);
        //intake.setPower(0.1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        // Set motor power to 0.5 at the start
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Example: Set motor power based on gamepad input
            // This is just a simple example. You can change this to test what you need.


            // Display the RPM of all motors
            printRpm();
        }
    }

    /**
     * Sets the power for the motors.
     * @param power The power to set, from -1.0 to 1.0.
     */
    private void setPower(double power) {
        flmotor.setPower(power);
        frmotor.setPower(power);
        blmotor.setPower(power);
        brmotor.setPower(power);
        shooter1.setPower(power);
        shooter2.setPower(power);
        //intake.setPower(power);
    }

    /**
     * Displays the current RPM of all motors on the driver station telemetry.
     */
    private void printRpm() {
        telemetry.addData("FL Motor RPM", "%.2f", getRpm(flmotor));
        telemetry.addData("FR Motor RPM", "%.2f", getRpm(frmotor));
        telemetry.addData("BL Motor RPM", "%.2f", getRpm(blmotor));
        telemetry.addData("BR Motor RPM", "%.2f", getRpm(brmotor));
        telemetry.addData("Shooter 1 RPM", "%.2f", getRpm(shooter1));
        telemetry.addData("Shooter 2 RPM", "%.2f", getRpm(shooter2));
        // telemetry.addData("Intake RPM", "%.2f", getRpm(intake));
        telemetry.update();
    }

    /**
     * Gets the current speed of a motor in Revolutions Per Minute (RPM).
     * @param motor The motor to get the RPM from.
     * @return The current RPM of the motor.
     */
    private double getRpm(DcMotorEx motor) {
        // getVelocity() returns ticks per second.
        // To convert ticks/sec to revolutions/minute:
        // (ticks/sec) * (60 sec/min) / (ticks/rev) = rev/min
        return (motor.getVelocity() * 60) / TICKS_PER_REVOLUTION;
    }
}
