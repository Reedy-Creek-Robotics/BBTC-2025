package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This OpMode moves a robot forward a set distance (1 foot) using encoder counts.
 * <p>
 * To use this, connect a motor with an encoder to the Control Hub
 * and configure it in the robot configuration with the name "driveMotor".
 * Make sure this motor is connected to a wheel on your robot.
 * <p>
 * The robot will move forward at a constant power until it reaches
 * the target position. Telemetry will show the progress.
 */
@Autonomous(name = "Drive Forward 1 Foot")
public class MotorTesting extends LinearOpMode {

    // Define the motor object
    private DcMotorEx driveMotor;

    // --- DEFINE CONSTANTS ---
    // UPDATE THESE VALUES for your specific robot.

    // Ticks per revolution for your motor.
    // Find the value for your motor from its datasheet.
    private static final double TICKS_PER_MOTOR_REV = 537.7; // Example: goBILDA 5203 series

    // The gear ratio between the motor and the wheel.
    // If the motor is connected directly to the wheel, this is 1.0.
    private static final double GEAR_RATIO = 1.0;

    // The diameter of the wheel in inches.
    private static final double WHEEL_DIAMETER_INCHES = 4.0;

    // Define the power at which to run the motor.
    private static final double MOTOR_POWER = 0.5;

    // Calculate the number of ticks per inch of robot movement.
    private static final double TICKS_PER_INCH = (TICKS_PER_MOTOR_REV * GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // The distance we want to travel in inches.
    private static final double DISTANCE_TO_TRAVEL_INCHES = 12.0; // 1 foot

    @Override
    public void runOpMode() {
        // Initialize the motor from the hardware map
        driveMotor = hardwareMap.get(DcMotorEx.class, "driveMotor");

        // Set the motor direction. Reverse this if the robot moves backward.
        driveMotor.setDirection(DcMotor.Direction.FORWARD);

        // Reset the encoder and set the motor to run to a specific position.
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("Ready to start.");
        telemetry.addData("Target Distance", "%.2f inches", DISTANCE_TO_TRAVEL_INCHES);
        telemetry.update();

        // Wait for the Play button to be pressed
        waitForStart();

        // Set the motor to run at the specified power
        driveMotor.setPower(MOTOR_POWER);

        // Calculate the target position in encoder ticks
        int targetPosition = (int) (DISTANCE_TO_TRAVEL_INCHES * TICKS_PER_INCH);
        driveMotor.setTargetPosition(targetPosition);

        while (opModeIsActive()) {
            // Display the current position and target on the Driver Hub
            telemetry.addData("Current Position", driveMotor.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.update();

            // The motor is running to the position, so we can exit the loop
            // once it gets close to the target.
            if (!driveMotor.isBusy()) {
                break; // Exit the loop
            }
        }

        // Stop the motor when the OpMode is stopped
        driveMotor.setPower(0);
    }
}
