package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Simple Auto: Drive Forward", group="Linear Opmode")
public class autonomus extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private DcMotor flmotor = null;
    private DcMotor blmotor = null;
    private DcMotor frmotor = null;
    private DcMotor brmotor = null;

    // Constants for driving
    static final double     COUNTS_PER_MOTOR_REV    = 537.7; // Example for a GOBUILDA 5203 series motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;   // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;   // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the Driver Hub.
        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        brmotor = hardwareMap.get(DcMotor.class, "flmotor");

        // To drive forward, most robots need the motors on one side to be reversed.
        // The specific motor to reverse depends on the robot's build.
        // Reverse the right side motorskji]\
        frmotor.setDirection(DcMotor.Direction.REVERSE);
        brmotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders and set motors to run using encoders.
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Example: Drive forward for 24 inches
            int targetPosition = (int)(24 * COUNTS_PER_INCH); // Drive 24 inches

            // Set target position for all motors
            flmotor.setTargetPosition(targetPosition);
            frmotor.setTargetPosition(targetPosition);
            blmotor.setTargetPosition(targetPosition);
            brmotor.setTargetPosition(targetPosition);

            // Set motors to RUN_TO_POSITION mode
            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set power for all motors
            flmotor.setPower(DRIVE_SPEED);
            frmotor.setPower(DRIVE_SPEED);
            blmotor.setPower(DRIVE_SPEED);
            brmotor.setPower(DRIVE_SPEED);

            // Wait until all motors are done moving
            while (opModeIsActive() &&
                   (flmotor.isBusy() || frmotor.isBusy() || blmotor.isBusy() || brmotor.isBusy())) {
                telemetry.addData("Path", "Driving to target");
                telemetry.addData("Target Position", targetPosition);
                telemetry.addData("Current Pos (lf, rf, lb, rb)", "%7d, %7d, %7d, %7d",
                        flmotor.getCurrentPosition(),
                        frmotor.getCurrentPosition(),
                        blmotor.getCurrentPosition(),
                        brmotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            flmotor.setPower(0);
            frmotor.setPower(0);
            blmotor.setPower(0);
            brmotor.setPower(0);

            // Turn off RUN_TO_POSITION
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Helper method to set the mode for all drive motors
    private void setMotorMode(DcMotor.RunMode mode) {
        flmotor.setMode(mode);
        frmotor.setMode(mode);
        blmotor.setMode(mode);
        brmotor.setMode(mode);
    }
}
