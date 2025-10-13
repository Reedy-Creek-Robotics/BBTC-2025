package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Wheel Spin On B", group = "Tutorial")
public class TeleOPtellingwhattodo extends LinearOpMode {
    // Declare a motor variable
    private DcMotor flmotor;
    private DcMotor frmotor;
    private DcMotor blmotor;
    private DcMotor brmotor;




    // Define constants for the motor
    // These will need to be tuned for your specific motor and setup.
    // TICKS_PER_REV can be found on the motor's datasheet.
    // For example, a REV HD Hex Motor has 288, a Gobilda Yellow Jacket has 537.7.
    static final double TICKS_PER_REV = 537.7;
    static final double GEAR_REDUCTION = 1.0; // If you have a gearbox, change this value
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For calculating circumference

    static final double COUNTS_PER_INCH = (TICKS_PER_REV * GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double ROTATIONS_TO_PERFORM = 3.0;

    @Override
    public void runOpMode() {
        // Initialize the hardware.
        // The name "motor" must match the name in your robot's configuration file.
        flmotor = hardwareMap.get(DcMotor.class, "flmotor");

        // Set motor direction. Reverse if the motor spins the wrong way.
        flmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset the encoder and set the motor to run using encoders.
        flmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start.");
        telemetry.addData(">", "Press Gamepad 1 'B' button to spin the wheel 3 times.");
        telemetry.update();
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");

        // Set motor direction. Reverse if the motor spins the wrong way.
        frmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset the encoder and set the motor to run using encoders.
        frmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized"); // Repeats too many times
        telemetry.addData(">", "Press Play to start.");// Repeats too many times
        telemetry.addData(">", "Press Gamepad 1 'B' button to spin the wheel 3 times.");// Repeats too many times
        telemetry.update();// Repeats too many times
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");

        // Set motor direction. Reverse if the motor spins the wrong way.
        blmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset the encoder and set the motor to run using encoders.
        blmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");// Repeats too many times
        telemetry.addData(">", "Press Play to start.");// Repeats too many times
        telemetry.addData(">", "Press Gamepad 1 'B' button to spin the wheel 3 times.");// Repeats too many times
        telemetry.update();// Repeats too many times
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");

        // Set motor direction. Reverse if the motor spins the wrong way.
        brmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset the encoder and set the motor to run using encoders.
        brmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");// Repeats too many times
        telemetry.addData(">", "Press Play to start.");// Repeats too many times
        telemetry.addData(">", "Press Gamepad 1 'B' button to spin the wheel 3 times.");// Repeats too many times
        telemetry.update();// Repeats too many times

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Check if the 'b' button on gamepad 1 is pressed
            if (gamepad1.b) {
                // Calculate the target position in encoder ticks
                int targetPositionfl = flmotor.getCurrentPosition() + (int) (ROTATIONS_TO_PERFORM * TICKS_PER_REV);

                // Set the target position for the motor
                flmotor.setTargetPosition(targetPositionfl);

                // Switch the motor to RUN_TO_POSITION mode
                flmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set the motor power to start moving
                flmotor.setPower(0.5); // Set power to 50%. Adjust as needed.


                int targetPositionfr = frmotor.getCurrentPosition() + (int) (ROTATIONS_TO_PERFORM * TICKS_PER_REV);

                // Set the target position for the motor
                frmotor.setTargetPosition(targetPositionfr);

                // Switch the motor to RUN_TO_POSITION mode
                frmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set the motor power to start moving
                frmotor.setPower(0.5); // Set power to 50%. Adjust as needed.


                int targetPositionbl = blmotor.getCurrentPosition() + (int) (ROTATIONS_TO_PERFORM * TICKS_PER_REV);

                // Set the target position for the motor
                blmotor.setTargetPosition(targetPositionbl);

                // Switch the motor to RUN_TO_POSITION mode
                blmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set the motor power to start moving
                blmotor.setPower(0.5); // Set power to 50%. Adjust as needed.


                int targetPosition = brmotor.getCurrentPosition() + (int) (ROTATIONS_TO_PERFORM * TICKS_PER_REV);

                // Set the target position for the motor
                brmotor.setTargetPosition(targetPosition);

                // Switch the motor to RUN_TO_POSITION mode
                brmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set the motor power to start moving
                brmotor.setPower(0.5); // Set power to 50%. Adjust as needed.

            }
        }
    }
}
