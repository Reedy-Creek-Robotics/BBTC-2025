package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TankTest", group = "Testing")
public class TankTest extends LinearOpMode {

    // Declare drive motors (only 2 now)
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Ensure these match your Robot Configuration on the Driver Hub.
        leftMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Set motor directions. Usually one side needs to be reversed so positive power moves forward.
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to assign power via gamepad
            double leftPower;
            double rightPower;

            // Tank drive calculation:
            // Left stick controls left motor, Right stick controls right motor.
            // Note: Gamepad Y axes are natively negative when pushed up, so we negate them.
            leftPower  = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;

            // Send calculated power to wheels
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // Show the elapsed game time and wheel power telemetry.
            telemetry.addData("Status", "Running");
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }
    }
}