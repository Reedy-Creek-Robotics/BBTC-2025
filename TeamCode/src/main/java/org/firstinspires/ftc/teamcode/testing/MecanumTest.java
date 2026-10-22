package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum TeleOp", group = "Linear OpMode")
public class MecanumTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // 1. Declare motors
        DcMotor leftFront  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        DcMotor leftRear   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRightMotor");
        DcMotor rightRear  = hardwareMap.get(DcMotor.class, "backRightMotor");

        // 2. Reverse left-side motors so forward power moves the robot forward
        // (Depending on your gearbox/mounting, you may need to flip these)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // 3. Optional: Set motor zero power behavior to BRAKE
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // gamepad stick inputs (Y is inverted on gamepads, so negate it)
            double y  = -gamepad1.left_stick_y; // Forward/Back
            double x  =  gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx =  gamepad1.right_stick_x; // Rotation

            // Calculate wheel power components
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            // Set power to motors
            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            // Send telemetry data to the Driver Station
            telemetry.addData("Motors", "LF (%.2f), RF (%.2f), LR (%.2f), RR (%.2f)",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}