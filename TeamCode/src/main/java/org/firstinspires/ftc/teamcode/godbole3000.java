package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MecanumTeleOp", group = "Robot")
<<<<<<< HEAD
public class godbole3000 extends LinearOpMode {
=======
public class MecanumTeleOp extends LinearOpMode {
>>>>>>> db2038841270a75ee59de16492283fcc7100e6a9

    @Override
    public void runOpMode() {

        // Declare motors (names must match the Robot Configuration)
        DcMotor frontLeftMotor  = hardwareMap.dcMotor.get("flmotor");
        DcMotor backLeftMotor   = hardwareMap.dcMotor.get("blmotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frmotor");
        DcMotor backRightMotor  = hardwareMap.dcMotor.get("brmotor");

        // Reverse right side motors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            // Gamepad inputs
            double y  = -gamepad1.left_stick_y;   // Forward/back
            double x  =  gamepad1.left_stick_x * 1.1; // Strafe (adjusted)
<<<<<<< HEAD
            double rx =  -gamepad1.right_stick_x;  // Rotation
=======
            double rx =  gamepad1.right_stick_x;  // Rotation
>>>>>>> db2038841270a75ee59de16492283fcc7100e6a9

            // Normalize motor powers
            double denominator = Math.max(
                    Math.abs(y) + Math.abs(x) + Math.abs(rx),
                    1.0
            );

            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
<<<<<<< HEAD
}
=======
}
>>>>>>> db2038841270a75ee59de16492283fcc7100e6a9
