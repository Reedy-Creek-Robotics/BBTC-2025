package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "motor test Ojas")
public class ServoTest extends LinearOpMode {
    //    private DcMotorEx motorTest;
    private CRServo servo;

    private boolean bWasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            handleMechanisms();
            telemetry.update();
        }
    }
    private void initializeHardware() {


        servo = hardwareMap.get(CRServo.class, "intakeServo");

        // Motor directions



        // Motor modes


        // IMU setup


        telemetry.addLine("Hardware initialized");
    }
    private void handleMechanisms(){
        boolean testOn;
        if (gamepad1.rightBumperWasPressed() && !bWasPressed);
        bWasPressed = gamepad1.b;
        if(gamepad1.b){
            servo.setPower(0.1);
        }

        if (gamepad1.a){
            servo.setPower(-1.0);
        }
        if(gamepad1.x){
            servo.setPower(0);
        }

    }
}
