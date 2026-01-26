package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDF_Testing extends OpMode {
    private DcMotorEx shooter;

    private DcMotor intakeTransfer;
    private CRServo servo;

    private double HighVelocity = 900;

    private double LowVelocity = 1300;

    double F = 0;
    double P = 0;

    double D = 0;

    double curTargetVelocity = HighVelocity;
    double[] stepSizes = {10, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;



    @Override
    public void init() {
        intakeTransfer = hardwareMap.get(DcMotorEx.class, "intakeTransfer");
        intakeTransfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
        servo = hardwareMap.get(CRServo.class,"intakeServo");
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter_1");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, D, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("init complete");
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == HighVelocity) {
                curTargetVelocity = LowVelocity;
                intakeTransfer.setPower(1);
                servo.setPower(1);
            } else {
                curTargetVelocity = HighVelocity;
                intakeTransfer.setPower(0);
                servo.setPower(0);
            }


        }
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if(gamepad1.rightBumperWasPressed()){
            D+= stepSizes[stepIndex];
        }
        if(gamepad1.leftBumperWasPressed()){
            D-= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if(gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }
        shooter.setVelocity(curTargetVelocity);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        double curVelocity = shooter.getVelocity();
        double error = curTargetVelocity - curVelocity;
        telemetry.addData("target velocity",curTargetVelocity);
        telemetry.addData("Current velocity","%.2f",curVelocity);
        telemetry.addData("Error","%.2f",error);
        telemetry.addData("Tuning P","%.4f (D-Pad U/D)",P);
        telemetry.addData("Tuning F","%.4f (D-Pad R/L",F);
        telemetry.addData("Step Size",curTargetVelocity);
        telemetry.addData("target velocity",curTargetVelocity);
        telemetry.addData("target velocity",curTargetVelocity);
    }
}
