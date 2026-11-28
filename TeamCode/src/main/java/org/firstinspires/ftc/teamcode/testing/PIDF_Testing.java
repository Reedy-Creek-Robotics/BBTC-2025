package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "PIDF testing")
public class PIDF_Testing extends OpMode {
    private DcMotorEx flywheel;

    private double HighVelocity = 1100;

    double F = 0;
    double P = 0;
    double D = 0;

    double curTargetVelocity = HighVelocity;
    double[] stepSizes = {10, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, D, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("init complete");
    }

    @Override
    public void loop() {
        // Toggle flywheel target velocity between 0 and HighVelocity using Y
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == HighVelocity) {
                curTargetVelocity = 0;
            } else {
                curTargetVelocity = HighVelocity;
            }
        }

        // Adjust step size using B
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Adjust F coefficient (D-Pad Left/Right)
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        // Adjust D coefficient (Bumpers)
        if (gamepad1.rightBumperWasPressed()) {
            D += stepSizes[stepIndex];
        }
        if (gamepad1.leftBumperWasPressed()) {
            D -= stepSizes[stepIndex];
        }

        // Adjust P coefficient (D-Pad Up/Down)
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        // Apply target velocity and PIDF coefficients
        flywheel.setVelocity(curTargetVelocity);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, D, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Telemetry
        double curVelocity = flywheel.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning D", "%.4f (Bumpers)", D);
        telemetry.addData("Tuning F", "%.4f (D-Pad R/L)", F);
        telemetry.addData("Step Size", "%.4f (B)", stepSizes[stepIndex]);
    }
}