package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Camera;

@TeleOp(name = "PIDF + Limelight (BLUE)")
public class PIDF_Blue_WithCam extends OpMode {

    private DcMotorEx shooter;
    private DcMotor intakeTransfer;
    private CRServo servo;
    private Camera camera;

    private final double HIGH_VEL = 900;
    private final double LOW_VEL  = 810;

    double P = 0;
    double F = 0;

    double targetVelocity = HIGH_VEL;

    double[] stepSizes = {10, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter_1");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeTransfer = hardwareMap.get(DcMotorEx.class, "intakeTransfer");
        intakeTransfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeTransfer.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = hardwareMap.get(CRServo.class, "intakeServo");
        servo.setDirection(DcMotorSimple.Direction.REVERSE);

        camera = new Camera(hardwareMap);
        camera.switchToBlue();   // 🔵 pipeline 0

        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F)
        );

        telemetry.addLine("BLUE PIDF + Limelight ready");
    }

    @Override
    public void loop() {
        camera.update();

        // Toggle shooter speed
        if (gamepad1.yWasPressed()) {
            targetVelocity = (targetVelocity == HIGH_VEL) ? LOW_VEL : HIGH_VEL;
        }

        // Step size
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // PIDF tuning
        if (gamepad1.dpadUpWasPressed())    P += stepSizes[stepIndex];
        if (gamepad1.dpadDownWasPressed())  P -= stepSizes[stepIndex];
        if (gamepad1.dpadRightWasPressed()) F += stepSizes[stepIndex];
        if (gamepad1.dpadLeftWasPressed())  F -= stepSizes[stepIndex];

        shooter.setVelocity(targetVelocity);
        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F)
        );

        double velocity = shooter.getVelocity();
        double error = targetVelocity - velocity;

        telemetry.addLine("=== LIMELIGHT ===");
        telemetry.addData("Pipeline", "BLUE (0)");
        telemetry.addData("Tag ID", camera.getTid());
        telemetry.addData("Distance (in)", "%.1f", camera.getDistance());

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Current Vel", "%.1f", velocity);
        telemetry.addData("Error", "%.1f", error);

        telemetry.addLine("=== PIDF ===");
        telemetry.addData("P", "%.5f", P);
        telemetry.addData("F", "%.5f", F);
        telemetry.addData("Step Size", stepSizes[stepIndex]);

        telemetry.update();
    }

    @Override
    public void stop() {
        camera.stopCamera();
    }
}
