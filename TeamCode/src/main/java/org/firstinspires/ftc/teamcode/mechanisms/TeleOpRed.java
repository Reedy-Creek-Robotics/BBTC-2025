package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants.ShotType;

/**
 * Red-alliance TeleOp.
 *
 * Uses the red Limelight pipeline and checks for red-side target lock
 * (AprilTag tx offset ~2.8° for far shots). Everything else is handled
 * by BaseTeleOp's state machines.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Red")
public class TeleOpRed extends BaseTeleOp {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        configureCameraPipeline();
        telemetry.update();

        waitForStart();

        // Start background camera processing
        threadedCamera.start();

        while (opModeIsActive()) {
            handleDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x);
            handleMechanismInputs();
            updateMechanisms();
            addTelemetry();
        }

        // Clean shutdown
        threadedCamera.stop();
    }

    @Override
    protected void configureCameraPipeline() {
        threadedCamera.setPipelineRed();
    }

    @Override
    protected boolean isTargetLocked() {
        if (!isShooterOn() || getSelectedShotType() != ShotType.LONG) {
            return false;
        }
        // tx is negated to match the original convention: error = -(camera.getTx())
        double error = -(threadedCamera.getTx());
        return (error >= 1.5) && (error <= 3.5);
    }
}
