package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants.ShotType;

/**
 * Blue-alliance TeleOp.
 *
 * Uses the blue Limelight pipeline and checks for blue-side target lock
 * (AprilTag tx offset ~-2.8° for far shots). Everything else is handled
 * by BaseTeleOp's state machines.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Blue")
public class TeleOpBlue extends BaseTeleOp {

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
        threadedCamera.setPipelineBlue();
    }

    @Override
    protected boolean isTargetLocked() {
        if (!isShooterOn() || getSelectedShotType() != ShotType.LONG) {
            return false;
        }
        // tx is negated to match the original convention: error = -(camera.getTx())
        // Blue target range: error in [-3.5, -1.5]
        // (Fixed: original had a bug where the condition was impossible to satisfy)
        double error = -(threadedCamera.getTx());
        return (error >= -3.5) && (error <= -1.5);
    }
}
