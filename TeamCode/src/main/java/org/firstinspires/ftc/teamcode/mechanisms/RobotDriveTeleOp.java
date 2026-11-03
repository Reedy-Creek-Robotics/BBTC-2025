package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name = "Robot Drive TeleOp", group = "Drive")
public class RobotDriveTeleOp extends LinearOpMode {

    // Instantiate your modular drivetrain subsystem
    private Drivetrain drivetrain = new Drivetrain();

    @Override
    public void runOpMode() {
        // Initialize the hardware through the subsystem
        drivetrain.init(hardwareMap);

        telemetry.addData("Status", "Initialized. Ready to drive!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read gamepad inputs
            // Note: Forward is negated (-) because FTC joysticks return negative values when pushed forward
            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double rotate  = gamepad1.right_stick_x;

            // Send inputs to the drivetrain subsystem
            drivetrain.driveRobotRelative(forward, strafe, rotate);

            // Optional telemetry for debugging
            telemetry.addData("Forward", forward);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);
            telemetry.update();
        }
    }
}