package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name = "Main Competition TeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    private Follower follower;
    private Drivetrain drivetrain = new Drivetrain();
    private Turret turret = new Turret();

    private final double START_X = 36.0;
    private final double START_Y = 12.0;
    private final double START_ANGLE = 90.0;

    @Override
    public void runOpMode() {
        // Initialize PedroPathing Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_ANGLE)));

        // Initialize subsystems
        drivetrain.init(hardwareMap);
        turret.init(hardwareMap);

        telemetry.addData("Status", "Initialized. Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Update PedroPathing localization
            follower.update();
            Pose robotPose = follower.getPose();

            // 2. Handle Drivetrain (Driver 1)
            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double rotate  = gamepad1.right_stick_x;
            drivetrain.driveRobotRelative(forward, strafe, rotate);

            // 3. Handle Turret automated aiming (passes current robot pose)
            turret.update(robotPose);

            // 4. Telemetry
            telemetry.addData("Robot X", robotPose.getX());
            telemetry.addData("Robot Y", robotPose.getY());
            telemetry.addData("Turret Error (deg)", turret.getErrorDegrees());
            telemetry.update();
        }
    }
}