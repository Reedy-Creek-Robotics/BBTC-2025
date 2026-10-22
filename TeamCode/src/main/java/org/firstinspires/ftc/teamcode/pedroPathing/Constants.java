package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Constants {

    // 1. Drivetrain Motor Mapping, Directions & Velocity Tuning
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)//Change based on speed required and motor toll(best if kept on 1)
            .leftFrontMotorName("frontLeftMotor")
            .leftRearMotorName("backLeftMotor")
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            // Inverted motor directions to fix strafing left instead of right
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(65.96135177011567)//Changed because of new weight and odometry pod position 10/1/2026
            .yVelocity(57.52653299166461);//Changed because of new weight and odometry pod position 10/1/2026

    // 2. goBILDA Pinpoint Localizer Constants
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.0)//changed because of new odometry pod position 10/1/2026
            .strafePodX(-3.0)//changed because of new odometry pod position 10/1/2026
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // 3. Robot Mass (in kg), Zero Power Acceleration & Centripetal Scaling
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(4.57) //change this to what the weight of bot is in kilograms
            .centripetalScaling(0.0005)
            .forwardZeroPowerAcceleration(-35)
            .lateralZeroPowerAcceleration(-25)
           // .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0.0, 0.0004, 0.0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0, 0.00015, 0.0))//change based on weight and other factors
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0.0, 0.003, 0.0));//change based on weight and other factors


    // 4. Default Path Velocity Constraints
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // 5. Build and Return the Follower Instance with Pinpoint Localizer
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}