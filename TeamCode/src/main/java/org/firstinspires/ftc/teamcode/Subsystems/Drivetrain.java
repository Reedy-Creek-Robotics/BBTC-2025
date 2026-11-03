package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class Drivetrain {
    public DcMotor flmotor, frmotor, blmotor, brmotor;
    public GoBildaPinpointDriver pinpoint;

    public void init(HardwareMap hardwareMap){
        // Hardware mapping for motors
        flmotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        blmotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frmotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        brmotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Hardware mapping for Pinpoint odometry (adjust string name if needed)
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Motor directions
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        // Zero power behavior
        DcMotor.ZeroPowerBehavior breakBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        flmotor.setZeroPowerBehavior(breakBehavior);
        flmotor.setPower(0.0);
        blmotor.setZeroPowerBehavior(breakBehavior);
        blmotor.setPower(0.0);
        frmotor.setZeroPowerBehavior(breakBehavior);
        frmotor.setPower(0.0);
        brmotor.setZeroPowerBehavior(breakBehavior);
        brmotor.setPower(0.0);

        // Run modes
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * Core drive mixing method (Robot-Relative)
     */
    public void driveRobotRelative(double forward, double strafe, double rotate) {
        DriveMath.WheelPowers powers = DriveMath.mecanumPowers(forward, strafe, rotate);
        flmotor.setPower(powers.frontLeft);
        blmotor.setPower(powers.backLeft);
        frmotor.setPower(powers.frontRight);
        brmotor.setPower(powers.backRight);
    }


    /**
     * Field-relative drive using the Pinpoint odometry computer
     */
    /*public void driveFieldRelative (double forward, double strafe, double rotate){
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);
        pinpoint.update();
        double currentHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        theta = AngleUnit.normalizeRadians(theta - currentHeading);

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        driveRobotRelative(newForward, newStrafe, rotate);
    }*/
}