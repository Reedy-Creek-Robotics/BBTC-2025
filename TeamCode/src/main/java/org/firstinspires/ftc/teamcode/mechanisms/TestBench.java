package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class TestBench {
    private DcMotor frwheel;
    private DcMotor flwheel;
    private DcMotor brwheel;
    private DcMotor blwheel;
    private IMU imu;


    public static final double TICKS_PER_REVOLUTION = 537.6;


    public void init(HardwareMap hwMap) {
        // --- IMU INITIALIZATION ---
        imu = hwMap.get(IMU.class, "imu");
        if (imu != null) {
            // Adjust Logo/Usb directions if your Control Hub is mounted differently
            IMU.Parameters parameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );
            imu.initialize(parameters);
        }


        // --- MOTORS INITIALIZATION ---
        frwheel = hwMap.get(DcMotor.class, "frwheel");
        if (frwheel != null) {
            frwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frwheel.setDirection(DcMotor.Direction.FORWARD);
        }


        flwheel = hwMap.get(DcMotor.class, "flwheel");
        if (flwheel != null) {
            flwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        brwheel = hwMap.get(DcMotor.class, "brwheel");
        if (brwheel != null) {
            brwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        blwheel = hwMap.get(DcMotor.class, "blwheel");
        if (blwheel != null) {
            blwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            blwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    // --- IMU ORIENTATION METHOD ---
    public YawPitchRollAngles getOrientation() {
        if (imu != null) {
            return imu.getRobotYawPitchRollAngles();
        }
        return new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);
    }


    // --- MOTOR POWER METHODS ---
    public void setAllWheelSpeeds(double speed) {
        if (frwheel != null) frwheel.setPower(speed);
        if (flwheel != null) flwheel.setPower(speed);
        if (brwheel != null) brwheel.setPower(speed);
        if (blwheel != null) blwheel.setPower(speed);
    }


    public void stopAllMotors() {
        setAllWheelSpeeds(0.0);
    }


    // --- ENCODER TICKS METHODS ---
    public int getFrWheelTicks() {
        return (frwheel != null) ? frwheel.getCurrentPosition() : 0;
    }


    public int getFlWheelTicks() {
        return (flwheel != null) ? flwheel.getCurrentPosition() : 0;
    }


    public int getBrWheelTicks() {
        return (brwheel != null) ? brwheel.getCurrentPosition() : 0;
    }


    public int getBlWheelTicks() {
        return (blwheel != null) ? blwheel.getCurrentPosition() : 0;
    }


    // --- REVOLUTION METHODS ---
    public double getFrWheelRevolutions() {
        return getFrWheelTicks() / TICKS_PER_REVOLUTION;
    }


    public double getFlWheelRevolutions() {
        return getFlWheelTicks() / TICKS_PER_REVOLUTION;
    }


    public double getBrWheelRevolutions() {
        return getBrWheelTicks() / TICKS_PER_REVOLUTION;
    }


    public double getBlWheelRevolutions() {
        return getBlWheelTicks() / TICKS_PER_REVOLUTION;
    }
}



