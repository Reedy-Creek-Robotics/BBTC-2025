package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "motor test Ojas")
public class MOTOR_TPS_TEST extends LinearOpMode {
    private DcMotor flmotor, frmotor, blmotor, brmotor;
    private DcMotorEx shooter_1, /*shooter_2,*/ intakeTransfer;

    private DcMotorEx motorTest;
    private CRServo intakeServo;
    private IMU imu;

    private boolean testOn = false;

    private boolean bWasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            handleMechanisms();
            telemetry.update();
        }
    }
    private void initializeHardware() {
        shooter_1 = hardwareMap.get(DcMotorEx.class, "shooter_1");
        /*shooter_2 = hardwareMap.get(DcMotorEx.class, "shooter_2");*/

        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");
        motorTest = hardwareMap.get(DcMotorEx.class, "motorTest");

        intakeTransfer = hardwareMap.get(DcMotorEx.class, "intakeTransfer");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        // Motor directions
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        shooter_1.setDirection(DcMotorSimple.Direction.FORWARD);
        /* shooter_2.setDirection(DcMotorSimple.Direction.REVERSE);*/

        intakeTransfer.setDirection(DcMotorSimple.Direction.REVERSE);

        // Motor modes
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /* shooter_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        intakeTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Shooter PIDF (recommended starting values)
        shooter_1.setVelocityPIDFCoefficients(0.1, 0.0, 0.0, 11.7);
        /*shooter_2.setVelocityPIDFCoefficients(0.1, 0.0, 0.0, 11.7);*/

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.addLine("Hardware initialized");
    }
    private void handleMechanisms(){
        double rpm = (motorTest.getVelocity() / 28) * 60;
        if (gamepad1.b && !bWasPressed) testOn = !testOn;
        bWasPressed = gamepad1.b;
        if(testOn){
            motorTest.setPower(0.8);
            telemetry.addData("rpm is:", rpm);
            telemetry.addData("tps is:", motorTest.getVelocity());
        } else {
            motorTest.setPower(0.0);
        }
    }

}
