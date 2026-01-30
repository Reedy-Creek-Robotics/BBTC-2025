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
@TeleOp(name = "motor test Ojas")
public class MOTOR_TPS_TEST extends LinearOpMode {
    private DcMotor flmotor, frmotor, blmotor, brmotor;
    private DcMotorEx shooter_1, intakeTransfer;

    private DcMotorEx motorTest;

//    private DcMotorEx motorTest;
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

        intakeTransfer.setDirection(DcMotorSimple.Direction.REVERSE);

        // Motor modes
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.addLine("Hardware initialized");
    }
    private void handleMechanisms(){
        if (gamepad1.rightBumperWasPressed() && !bWasPressed) testOn = !testOn;
        bWasPressed = gamepad1.b;
        if(testOn){
            motorTest.setPower(1);
            telemetry.addData("tps is:", shooter_1.getVelocity());
        } else {
            motorTest.setPower(0.0);
        }
    }
}
