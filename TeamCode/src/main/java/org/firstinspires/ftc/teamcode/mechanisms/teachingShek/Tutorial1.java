package org.firstinspires.ftc.teamcode.mechanisms.teachingShek;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "tutorial with Sheigoon")

public class Tutorial1 extends LinearOpMode {
    private DcMotor Sheik;

    public void initializehardware(){
        Sheik = hardwareMap.get(DcMotor.class,"67");

        Sheik.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Sheik.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addLine("Hardware initialized");
    }
    @Override
    public void runOpMode() throws InterruptedException {
        if(gamepad1.bWasPressed()){
            Sheik.setPower(1);
        }
    }
}
