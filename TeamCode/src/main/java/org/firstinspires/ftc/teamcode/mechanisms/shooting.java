package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="shooting", group="Linear Opmode")
// REMOVE 'abstract' from the line below
public class shooting extends LinearOpMode {
    private DcMotor shooter1;
    private DcMotor shooter2;

    private boolean shooterOn = false;
    private boolean b_curr = false;
    private boolean b_prev = false;

    @Override
    public void runOpMode() {
        initShooter();
        waitForStart();
        loopShooter();
    }

    public void initShooter() {
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
    }

    public void loopShooter() {
        while(opModeIsActive()) {
            b_prev = b_curr;
            b_curr = gamepad1.b; // Or whatever button you want to use
            if (b_curr && !b_prev) {
                shooterOn = !shooterOn;
            }
            shooter1.setPower(shooterOn ? 0.25 : 0.0);
            shooter2.setPower(shooterOn ? 0.25 : 0.0);
        }
    }
}
