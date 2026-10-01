package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servo")
public class SServo extends OpMode {
public Servo servo;

    public void init() {

    }

    @Override
    public void loop() {
        if (gamepad1.a) {//the y and a buttons are fliped
            servo.setPosition(0.5);
        }
        else if (gamepad1.b) {
            servo.setPosition(-0.5);
        }
        else if (gamepad1.y) {//the y and a buttons are fliped
            servo.setPosition(0);
        }
    }
}
