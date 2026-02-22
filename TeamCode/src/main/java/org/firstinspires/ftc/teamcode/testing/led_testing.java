package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class led_testing extends OpMode {
    public Servo led = null;
    @Override
    public void init() {
        led=hardwareMap.get(Servo.class,"led");
        led.setPosition(0.666);
    }

    @Override
    public void loop() {

    }
}
