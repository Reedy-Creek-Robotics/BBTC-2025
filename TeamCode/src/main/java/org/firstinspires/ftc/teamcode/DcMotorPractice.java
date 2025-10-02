// getting revolutions from encoders
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.TestBench;

@Autonomous(name = "DcMotor Revolution Test", group = "Practice")
public class DcMotorPractice extends OpMode {
    TestBench bench = new TestBench();

    final double MOTOR_RUN_SPEED = 0.4; // Speed at which motors will run for the test

    @Override
    public void init() {
        bench.init(hardwareMap); // This initializes motors and resets encoders in TestBench
        telemetry.addData("Status", "Initialized and Encoders Reset");
        telemetry.addData("Using TPR", "%.2f ticks/revolution", TestBench.TICKS_PER_REVOLUTION);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Display initial (close to zero) revolutions before start
        telemetry.addData("FR Revs (Pre-Start)", "%.3f", bench.getFrWheelRevolutions());
        telemetry.addData("FL Revs (Pre-Start)", "%.3f", bench.getFlWheelRevolutions());
        telemetry.update();
    }

    @Override
    public void start() {
        // Start motors when the OpMode begins
        bench.setAllWheelSpeeds(MOTOR_RUN_SPEED);
        telemetry.addData("Status", "Motors Running at %.2f power", MOTOR_RUN_SPEED);
        telemetry.update();
    }

    @Override
    public void loop() {
        // In the main loop, continuously get the revolutions from TestBench and display them
        double frRevolutions = bench.getFrWheelRevolutions();
        double flRevolutions = bench.getFlWheelRevolutions();
        double brRevolutions = bench.getBrWheelRevolutions();
        double blRevolutions = bench.getBlWheelRevolutions();
        telemetry.addData("Front Right Revs", "%.3f", frRevolutions);
        telemetry.addData("Front Left Revs", "%.3f", flRevolutions);
        telemetry.addData("Back Right Revs", "%.3f", brRevolutions);
        telemetry.addData("Back Left Revs", "%.3f", blRevolutions);
        telemetry.addLine("---"); // Separator
        telemetry.addData("FR Ticks", bench.getFrWheelTicks());
        telemetry.addData("FL Ticks", bench.getFlWheelTicks());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Ensure motors are stopped when the OpMode is finished
        bench.stopAllMotors();
        telemetry.addData("Status", "Stopped. Motors Off.");
        telemetry.update();
    }
}
