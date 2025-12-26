package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.TestBench

import java.util.List;

@TeleOp(name = "Limelight3A All Data Sensing", group = "Sensors")
public class AprilTagAutoAlignFTC extends OpMode {

    private Limelight3A limelight;
    private double dist;

    TestBench bench = new TestBench();

    @Override
    public void init() {
        bench.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetry.addLine("Limelight 3A initialized. Ready to sense data.");
        telemetry.update();
    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = bench.getOrientation();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES))
    }
}
