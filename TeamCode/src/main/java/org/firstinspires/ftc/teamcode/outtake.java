
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name = "Shooter Test", group = "Practice")
public class outtake extends OpMode {

    DcMotor shooterMotor;

    final double MOTOR_RUN_SPEED = 1.0; // Full power

    @Override
    public void init() {
        // Initialize the shooter motor from the hardware map
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter"); // Make sure the config name matches
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Shooter Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            shooterMotor.setPower(MOTOR_RUN_SPEED);
            telemetry.addData("Status", "Shooter Running at %.2f power", MOTOR_RUN_SPEED);
        } else {
            shooterMotor.setPower(0);
            telemetry.addData("Status", "Shooter Stopped. Hold X to run.");
        }

        // Optional: show encoder value
        telemetry.addData("Shooter Ticks", shooterMotor.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterMotor.setPower(0);
        telemetry.addData("Status", "Shooter Motor Off");
        telemetry.update();
    }
}
