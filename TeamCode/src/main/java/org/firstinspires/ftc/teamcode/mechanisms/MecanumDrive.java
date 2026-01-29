package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.sun.source.util.TreePathScanner;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Localizer;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive {
    public static class Params {
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        // Drive model parameters
        public static final double inPerTick = 0.023278;

        public static double tps = 1000;
        public double lateralInPerTick = inPerTick;
        public double trackWidth = 14;

        // Feedforward parameters
        public double kS = 0.12;
        public double kV = 0.012;
        public double kA = 0;

        // Constraints
        public double maxWheelVel = 75;
        public double minProfileAccel = -25;
        public double maxProfileAccel = 30;
        public double maxAngVel = 5;
        public double maxAngAccel = Math.PI;

        // Gains
        public double axialGain = 1.8;
        public double lateralGain = 1.5;
        public double headingGain = 3.8;

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0;
    }

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.trackWidth, PARAMS.inPerTick / PARAMS.lateralInPerTick);
    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront, shooter_1, intakeTransfer;
    public final CRServo intakeServo;
    public final VoltageSensor voltageSensor;
    public final LazyImu lazyImu;
    public final Localizer localizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;
        public final IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;
        private Pose2d pose;

        public DriveLocalizer(Pose2d pose) {
            leftFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
            leftBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
            rightBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
            rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));

            // Matching encoder direction to motor direction for forward = positive
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

            imu = lazyImu.get();
            this.pose = pose;
        }

        @Override public void setPose(Pose2d pose) { this.pose = pose; }
        @Override public Pose2d getPose() { return pose; }

        @Override
        public PoseVelocity2d update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;
                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;
                lastHeading = heading;
                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{(leftFrontPosVel.position - lastLeftFrontPos), leftFrontPosVel.velocity}).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{(leftBackPosVel.position - lastLeftBackPos), leftBackPosVel.velocity}).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{(rightBackPosVel.position - lastRightBackPos), rightBackPosVel.velocity}).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{(rightFrontPosVel.position - lastRightFrontPos), rightFrontPosVel.velocity}).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;
            lastHeading = heading;

            pose = pose.plus(new Twist2d(twist.line.value(), headingDelta));
            return twist.velocity().value();
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "flmotor");
        leftBack = hardwareMap.get(DcMotorEx.class, "blmotor");
        rightBack = hardwareMap.get(DcMotorEx.class, "brmotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "frmotor");
        shooter_1 = hardwareMap.get(DcMotorEx.class,"shooter_1");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeTransfer = hardwareMap.get(DcMotorEx.class, "intakeTransfer");

        // Drive Motor Directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- SHOOTER & INTAKE CONFIGURATION ---
        shooter_1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter_1.setVelocityPIDFCoefficients(900, 0.0, 0.5, 25.5);//P = 550 F = 25

        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeTransfer.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        localizer = new DriveLocalizer(pose);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels =
                kinematics.inverse(PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1.0;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, Math.abs(power.value()));
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;
        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;
            List<Double> disps = com.acmerobotics.roadrunner.Math.range(0, t.path.length(), Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t = (beginTs < 0) ? (beginTs = Actions.now()) * 0 : Actions.now() - beginTs;

            if (t >= timeTrajectory.duration) {
                setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            ).compute(txWorldTarget, localizer.getPose(), robotVelRobot);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            return true;
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;
        private double beginTs = -1;
        public TurnAction(TimeTurn turn) { this.turn = turn; }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t = (beginTs < 0) ? (beginTs = Actions.now()) * 0 : Actions.now() - beginTs;
            if (t >= turn.duration) {
                setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
                return false;
            }
            Pose2dDual<Time> txWorldTarget = turn.get(t);
            PoseVelocity2d robotVelRobot = updatePoseEstimate();
            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            ).compute(txWorldTarget, localizer.getPose(), robotVelRobot);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);
            return true;
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());
        while (poseHistory.size() > 100) poseHistory.removeFirst();
        return vel;
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(TurnAction::new, FollowTrajectoryAction::new, new TrajectoryBuilderParams(1e-6, new ProfileParams(0.25, 0.1, 1e-2)), beginPose, 0.0, defaultTurnConstraints, defaultVelConstraint, defaultAccelConstraint);
    }

    // --- SHOOTER ACTIONS ---
    public Action shooterOn() {
       return new InstantAction(() -> shooter_1.setVelocity(Params.tps));
    }
    public Action transferOn() {
        return new InstantAction(() -> {
            intakeTransfer.setPower(1.0);
            intakeServo.setPower(1.0);

        });
    }
    public Action intakeOn() {
        return new InstantAction(() -> intakeTransfer.setPower(1.0));
    }

    public Action intakeOff() {
        return new InstantAction(() -> intakeTransfer.setPower(0));
    }
    public Action stopAll() {
        return new InstantAction(() -> {
            intakeTransfer.setPower(0);
            intakeServo.setPower(0);
            shooter_1.setPower(0);
        });
    }
}