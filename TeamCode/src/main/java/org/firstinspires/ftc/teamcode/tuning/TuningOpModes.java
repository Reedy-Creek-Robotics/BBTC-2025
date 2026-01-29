package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.EncoderGroup;
import com.acmerobotics.roadrunner.ftc.EncoderRef;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxQuadratureEncoderGroup;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
@Disabled
public final class TuningOpModes {
    // Set this to your MecanumDrive class
    public static final Class<?> DRIVE_CLASS = MecanumDrive.class;

    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewFactory dvf;
        if (DRIVE_CLASS.equals(MecanumDrive.class)) {
            dvf = hardwareMap -> {
                MecanumDrive md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                // Use the standard Control Hub IMU
                LazyImu lazyImu = md.lazyImu;

                List<EncoderGroup> encoderGroups = new ArrayList<>();
                List<EncoderRef> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();

                // Logic for Built-in Motor Encoders
                if (md.localizer instanceof MecanumDrive.DriveLocalizer) {
                    MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) md.localizer;

                    // Group all 4 drive motor encoders together
                    encoderGroups.add(new LynxQuadratureEncoderGroup(
                            hardwareMap.getAll(LynxModule.class),
                            Arrays.asList(dl.leftFront, dl.leftBack, dl.rightFront, dl.rightBack)
                    ));

                    // Map the indices of the encoders in the group above
                    leftEncs.add(new EncoderRef(0, 0));  // Left Front
                    leftEncs.add(new EncoderRef(0, 1));  // Left Back
                    rightEncs.add(new EncoderRef(0, 2)); // Right Back
                    rightEncs.add(new EncoderRef(0, 3)); // Right Front
                } else {
                    throw new RuntimeException("Localizer must be DriveLocalizer for 'No Hardware' setup.");
                }

                return new DriveView(
                        DriveType.MECANUM,
                        MecanumDrive.PARAMS.inPerTick,
                        MecanumDrive.PARAMS.maxWheelVel,
                        MecanumDrive.PARAMS.minProfileAccel,
                        MecanumDrive.PARAMS.maxProfileAccel,
                        encoderGroups,
                        Arrays.asList(md.leftFront, md.leftBack),
                        Arrays.asList(md.rightFront, md.rightBack),
                        leftEncs,
                        rightEncs,
                        new ArrayList<>(), // No Parallel Dead Wheels
                        new ArrayList<>(), // No Perpendicular Dead Wheels
                        lazyImu,
                        md.voltageSensor,
                        () -> new MotorFeedforward(MecanumDrive.PARAMS.kS,
                                MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                                MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick),
                        0
                );
            };
        } else {
            throw new RuntimeException("TuningOpModes only configured for MecanumDrive.");
        }

        // Standard Tuning Routines
        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));

        // Path following tests
        manager.register(metaForClass(org.firstinspires.ftc.teamcode.tuning.ManualFeedbackTuner.class), org.firstinspires.ftc.teamcode.tuning.ManualFeedbackTuner.class);
        manager.register(metaForClass(org.firstinspires.ftc.teamcode.tuning.SplineTest.class), org.firstinspires.ftc.teamcode.tuning.SplineTest.class);
        manager.register(metaForClass(org.firstinspires.ftc.teamcode.tuning.LocalizationTest.class), org.firstinspires.ftc.teamcode.tuning.LocalizationTest.class);

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    org.firstinspires.ftc.teamcode.tuning.ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}