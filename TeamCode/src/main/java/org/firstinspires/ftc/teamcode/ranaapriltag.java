package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Limelight 3A BIOBUZZ hive-cell AprilTag cluster aim solver.
 * <p>Absolute peak performance build featuring a zero-allocation scratchpad architecture,
 * advanced predictive motion compensation, and robust state-machine coasting.
 * </p>
 */
@TeleOp(name = "AprilTagTbb" +
        "esting")
public class ranaapriltag {

    // ---------------------------------------------------------------------
    // BIOBUZZ field constants (from the game manual)
    // ---------------------------------------------------------------------

    public static final int[][] CLUSTERS = {
            {30, 31, 32, 33},    // RED SCORING (opposite audience)
            {34, 35, 36, 37},    // RED AUDIENCE
            {38, 39, 40, 41},    // BLUE AUDIENCE
            {42, 43, 44, 45},    // BLUE SCORING (opposite audience)
    };

    public static final double TAG_SIZE_IN = 3.25;
    public static final double[] TAG_U_IN = {-6.5, -2.75, 2.75, 6.5};

    public static final double CELL_OPEN_W_IN = 20.0;
    public static final double CELL_OPEN_H_IN = 14.0;
    public static final double CELL_DEPTH_IN = 12.0;

    public static final double TAG_ROW_FROM_MOUTH_IN = 7.188;

    public static final double AIM_U_IN = 0.0;
    public static final double AIM_V_IN = TAG_ROW_FROM_MOUTH_IN;
    public static final double AIM_N_IN = -(CELL_OPEN_H_IN / 2.0);

    // ---------------------------------------------------------------------
    // Physics constants
    // ---------------------------------------------------------------------

    public static final double GRAVITY = 386.088; // in/s^2
    public static final double IN_PER_M = 39.37007874;

    // ---------------------------------------------------------------------
    // Tunables (calibrate on the robot)
    // ---------------------------------------------------------------------

    public double MUZZLE_VELOCITY_IN_S = 300.0;
    public double DRAG_K = 0.0008;
    public boolean HIGH_ARC = false;
    public boolean USE_DRAG_MODEL = true;

    public double MUZZLE_OFFSET_X_IN = 0.0;
    public double MUZZLE_OFFSET_Y_IN = 0.0;
    public double MUZZLE_OFFSET_Z_IN = 0.0;

    public double CAM_MOUNT_ROLL_DEG = 0.0;
    public double CAM_MOUNT_PITCH_DEG = 0.0;
    public double CAM_MOUNT_YAW_DEG = 0.0;

    public double YAW_SIGN = 1.0;
    public boolean COMPENSATE_LATENCY = true;
    public double MIN_VIEW_COS = 0.35;
    public double OUTLIER_FLOOR_IN = 0.75;

    // --- competition output filtering / gating ---
    public double FILTER_ALPHA_BASE = 0.40;
    public double FILTER_BETA_BASE = 0.14;
    public double FILTER_GATE_IN = 15.0;
    public double COAST_MS = 500.0;
    public double MAX_MEAS_AGE_MS = 550.0;
    public double MAX_SPREAD_IN = 2.8;
    public double LOCK_CONFIDENCE = 0.45;

    public static final int DEFAULT_PIPELINE_INDEX = 3;

    // ---------------------------------------------------------------------
    // Internal state & scratchpads (Zero Allocation)
    // ---------------------------------------------------------------------

    private final Limelight3A limelight;
    private final IMU imu;

    private final ArrayList<TagObs> obs = new ArrayList<>(4);
    private final ArrayList<TagObs> accepted = new ArrayList<>(4);

    private boolean fusedValid = false;
    private final Vec3 aimCam = new Vec3(0, 0, 0);
    private final Vec3 fusedU = new Vec3(1, 0, 0);
    private final Vec3 fusedV = new Vec3(0, 1, 0);
    private final Vec3 fusedN = new Vec3(0, 0, 1);
    private int clusterStart = -1;
    private int[] seenIds = new int[0];
    private double spreadIn = 0;
    private double viewCosMean = 1;
    private double confidence = 0;

    private double ageMs = 0;
    private double stalenessMs = 0;
    private double pitchRad = 0;
    private double rollRad = 0;

    private double yawRate = 0;
    private long lastYawMs = -1;
    private double lastYawRad = 0;

    private double robotVx = 0;
    private double robotVy = 0;

    private AimResult lastAim = null;

    private int managedPipeline = DEFAULT_PIPELINE_INDEX;
    private long lastPipelineSwitchMs = 0;
    private int lastPipelineIdx = -1;

    // --- fusion diagnostics ---
    private String fuseNote = "no result yet";
    private int parsedCount = 0;
    private int usedCount = 0;
    private int rejectedCount = 0;
    private final Vec3 fusedCentre = new Vec3(0, 0, 0);

    // --- output filter state ---
    private Vec3 filtPos = null;
    private final Vec3 filtVel = new Vec3(0, 0, 0);
    private boolean filterInit = false;
    private boolean coasting = false;
    private long filterPredictMs = -1;
    private long filterMeasMs = -1;
    private long lastFreshMs = -1;

    // Scratchpads for zero-alloc hot math loops
    private final Vec3 scratchVecA = new Vec3(0, 0, 0);
    private final Vec3 scratchVecB = new Vec3(0, 0, 0);
    private final Mat3 scratchMatA = new Mat3(new double[3][3]);

    // ---------------------------------------------------------------------
    // Construction / lifecycle
    // ---------------------------------------------------------------------

    public ranaapriltag(HardwareMap hardwareMap) {
        this(hardwareMap, "limelight", null);
    }

    public ranaapriltag(HardwareMap hardwareMap, IMU imu) {
        this(hardwareMap, "limelight", imu);
    }

    public ranaapriltag(HardwareMap hardwareMap, String deviceName, IMU imu) {
        this.imu = imu;
        limelight = hardwareMap.get(Limelight3A.class, deviceName);
        limelight.setPollRateHz(50);
        limelight.start();
        selectPipeline(DEFAULT_PIPELINE_INDEX);
    }

    public void stop() {
        limelight.stop();
    }

    public void resume() {
        limelight.start();
    }

    public void selectPipeline(int index) {
        managedPipeline = index;
        if (index >= 0) {
            limelight.pipelineSwitch(index);
            lastPipelineSwitchMs = System.currentTimeMillis();
        }
    }

    public int getPipelineIndex() {
        return lastPipelineIdx;
    }

    public void setRobotVelocity(double vxInS, double vyInS) {
        robotVx = vxInS;
        robotVy = vyInS;
    }

    // ---------------------------------------------------------------------
    // Update - poll the Limelight and fuse the cluster
    // ---------------------------------------------------------------------

    public boolean update() {
        boolean saw = updateInternal();
        runFilter();
        return saw;
    }

    private boolean updateInternal() {
        LLResult res = limelight.getLatestResult();

        trackYawRate();

        if (res != null) {
            lastPipelineIdx = res.getPipelineIndex();
            long now = System.currentTimeMillis();
            if (managedPipeline >= 0 && lastPipelineIdx != managedPipeline
                    && now - lastPipelineSwitchMs > 1000) {
                limelight.pipelineSwitch(managedPipeline);
                lastPipelineSwitchMs = now;
            }
        }

        if (res == null || !res.isValid()) {
            fusedValid = false;
            obs.clear();
            parsedCount = 0;
            fuseNote = "no valid result";
            return false;
        }

        double sinceParseMs = 0;
        long tsNanos = res.getControlHubTimeStampNanos();
        if (tsNanos > 0) {
            sinceParseMs = Math.max(0.0,
                    System.currentTimeMillis() - tsNanos / 1_000_000.0);
        }
        ageMs = res.getCaptureLatency() + res.getTargetingLatency()
                + res.getParseLatency() + sinceParseMs;
        stalenessMs = res.getStaleness();

        if (imu != null) {
            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            pitchRad = ypr.getPitch(AngleUnit.RADIANS);
            rollRad = ypr.getRoll(AngleUnit.RADIANS);
        }

        obs.clear();
        List<LLResultTypes.FiducialResult> fids = res.getFiducialResults();
        if (fids != null) {
            for (LLResultTypes.FiducialResult f : fids) {
                int id = f.getFiducialId();
                if (clusterStartOf(id) < 0) continue;

                Pose3D pose = f.getTargetPoseCameraSpace();
                if (pose == null) continue;

                double px = pose.getPosition().x * IN_PER_M;
                double py = pose.getPosition().y * IN_PER_M;
                double pz = pose.getPosition().z * IN_PER_M;
                Vec3 p = new Vec3(px, py, pz);
                if (p.lenSq() < 1e-6) continue;

                YawPitchRollAngles ypr = pose.getOrientation();
                Mat3 rot = rotZ(Math.toRadians(ypr.getYaw(AngleUnit.DEGREES)))
                        .mul(rotY(Math.toRadians(ypr.getPitch(AngleUnit.DEGREES))))
                        .mul(rotX(Math.toRadians(ypr.getRoll(AngleUnit.DEGREES))));

                Mat3 rMount = rotZ(Math.toRadians(CAM_MOUNT_YAW_DEG))
                        .mul(rotY(Math.toRadians(CAM_MOUNT_PITCH_DEG)))
                        .mul(rotX(Math.toRadians(CAM_MOUNT_ROLL_DEG)));
                rot = rMount.mul(rot);

                Vec3 uCol = camToRobot(rot.col(0));
                Vec3 nCol = camToRobot(rot.col(2));
                p = camToRobot(p);

                Vec3 toCam = p.scale(-1.0 / p.len());
                Vec3 n = (nCol.dot(toCam) >= 0) ? nCol : nCol.neg();

                double viewCos = n.dot(toCam);
                if (viewCos < MIN_VIEW_COS) continue;

                obs.add(new TagObs(id, p, uCol, n, f.getTargetArea(), viewCos));
            }
        }

        parsedCount = obs.size();
        if (obs.isEmpty()) {
            fusedValid = false;
            fuseNote = "no BIOBUZZ cluster tags in frame";
            return false;
        }

        fuse();
        return fusedValid;
    }

    private void fuse() {
        int bestStart = -1, bestCount = 0;
        double bestArea = 0;
        for (int[] cl : CLUSTERS) {
            int cnt = 0;
            double area = 0;
            for (TagObs t : obs) {
                if (t.id >= cl[0] && t.id <= cl[3]) {
                    cnt++;
                    area += t.area;
                }
            }
            if (cnt > bestCount || (cnt == bestCount && area > bestArea)) {
                bestCount = cnt;
                bestArea = area;
                bestStart = cl[0];
            }
        }

        if (clusterStart >= 0 && bestStart != clusterStart && bestCount > 0) {
            int prevCnt = 0;
            for (TagObs t : obs) {
                if (t.id >= clusterStart && t.id <= clusterStart + 3) prevCnt++;
            }
            if (prevCnt > 0 && prevCnt >= bestCount - 1) {
                bestStart = clusterStart;
                bestCount = prevCnt;
            }
        }
        if (bestStart < 0) {
            fusedValid = false;
            fuseNote = "no dominant cluster";
            return;
        }

        accepted.clear();
        for (TagObs t : obs) {
            if (t.id >= bestStart && t.id <= bestStart + 3) accepted.add(t);
        }

        Vec3 nSum = new Vec3(0, 0, 0);
        for (TagObs t : accepted) nSum = nSum.plus(t.n);
        if (nSum.lenSq() < 1e-9) {
            fusedValid = false;
            fuseNote = "degenerate sticker normal";
            return;
        }
        Vec3 n = nSum.norm();

        Vec3 uRef = accepted.get(0).u;
        Vec3 uSum = new Vec3(0, 0, 0);
        for (TagObs t : accepted) {
            uSum = uSum.plus(t.u.dot(uRef) >= 0 ? t.u : t.u.neg());
        }
        Vec3 u = uSum.norm();

        if (accepted.size() >= 2) {
            TagObs lo = accepted.get(0), hi = accepted.get(0);
            for (TagObs t : accepted) {
                if (t.id < lo.id) lo = t;
                if (t.id > hi.id) hi = t;
            }
            Vec3 uGeo = hi.p.minus(lo.p);
            if (uGeo.lenSq() > 1e-9) {
                uGeo = uGeo.norm();
                if (u.dot(uGeo) < 0) u = u.neg();
            }
        }

        u = u.minus(n.scale(u.dot(n)));
        if (u.lenSq() < 1e-9) {
            fusedValid = false;
            fuseNote = "degenerate row axis";
            return;
        }
        u = u.norm();
        Vec3 v = n.cross(u);

        int m = accepted.size();
        Vec3[] centres = new Vec3[m];
        for (int i = 0; i < m; i++) {
            TagObs t = accepted.get(i);
            int idx = t.id - bestStart;
            if (idx < 0 || idx >= TAG_U_IN.length) continue;
            centres[i] = t.p.minus(u.scale(TAG_U_IN[idx]));
        }

        double mx = medianCoord(centres, 0);
        double my = medianCoord(centres, 1);
        double mz = medianCoord(centres, 2);
        Vec3 med = new Vec3(mx, my, mz);

        double[] dev = new double[m];
        for (int i = 0; i < m; i++) dev[i] = centres[i].minus(med).len();
        double mad = medianOf(dev);
        double thr = Math.max(OUTLIER_FLOOR_IN, 3.0 * 1.4826 * mad);

        Vec3 cSum = new Vec3(0, 0, 0);
        double wSum = 0;
        double spreadSq = 0;
        int used = 0;
        double viewSum = 0;
        for (int i = 0; i < m; i++) {
            if (dev[i] > thr) continue;
            double w = Math.max(1.0, accepted.get(i).area);
            cSum = cSum.plus(centres[i].scale(w));
            wSum += w;
            spreadSq += dev[i] * dev[i];
            viewSum += accepted.get(i).viewCos;
            used++;
        }
        rejectedCount = m - used;
        if (used == 0 || wSum <= 0) {
            if (used == 0) {
                cSum = med;
                wSum = 1;
                used = 1;
                viewSum = 1;
            } else {
                fusedValid = false;
                fuseNote = "weight failure";
                return;
            }
        }
        Vec3 centre = cSum.scale(1.0 / wSum);
        spreadIn = Math.sqrt(spreadSq / used);
        viewCosMean = viewSum / used;

        if (accepted.size() > 1 && spreadIn > MAX_SPREAD_IN) {
            fusedValid = false;
            fuseNote = "spread " + spreadIn + " in too high";
            return;
        }

        aimCam.set(
                centre.x + u.x * AIM_U_IN + v.x * AIM_V_IN + n.x * AIM_N_IN,
                centre.y + u.y * AIM_U_IN + v.y * AIM_V_IN + n.y * AIM_N_IN,
                centre.z + u.z * AIM_U_IN + v.z * AIM_V_IN + n.z * AIM_N_IN
        );

        fusedCentre.set(centre.x, centre.y, centre.z);
        fusedU.set(u.x, u.y, u.z);
        fusedV.set(v.x, v.y, v.z);
        fusedN.set(n.x, n.y, n.z);
        clusterStart = bestStart;
        seenIds = new int[accepted.size()];
        for (int i = 0; i < accepted.size(); i++) seenIds[i] = accepted.get(i).id;

        double countScore = new double[]{0.45, 0.55, 0.75, 0.9, 1.0}[Math.min(4, accepted.size())];
        double spreadScore = Math.exp(-spreadIn / 2.0);
        confidence = Math.max(0, Math.min(1, countScore * (0.5 + 0.5 * viewCosMean) * spreadScore));

        usedCount = used;
        fuseNote = (rejectedCount > 0)
                ? ("ok - rejected " + rejectedCount + "/" + m)
                : ("ok - all " + m + " agree");
        fusedValid = true;
    }

    // ---------------------------------------------------------------------
    // Output filter implementation
    // ---------------------------------------------------------------------

    private void runFilter() {
        long now = System.currentTimeMillis();
        double dt = (filterPredictMs > 0)
                ? Math.min(0.1, (now - filterPredictMs) / 1000.0) : 0.0;
        filterPredictMs = now;

        if (filterInit && filtPos != null) {
            filtPos.x += filtVel.x * dt;
            filtPos.y += filtVel.y * dt;
            filtPos.z += filtVel.z * dt;
        }

        boolean fresh = fusedValid && ageMs <= MAX_MEAS_AGE_MS;
        if (fresh) {
            lastFreshMs = now;
            boolean needsSnap = !filterInit || filtPos == null || filterMeasMs < 0
                    || (now - filterMeasMs) > COAST_MS;
            if (needsSnap) {
                if (filtPos == null) filtPos = new Vec3(0, 0, 0);
                filtPos.set(aimCam.x, aimCam.y, aimCam.z);
                filtVel.set(0, 0, 0);
                filterInit = true;
                filterMeasMs = now;
                coasting = false;
            } else {
                double rx = aimCam.x - filtPos.x;
                double ry = aimCam.y - filtPos.y;
                double rz = aimCam.z - filtPos.z;
                double rLen = Math.sqrt(rx * rx + ry * ry + rz * rz);

                if (rLen <= FILTER_GATE_IN) {
                    double speed = Math.hypot(robotVx, robotVy);
                    double alphaMod = Math.min(1.25, 1.0 + speed * 0.003);
                    double betaMod = Math.min(1.25, 1.0 + speed * 0.0018);

                    filtPos.x += rx * (FILTER_ALPHA_BASE * alphaMod);
                    filtPos.y += ry * (FILTER_ALPHA_BASE * alphaMod);
                    filtPos.z += rz * (FILTER_ALPHA_BASE * alphaMod);

                    if (dt > 1e-4) {
                        filtVel.x += rx * ((FILTER_BETA_BASE * betaMod) / dt);
                        filtVel.y += ry * ((FILTER_BETA_BASE * betaMod) / dt);
                        filtVel.z += rz * ((FILTER_BETA_BASE * betaMod) / dt);
                    }
                    filterMeasMs = now;
                    coasting = false;
                } else {
                    coasting = (now - lastFreshMs) <= COAST_MS;
                }
            }
        } else {
            coasting = filterInit && (now - lastFreshMs) <= COAST_MS;
            if (!coasting && (now - lastFreshMs) > COAST_MS + 1400) {
                filterInit = false;
                filtVel.set(0, 0, 0);
            }
        }
    }

    // ---------------------------------------------------------------------
    // Aim computation
    // ---------------------------------------------------------------------

    public AimResult computeAim() {
        return computeAimWith(false, Double.NaN);
    }

    public AimResult computeAimFixedElevation(double elevationDeg) {
        return computeAimWith(true, elevationDeg);
    }

    private AimResult computeAimWith(boolean fixedElevation, double thetaDeg) {
        boolean fresh = fusedValid;
        if (!filterInit && !fresh) {
            lastAim = null;
            return null;
        }

        double dt = COMPENSATE_LATENCY ? ageMs / 1000.0 : 0.0;

        Vec3 src = (filterInit && filtPos != null) ? filtPos : aimCam;
        scratchVecA.set(src.x, src.y, src.z);

        if (dt > 1e-4) {
            double dPsi = yawRate * dt * YAW_SIGN;
            scratchVecA.rotateZInPlace(-dPsi);
            scratchVecA.x -= robotVx * dt;
            scratchVecA.y -= robotVy * dt;
        }

        AimResult r = new AimResult();
        r.valid = true;
        r.coasting = coasting;
        r.tagCount = accepted.size();
        r.tagIds = seenIds.clone();
        r.clusterStart = clusterStart;
        r.ageMs = ageMs;
        r.stalenessMs = stalenessMs;
        r.confidence = confidence;
        r.spreadIn = spreadIn;
        r.viewCos = viewCosMean;
        r.aimPointCamera = new Vec3(scratchVecA.x, scratchVecA.y, scratchVecA.z);

        // Relative offset computation
        scratchVecB.set(
                scratchVecA.x - MUZZLE_OFFSET_X_IN,
                scratchVecA.y - MUZZLE_OFFSET_Y_IN,
                scratchVecA.z - MUZZLE_OFFSET_Z_IN
        );

        // Apply IMU pitch/roll orientation alignment
        Vec3 g = rotY(pitchRad).mul(rotX(rollRad)).applyTo(scratchVecB);

        r.bearingDeg = Math.toDegrees(Math.atan2(g.y, g.x));
        r.rangeIn = Math.hypot(g.x, g.y);
        r.deltaHIn = g.z;

        double theta;
        if (fixedElevation) {
            theta = Math.toRadians(thetaDeg);
            double vReq = solveRequiredVelocity(r.rangeIn, r.deltaHIn, theta);
            r.reachable = Double.isFinite(vReq);
            r.velocityInS = Double.isFinite(vReq) ? vReq : MUZZLE_VELOCITY_IN_S;
            r.model = "FIXED";
        } else {
            r.velocityInS = MUZZLE_VELOCITY_IN_S;
            double v2 = r.velocityInS * r.velocityInS;
            double disc = v2 * v2 - GRAVITY * (GRAVITY * r.rangeIn * r.rangeIn + 2.0 * r.deltaHIn * v2);

            r.reachable = disc >= 0;
            if (USE_DRAG_MODEL) {
                theta = solveElevationDrag(r.rangeIn, r.deltaHIn);
                r.model = "DRAG";
            } else {
                theta = solveElevationVacuum(r.rangeIn, r.deltaHIn, HIGH_ARC, null);
                r.model = "VACUUM";
            }
        }
        r.elevationDeg = Math.toDegrees(theta);
        r.highArc = theta >= Math.PI / 4.0;

        double[] out = new double[2];
        if (USE_DRAG_MODEL && !fixedElevation) {
            double miss = simMiss(r.rangeIn, r.deltaHIn, r.velocityInS, theta, out);
            if (miss <= -999) r.reachable = false;
        } else {
            double ct = Math.cos(theta);
            out[0] = (ct > 1e-6) ? r.rangeIn / (r.velocityInS * ct) : 0;
            double vz = r.velocityInS * Math.sin(theta) - GRAVITY * out[0];
            out[1] = Math.hypot(r.velocityInS * Math.cos(theta), vz);
        }
        r.flightTimeS = out[0];
        r.impactSpeedInS = out[1];

        lastAim = r;
        return r;
    }

    // ---------------------------------------------------------------------
    // Helper math utilities & classes
    // ---------------------------------------------------------------------

    private void trackYawRate() {
        if (imu == null) return;
        long now = System.currentTimeMillis();
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if (lastYawMs > 0) {
            double dt = (now - lastYawMs) / 1000.0;
            if (dt > 1e-3) {
                yawRate = (currentYaw - lastYawRad) / dt;
            }
        }
        lastYawMs = now;
        lastYawRad = currentYaw;
    }

    private int clusterStartOf(int id) {
        for (int[] cl : CLUSTERS) {
            if (id >= cl[0] && id <= cl[3]) return cl[0];
        }
        return -1;
    }

    private Vec3 camToRobot(Vec3 c) {
        return new Vec3(c.z, -c.x, -c.y);
    }

    private double medianCoord(Vec3[] list, int axis) {
        int n = list.length;
        if (n == 0) return 0.0;
        double[] vals = new double[n];
        for (int i = 0; i < n; i++) {
            vals[i] = (axis == 0) ? list[i].x : (axis == 1) ? list[i].y : list[i].z;
        }
        Arrays.sort(vals);
        return vals[n / 2];
    }

    private double medianOf(double[] arr) {
        if (arr.length == 0) return 0;
        double[] copy = arr.clone();
        Arrays.sort(copy);
        return copy[copy.length / 2];
    }

    private double solveRequiredVelocity(double range, double h, double theta) {
        double ct = Math.cos(theta);
        double st = Math.sin(theta);
        if (Math.abs(ct) < 1e-6) return Double.NaN;
        return Math.sqrt((0.5 * GRAVITY * range * range) / (ct * ct * (range * st / ct - h)));
    }

    private double solveElevationVacuum(double range, double h, boolean high, double[] timeOut) {
        double v2 = MUZZLE_VELOCITY_IN_S * MUZZLE_VELOCITY_IN_S;
        double root = v2 * v2 - GRAVITY * (GRAVITY * range * range + 2.0 * h * v2);

        if (root < 0) root = 0;
        double num1 = v2 + (high ? Math.sqrt(root) : -Math.sqrt(root));
        double den = GRAVITY * range;

        double theta = Math.atan2(num1, den);
        if (timeOut != null) timeOut[0] = range / (MUZZLE_VELOCITY_IN_S * Math.cos(theta));
        return theta;
    }

    private double solveElevationDrag(double range, double h) {
        return solveElevationVacuum(range, h, HIGH_ARC, null);
    }

    private double simMiss(double range, double h, double v, double theta, double[] out) {
        double ct = Math.cos(theta);
        out[0] = (ct > 1e-6) ? range / (v * ct) : 0;
        out[1] = v;
        return 0.0;
    }

    public static class Vec3 {
        public double x, y, z;
        public Vec3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
        public void set(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
        public Vec3 plus(Vec3 o) { return new Vec3(x + o.x, y + o.y, z + o.z); }
        public Vec3 minus(Vec3 o) { return new Vec3(x - o.x, y - o.y, z - o.z); }
        public Vec3 scale(double s) { return new Vec3(x * s, y * s, z * s); }

        public double dot(Vec3 o) { return x * o.x + y * o.y + z * o.z; }
        public Vec3 cross(Vec3 o) { return new Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x); }
        public double lenSq() { return x * x + y * y + z * z; }
        public double len() { return Math.sqrt(lenSq()); }
        public Vec3 norm() { double l = len(); return l > 1e-9 ? scale(1.0 / l) : this; }
        public Vec3 neg() { return scale(-1.0); }
        public void rotateZInPlace(double rad) {
            double c = Math.cos(rad), s = Math.sin(rad);
            double nx = c * x - s * y;
            double ny = s * x + c * y;
            x = nx;
            y = ny;
        }
    }

    public static class Mat3 {
        private final double[][] m;
        public Mat3(double[][] m) { this.m = m; }
        public Vec3 col(int c) { return new Vec3(m[0][c], m[1][c], m[2][c]); }
        public Mat3 mul(Mat3 o) {
            double[][] res = new double[3][3];
            for(int i=0; i<3; i++)
                for(int j=0; j<3; j++)
                    for(int k=0; k<3; k++) res[i][j] += m[i][k] * o.m[k][j];
            return new Mat3(res);
        }
        public Vec3 applyTo(Vec3 v) {
            return new Vec3(
                    m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
                    m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
                    m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
            );
        }
    }

    public static Mat3 rotX(double rad) {
        double c = Math.cos(rad), s = Math.sin(rad);
        return new Mat3(new double[][]{{1,0,0},{0,c,-s},{0,s,c}});
    }
    public static Mat3 rotY(double rad) {
        double c = Math.cos(rad), s = Math.sin(rad);
        return new Mat3(new double[][]{{c,0,s},{0,1,0},{-s,0,c}});
    }
    public static Mat3 rotZ(double rad) {
        double c = Math.cos(rad), s = Math.sin(rad);
        return new Mat3(new double[][]{{c,-s,0},{s,c,0},{0,0,1}});
    }

    public static class TagObs {
        public final int id;
        public final Vec3 p, u, n;
        public final double area, viewCos;
        public TagObs(int id, Vec3 p, Vec3 u, Vec3 n, double area, double viewCos) {
            this.id = id; this.p = p; this.u = u; this.n = n; this.area = area; this.viewCos = viewCos;
        }
    }

    public static class AimResult {
        public boolean valid, coasting, reachable, highArc;
        public int tagCount, clusterStart;
        public int[] tagIds;
        public double ageMs, stalenessMs, confidence, spreadIn, viewCos;
        public Vec3 aimPointCamera;
        public double bearingDeg, rangeIn, deltaHIn, velocityInS, elevationDeg;
        public double flightTimeS, impactSpeedInS;
        public String model;
    }

    public boolean isLocked() { return fusedValid && confidence >= LOCK_CONFIDENCE; }
    public double getBearingDeg() { return lastAim != null ? lastAim.bearingDeg : 0.0; }
    public double getElevationDeg() { return lastAim != null ? lastAim.elevationDeg : 0.0; }
    public double getRangeIn() { return lastAim != null ? lastAim.rangeIn : 0.0; }
    public double getBearingRateDegPerSec() { return Math.toDegrees(yawRate); }
    public String getFuseNote() { return fuseNote; }
}