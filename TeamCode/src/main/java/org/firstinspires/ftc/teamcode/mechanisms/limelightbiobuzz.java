package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Limelight 3A BIOBUZZ hive-cell AprilTag cluster aim solver.
 *
 * <p>Uses all four 36h11 AprilTags of a BIOBUZZ HIVE CELL cluster (per-cell sticker) to compute
 * the shot: bearing (yaw) to the cell opening centre plus required launch elevation and muzzle
 * velocity, with latency compensation.</p>
 *
 * <h3>Frames</h3>
 * <ul>
 *   <li>Camera frame (Limelight): X right, Y down, Z forward (optical axis).</li>
 *   <li>Robot frame (this class): X forward, Y left, Z up. Mapping: r = (cz, -cx, -cy).</li>
 *   <li>Cluster frame (sticker): u = increasing tag ID (printed right, seen facing sticker),
 *       v = printed up (= n x u), n = sticker outward normal (toward camera).</li>
 * </ul>
 *
 * <h3>Field geometry (BIOBUZZ manual, Figures 9-15/9-16)</h3>
 * <ul>
 *   <li>Tag size 3.25 in, tag centres at u = -6.5, -2.75, +2.75, +6.5 in from cluster centre.</li>
 *   <li>Tag row centreline is 7.188 in from the mouth edge (9.938 reference-hole distance
 *       minus 2.75 row-to-holes).</li>
 *   <li>Cell opening 20 in wide x 14 in tall, 12 in deep; sticker on the bottom face.</li>
 *   <li>Aim point (opening centre) in cluster frame = (0, +7.188, -7.0) in:
 *       0 across (centred), +7.188 toward the mouth edge along the sticker,
 *       -7 in behind the sticker plane (half of the 14 in opening height).</li>
 * </ul>
 *
 * <h3>Competition behaviour</h3>
 * <ul>
 *   <li>Output filtering: constant-velocity filter on the fused aim point (smooth, no
 *       per-frame jitter), innovation gate rejects spikes (wrong-cell flicker, motion blur),
 *       and a coast window ({@link #COAST_MS}) keeps the turret command alive through brief
 *       occlusions instead of blinking to null.</li>
 *   <li>Gates: {@link #MAX_SPREAD_IN} rejects a fusion whose tags disagree, {@link #MAX_MEAS_AGE_MS}
 *       rejects stale results, {@link #MIN_VIEW_COS} rejects edge-on views.</li>
 *   <li>Cluster hysteresis: the previously tracked cluster is kept when counts tie, so the
 *       aim never hops cell to cell.</li>
 *   <li>Turret API: gate the turret on {@link #isLocked()}, feed forward with
 *       {@link #getBearingRateDegPerSec()}, read {@link #getBearingDeg()} /
 *       {@link #getElevationDeg()} / {@link #getRangeIn()}. Use {@link #computeAim()} for a
 *       free launch elevation, or {@link #computeAimFixedElevation(double)} to solve velocity
 *       at a locked barrel angle.</li>
 *   <li>Threading: call {@link #update()}, the compute methods and the getters from a single
 *       thread (the OpMode loop); the class is not synchronized.</li>
 * </ul>
 *
 * <h3>Calibration TODOs (tune on robot)</h3>
 * <ul>
 *   <li>{@link #MUZZLE_VELOCITY_IN_S} - measure with a chronograph / flag method.</li>
 *   <li>{@link #DRAG_K} - quadratic drag coefficient (1/in); raise until long shots drop.</li>
 *   <li>{@link #MUZZLE_OFFSET_X_IN} / {@link #MUZZLE_OFFSET_Y_IN} / {@link #MUZZLE_OFFSET_Z_IN}
 *       - muzzle position relative to the camera, robot axes (inches).</li>
 *   <li>{@link #CAM_MOUNT_ROLL_DEG} / {@link #CAM_MOUNT_PITCH_DEG} / {@link #CAM_MOUNT_YAW_DEG}
 *       - camera mount rotation (deg) if the camera is not square to robot.</li>
 *   <li>IMU yaw sign: verify turning left increases yaw; flip {@link #YAW_SIGN} if not.</li>
 * </ul>
 */
public class limelightbiobuzz {

    // ---------------------------------------------------------------------
    // BIOBUZZ field constants (from the game manual)
    // ---------------------------------------------------------------------

    /** Cluster tag ID ranges, in printed left-to-right order when facing the sticker. */
    public static final int[][] CLUSTERS = {
            {30, 31, 32, 33},   // RED SCORING (opposite audience)
            {34, 35, 36, 37},   // RED AUDIENCE
            {38, 39, 40, 41},   // BLUE AUDIENCE
            {42, 43, 44, 45},   // BLUE SCORING (opposite audience)
    };

    /** AprilTag printed size, inches (3.25 in square / 8.25 cm). */
    public static final double TAG_SIZE_IN = 3.25;

    /** Tag centre offsets along the sticker row (u), inches, indexed by (id - clusterStart). */
    public static final double[] TAG_U_IN = {-6.5, -2.75, 2.75, 6.5};

    /** Cell opening size, inches (20 wide x 14 tall, 12 deep). */
    public static final double CELL_OPEN_W_IN = 20.0;
    public static final double CELL_OPEN_H_IN = 14.0;
    public static final double CELL_DEPTH_IN = 12.0;

    /** Tag row centreline distance from the mouth edge along the sticker, inches (9.938 - 2.75). */
    public static final double TAG_ROW_FROM_MOUTH_IN = 7.188;

    /**
     * Cell opening centre (aim point) expressed in the cluster frame (u, v, n), inches.
     * u: centred across the opening -> 0.
     * v: opening edge is TAG_ROW_FROM_MOUTH toward the mouth edge from the row -> +7.188.
     * n: opening centre sits CELL_OPEN_H/2 behind the sticker plane -> -7.0.
     */
    public static final double AIM_U_IN = 0.0;
    public static final double AIM_V_IN = TAG_ROW_FROM_MOUTH_IN;
    public static final double AIM_N_IN = -(CELL_OPEN_H_IN / 2.0);

    // ---------------------------------------------------------------------
    // Physics constants
    // ---------------------------------------------------------------------

    /** Gravity, in/s^2 (32.174 ft/s^2). */
    public static final double GRAVITY = 386.088;
    /** Inches per meter (Limelight poses are meters). */
    public static final double IN_PER_M = 39.37007874;

    // ---------------------------------------------------------------------
    // Tunables (calibrate on the robot - see class javadoc)
    // ---------------------------------------------------------------------

    /** Estimated muzzle velocity, in/s. Placeholder - MEASURE THIS. */
    public double MUZZLE_VELOCITY_IN_S = 300.0;

    /** Quadratic drag coefficient, 1/in. 0 = vacuum. Placeholder - CALIBRATE THIS. */
    public double DRAG_K = 0.0008;

    /** Prefer the high arc. */
    public boolean HIGH_ARC = false;

    /** Refine the vacuum solution with the quadratic-drag model. */
    public boolean USE_DRAG_MODEL = true;

    /**
     * Muzzle position relative to the CAMERA, in robot axes (X fwd, Y left, Z up), inches.
     * The fused aim point is camera-relative (the camera is our origin), so this must be
     * muzzle minus camera. Example: muzzle 2 in right, 3 in above camera -> (0, -2, 3).
     */
    public double MUZZLE_OFFSET_X_IN = 0.0;
    public double MUZZLE_OFFSET_Y_IN = 0.0;
    public double MUZZLE_OFFSET_Z_IN = 0.0;

    /** Camera mount rotation relative to ideal (X right, Y down, Z fwd), degrees. 0 if square. */
    public double CAM_MOUNT_ROLL_DEG = 0.0;
    public double CAM_MOUNT_PITCH_DEG = 0.0;
    public double CAM_MOUNT_YAW_DEG = 0.0;

    /** Sign of IMU yaw vs right-handed Z-up (verify on robot: turn left, yaw should rise). */
    public double YAW_SIGN = 1.0;

    /** Latency compensation (aim-rotation while the result was in flight). */
    public boolean COMPENSATE_LATENCY = true;

    /** Reject observations viewing the sticker more than this far off-axis (cos of incidence). */
    public double MIN_VIEW_COS = 0.35;

    /** Per-tag cluster-centre residual (in) above which an observation is dropped (MAD floor). */
    public double OUTLIER_FLOOR_IN = 0.75;

    // --- competition output filtering / gating ---

    /** Position correction weight of the output filter (0..1). Lower = smoother. */
    public double FILTER_ALPHA = 0.35;

    /** Velocity correction weight of the output filter (0..1). Lower = smoother. */
    public double FILTER_BETA = 0.10;

    /** Innovation gate (in): a measurement further than this from the prediction is a spike. */
    public double FILTER_GATE_IN = 8.0;

    /** Keep predicting through this many ms of lost vision before declaring "lost". */
    public double COAST_MS = 300.0;

    /** Results older than this are not used to correct the filter. */
    public double MAX_MEAS_AGE_MS = 400.0;

    /** Reject a fused solution whose per-tag centre estimates disagree by more than this (in). */
    public double MAX_SPREAD_IN = 2.0;

    /** isLocked() requires at least this fused confidence. */
    public double LOCK_CONFIDENCE = 0.6;

    /**
     * Pipeline index holding the BIOBUZZ AprilTags config (web UI: pipeline 3).
     * The solver keeps the camera pinned to it and re-selects it if anything
     * (camera reboot, another OpMode, web UI edit) changes the active pipeline.
     * Call {@link #selectPipeline(int)} with -1 to stop managing it and follow whatever is active.
     */
    public static final int DEFAULT_PIPELINE_INDEX = 3;

    // ---------------------------------------------------------------------
    // Internal state
    // ---------------------------------------------------------------------

    private final Limelight3A limelight;
    private final IMU imu;

    private final ArrayList<TagObs> obs = new ArrayList<>();
    private final ArrayList<TagObs> accepted = new ArrayList<>();

    private boolean fusedValid = false;
    private Vec3 aimCam = new Vec3(0, 0, 0);      // aim point, camera-relative, robot axes, inches
    private Vec3 fusedU = new Vec3(1, 0, 0);
    private Vec3 fusedV = new Vec3(0, 1, 0);
    private Vec3 fusedN = new Vec3(0, 0, 1);
    private int clusterStart = -1;
    private int[] seenIds = new int[0];
    private double spreadIn = 0;      // fusion residual of per-tag aim points
    private double viewCosMean = 1;   // mean incidence cosine of accepted tags
    private double confidence = 0;

    private double ageMs = 0;
    private double stalenessMs = 0;
    private double pitchRad = 0;
    private double rollRad = 0;

    private double yawRate = 0;       // rad/s, tracked from IMU
    private long lastYawMs = -1;
    private double lastYawRad = 0;

    /** Robot velocity in robot axes (in/s) for translation compensation; set via setter. */
    private double robotVx = 0;
    private double robotVy = 0;

    private AimResult lastAim = null;

    private int managedPipeline = DEFAULT_PIPELINE_INDEX;
    private long lastPipelineSwitchMs = 0;
    private int lastPipelineIdx = -1;   // pipeline index reported by the latest result

    // --- fusion diagnostics (for telemetry) ---
    private String fuseNote = "no result yet";
    private int parsedCount = 0;        // cluster tags parsed this cycle
    private int usedCount = 0;          // tags that survived outlier rejection
    private int rejectedCount = 0;      // tags dropped by the MAD filter
    private Vec3 fusedCentre = new Vec3(0, 0, 0);   // triangulated cluster centre, robot axes

    // --- output filter (constant-velocity filter on the fused aim point) ---
    private Vec3 filtPos = null;            // filtered aim point, robot axes, inches
    private Vec3 filtVel = new Vec3(0, 0, 0);
    private boolean filterInit = false;
    private boolean coasting = false;
    private long filterPredictMs = -1;
    private long filterMeasMs = -1;      // last ACCEPTED correction
    private long lastFreshMs = -1;       // last cycle with a fresh fusion

    // ---------------------------------------------------------------------
    // Construction / lifecycle
    // ---------------------------------------------------------------------

    public limelightbiobuzz(HardwareMap hardwareMap) {
        this(hardwareMap, "limelight", null);
    }

    public limelightbiobuzz(HardwareMap hardwareMap, IMU imu) {
        this(hardwareMap, "limelight", imu);
    }

    public limelightbiobuzz(HardwareMap hardwareMap, String deviceName, IMU imu) {
        this.imu = imu;
        limelight = hardwareMap.get(Limelight3A.class, deviceName);
        limelight.setPollRateHz(50);
        limelight.start();
        selectPipeline(DEFAULT_PIPELINE_INDEX);   // pin to pipeline 3 (AprilTags config)
    }

    public void stop() {
        limelight.stop();
    }

    public void resume() {
        limelight.start();
    }

    /**
     * Select and pin the camera's active pipeline. Pass a value >= 0 to force that pipeline
     * (kept enforced by {@link #update()}), or -1 to leave the camera on whatever is active.
     */
    public void selectPipeline(int index) {
        managedPipeline = index;
        if (index >= 0) {
            limelight.pipelineSwitch(index);
            lastPipelineSwitchMs = System.currentTimeMillis();
        }
    }

    /** Pipeline index reported by the most recent result (-1 if none yet). */
    public int getPipelineIndex() {
        return lastPipelineIdx;
    }

    /** Optional: feed robot velocity (robot axes, in/s) for translation latency compensation. */
    public void setRobotVelocity(double vxInS, double vyInS) {
        robotVx = vxInS;
        robotVy = vyInS;
    }

    // ---------------------------------------------------------------------
    // Update - poll the Limelight and fuse the cluster
    // ---------------------------------------------------------------------

    /**
     * Poll the latest Limelight result, parse all visible BIOBUZZ cluster tags and fuse them
     * into one aim point. Call once per loop.
     *
     * @return true if at least one cluster tag was seen this cycle.
     */
    public boolean update() {
        boolean saw = updateInternal();
        runFilter();     // runs every cycle, even with no vision (predict/coast)
        return saw;
    }

    private boolean updateInternal() {
        LLResult res = limelight.getLatestResult();

        // IMU sampling every update, even with no vision, so yaw rate stays fresh.
        trackYawRate();

        // Keep the camera pinned to the AprilTags pipeline (3). If a reboot, the web UI,
        // or another client changed the active pipeline, switch back (throttled so a
        // stale frame can't cause a switch storm).
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

        // ---- result age: capture + targeting + parse + time since hub parse ----
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

        // ---- parse fiducials ----
        obs.clear();
        List<LLResultTypes.FiducialResult> fids = res.getFiducialResults();
        if (fids != null) {
            for (LLResultTypes.FiducialResult f : fids) {
                int id = f.getFiducialId();
                if (clusterStartOf(id) < 0) continue;   // not a BIOBUZZ cluster tag

                Pose3D pose = f.getTargetPoseCameraSpace();
                if (pose == null) continue;

                // t6t_cs: target pose in camera space, meters, YPR degrees.
                double px = pose.getPosition().x * IN_PER_M;
                double py = pose.getPosition().y * IN_PER_M;
                double pz = pose.getPosition().z * IN_PER_M;
                Vec3 p = new Vec3(px, py, pz);
                if (p.lenSq() < 1e-6) continue;

                YawPitchRollAngles ypr = pose.getOrientation();
                Mat3 rot = rotZ(Math.toRadians(ypr.getYaw(AngleUnit.DEGREES)))
                        .mul(rotY(Math.toRadians(ypr.getPitch(AngleUnit.DEGREES))))
                        .mul(rotX(Math.toRadians(ypr.getRoll(AngleUnit.DEGREES))));

                // Apply camera mount correction before mapping to robot axes.
                Mat3 rMount = rotZ(Math.toRadians(CAM_MOUNT_YAW_DEG))
                        .mul(rotY(Math.toRadians(CAM_MOUNT_PITCH_DEG)))
                        .mul(rotX(Math.toRadians(CAM_MOUNT_ROLL_DEG)));
                rot = rMount.mul(rot);

                // Tag frame axes (columns of R), mapped to ROBOT axes:
                // camera (X right, Y down, Z forward) -> robot (X fwd, Y left, Z up).
                Vec3 uCol = camToRobot(rot.col(0));     // pose X column
                Vec3 nCol = camToRobot(rot.col(2));     // pose Z column
                p = camToRobot(p);

                // n: sign-fix to point OUT of the sticker toward the camera.
                Vec3 toCam = p.scale(-1.0 / p.len());
                Vec3 n = (nCol.dot(toCam) >= 0) ? nCol : nCol.neg();

                // Incidence cosine (1 = head-on). Reject near edge-on poses (unstable).
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

    /**
     * Fuse all observations of the dominant cluster into one aim point.
     *
     * <p>Strategy: u = pose X column, sign-checked against the physical tag layout
     * (u_geo = normalize(p_higherID - p_lowerID)) so u always runs with increasing tag ID.
     * n = sign-fixed pose Z column (toward camera). v = n x u (printed up).
     * Per-tag implied cluster centre = tagPos - TAG_U*u; median + MAD outlier rejection;
     * area-weighted mean; aim = centre + (AIM_U, AIM_V, AIM_N) in (u, v, n).</p>
     */
    private void fuse() {
        // ---- dominant cluster (most tags, tie-break by total area) ----
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
        // Hysteresis: if the previously-fused cluster is still visible and within one tag
        // of the leader, stay with it - prevents cell-to-cell flicker when counts tie.
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

        // ---- consensus n (all sticker share one normal) ----
        Vec3 nSum = new Vec3(0, 0, 0);
        for (TagObs t : accepted) nSum = nSum.plus(t.n);
        if (nSum.lenSq() < 1e-9) {
            fusedValid = false;
            fuseNote = "degenerate sticker normal";
            return;
        }
        Vec3 n = nSum.norm();

        // ---- consensus u: average pose X columns after sign alignment ----
        // Align each column to the first observation, then verify/flip against u_geo
        // (lower-ID -> higher-ID direction) when >= 2 tags are visible.
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
                if (u.dot(uGeo) < 0) u = u.neg();   // increasing ID must run along +u
            }
        }

        // Re-orthonormalise (u, v, n) with v = n x u (right-handed, v = printed up).
        u = u.minus(n.scale(u.dot(n)));       // remove any n component
        if (u.lenSq() < 1e-9) {
            fusedValid = false;
            fuseNote = "degenerate row axis";
            return;
        }
        u = u.norm();
        Vec3 v = n.cross(u);

        // ---- per-tag implied cluster centre in camera frame ----
        int m = accepted.size();
        Vec3[] centres = new Vec3[m];
        for (int i = 0; i < m; i++) {
            TagObs t = accepted.get(i);
            int idx = t.id - bestStart;
            centres[i] = t.p.minus(u.scale(TAG_U_IN[idx]));
        }

        // ---- median + MAD outlier rejection (small n: component-wise median) ----
        double mx = medianCoord(centres, 0);
        double my = medianCoord(centres, 1);
        double mz = medianCoord(centres, 2);
        Vec3 med = new Vec3(mx, my, mz);

        double[] dev = new double[m];
        for (int i = 0; i < m; i++) dev[i] = centres[i].minus(med).len();
        double mad = medianOf(dev);
        double thr = Math.max(OUTLIER_FLOOR_IN, 3.0 * 1.4826 * mad);

        // ---- area-weighted mean of survivors ----
        Vec3 cSum = new Vec3(0, 0, 0);
        double wSum = 0;
        double spreadSq = 0;
        int used = 0;
        double viewSum = 0;
        for (int i = 0; i < m; i++) {
            if (dev[i] > thr) continue;
            double w = Math.max(1.0, accepted.get(i).area);   // area in px^2
            cSum = cSum.plus(centres[i].scale(w));
            wSum += w;
            spreadSq += dev[i] * dev[i];
            viewSum += accepted.get(i).viewCos;
            used++;
        }
        rejectedCount = m - used;
        if (used == 0 || wSum <= 0) {          // all rejected -> fall back to median
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

        // Competition gate: if the per-tag centre estimates disagree this much, the fuse
        // is wrong (bad pose, mixed cluster, motion blur) - better no shot than a bad one.
        if (accepted.size() > 1 && spreadIn > MAX_SPREAD_IN) {
            fusedValid = false;
            fuseNote = "spread " + spreadIn + " in too high";
            return;
        }

        // ---- aim point: cluster centre + fixed offset in cluster frame ----
        aimCam = centre
                .plus(u.scale(AIM_U_IN))
                .plus(v.scale(AIM_V_IN))
                .plus(n.scale(AIM_N_IN));

        fusedCentre = centre;
        fusedU = u;
        fusedV = v;
        fusedN = n;
        clusterStart = bestStart;
        seenIds = new int[accepted.size()];
        for (int i = 0; i < accepted.size(); i++) seenIds[i] = accepted.get(i).id;

        // ---- confidence: tag count, incidence, consistency ----
        double countScore = new double[]{0.45, 0.55, 0.75, 0.9, 1.0}[Math.min(4, accepted.size())];
        double spreadScore = Math.exp(-spreadIn / 2.0);
        confidence = Math.max(0, Math.min(1,
                countScore * (0.5 + 0.5 * viewCosMean) * spreadScore));

        usedCount = used;
        fuseNote = (rejectedCount > 0)
                ? ("ok - rejected " + rejectedCount + "/" + m)
                : ("ok - all " + m + " agree");
        fusedValid = true;
    }

    // ---------------------------------------------------------------------
    // Aim computation
    // ---------------------------------------------------------------------

    /**
     * Compute the shot for the current filtered aim point.
     *
     * <p>Uses the output-filter point (smooth, spike-gated, coasting-capable) whenever the
     * filter has acquired; falls back to the raw fusion on the very first frame.</p>
     *
     * @return the aim solution, or null if we have no usable cluster solution.
     */
    public AimResult computeAim() {
        return computeAimWith(false, Double.NaN);
    }

    /**
     * Compute the shot at a FIXED launch elevation (deg). The returned solution carries the
     * muzzle velocity required to reach the target at that angle (mechanisms with a
     * fixed-barrel shooter, or to compare flat vs lobbed options at match speed).
     *
     * @param elevationDeg fixed launch elevation above the gravity-aligned horizon, degrees
     * @return the aim solution with {@link AimResult#velocityInS} solved, or null if unusable.
     */
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

        // ---- latency compensation: robot rotated (and moved) while the result aged ----
        Vec3 src = filterInit ? filtPos : aimCam;
        Vec3 p = src;
        if (dt > 1e-4) {
            double dPsi = yawRate * dt * YAW_SIGN;
            p = p.rotZ(-dPsi);                     // fixed point, new robot frame
            p = p.minus(new Vec3(robotVx * dt, robotVy * dt, 0));
        }

        AimResult r = new AimResult();
        r.valid = true;
        r.coasting = filterInit && !fresh;
        r.tagCount = accepted.size();
        r.tagIds = seenIds.clone();
        r.clusterStart = clusterStart;
        r.ageMs = ageMs;
        r.stalenessMs = stalenessMs;
        r.confidence = confidence;
        r.spreadIn = spreadIn;
        r.viewCos = viewCosMean;
        r.aimPointCamera = p;                       // camera-relative, robot axes, inches

        // ---- launch origin = muzzle (relative to camera) ----
        Vec3 rel = p.minus(new Vec3(
                MUZZLE_OFFSET_X_IN, MUZZLE_OFFSET_Y_IN, MUZZLE_OFFSET_Z_IN));

        // ---- gravity-aligned frame: elevation and range about gravity, not the robot deck ----
        Vec3 g = rotY(pitchRad).mul(rotX(rollRad)).applyTo(rel);

        r.bearingDeg = Math.toDegrees(Math.atan2(g.y, g.x));
        r.rangeIn = Math.hypot(g.x, g.y);
        r.deltaHIn = g.z;

        // ---- solve launch ----
        double theta;
        if (fixedElevation) {
            theta = Math.toRadians(thetaDeg);
            double vReq = solveRequiredVelocity(r.rangeIn, r.deltaHIn, theta);
            r.reachable = Double.isFinite(vReq);
            r.velocityInS = Double.isFinite(vReq) ? vReq : MUZZLE_VELOCITY_IN_S;
            r.model = "FIXED";
        } else {
            r.velocityInS = MUZZLE_VELOCITY_IN_S;
            // Reachability: the vacuum discriminant bounds what ANY model can hit.
            double v2 = r.velocityInS * r.velocityInS;
            double disc = v2 * v2 - GRAVITY
                    * (GRAVITY * r.rangeIn * r.rangeIn + 2.0 * r.deltaHIn * v2);
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

        // ---- flight time & impact speed from the chosen model ----
        double[] out = new double[2];
        if (USE_DRAG_MODEL && !fixedElevation) {
            double miss = simMiss(r.rangeIn, r.deltaHIn, r.velocityInS, theta, out);
            if (miss <= -999) r.reachable = false;   // sim never got there
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
    // Output filter - smooth, spike-gated, survives brief vision dropouts
    // ---------------------------------------------------------------------

    /**
     * Constant-velocity filter on the fused aim point, run every {@link #update()} cycle.
     *
     * <p>Predicts forward each cycle, corrects with new measurements (FILTER_ALPHA /
     * FILTER_BETA), rejects spikes beyond FILTER_GATE_IN (wrong-cell flicker, motion blur),
     * and keeps predicting through COAST_MS of lost vision so the turret command never
     * blinks. After the coast window the filter goes "lost" and must re-acquire (snap).</p>
     */
    private void runFilter() {
        long now = System.currentTimeMillis();
        double dt = (filterPredictMs > 0)
                ? Math.min(0.1, (now - filterPredictMs) / 1000.0) : 0.0;
        filterPredictMs = now;

        if (filterInit) {
            filtPos = filtPos.plus(filtVel.scale(dt));      // predict to now
        }

        boolean fresh = fusedValid && ageMs <= MAX_MEAS_AGE_MS;
        if (fresh) {
            lastFreshMs = now;
            boolean needsSnap = !filterInit || filterMeasMs < 0
                    || (now - filterMeasMs) > COAST_MS;
            if (needsSnap) {
                // (re)acquire: snap to the measurement, forget old velocity.
                // Also covers a filter stuck behind a persistently-gated track.
                filtPos = aimCam;
                filtVel = new Vec3(0, 0, 0);
                filterInit = true;
                filterMeasMs = now;
            } else {
                Vec3 res = aimCam.minus(filtPos);
                if (res.len() <= FILTER_GATE_IN) {
                    // accepted: correct position and velocity
                    filtPos = filtPos.plus(res.scale(FILTER_ALPHA));
                    filtVel = filtVel.plus(res.scale(FILTER_BETA / Math.max(dt, 0.004)));
                    filterMeasMs = now;
                }
                // else: spike - keep predicting on the old track (does NOT refresh
                // filterMeasMs, so sustained gating falls back to a snap after COAST_MS)
            }
            coasting = false;
        } else {
            if (filterInit && lastFreshMs > 0 && (now - lastFreshMs) > COAST_MS) {
                filterInit = false;                        // lost for too long
                filtVel = new Vec3(0, 0, 0);
                coasting = false;
            } else {
                coasting = filterInit;                     // coasting inside the window
            }
        }
    }

    // ---------------------------------------------------------------------
    // Turret-facing API
    // ---------------------------------------------------------------------

    /** True while the output filter holds a usable solution (fresh or coasting). */
    public boolean isFilterValid() {
        return filterInit;
    }

    /** True while coasting through a vision dropout (still usable, within the COAST_MS window). */
    public boolean isCoasting() {
        return coasting;
    }

    /**
     * Locked = a fresh-enough solution above LOCK_CONFIDENCE. This is the flag the turret
     * controller should gate on; it never flickers (hysteresis comes from the filter and
     * the coast window).
     */
    public boolean isLocked() {
        return filterInit && confidence >= LOCK_CONFIDENCE && filterMeasMs > 0
                && (System.currentTimeMillis() - filterMeasMs) <= COAST_MS;
    }

    /** Filtered aim azimuth, degrees (+ = left of forward). NaN until acquired. */
    public double getBearingDeg() {
        return filterInit ? azDeg(filtPos) : Double.NaN;
    }

    /** Filtered aim elevation, degrees (+ = up). NaN until acquired. */
    public double getElevationDeg() {
        return filterInit ? elDeg(filtPos) : Double.NaN;
    }

    /** Filtered aim range, inches. NaN until acquired. */
    public double getRangeIn() {
        return filterInit ? filtPos.len() : Double.NaN;
    }

    /** Aim-point azimuth rate, deg/s - feed this forward to the turret controller. */
    public double getBearingRateDegPerSec() {
        if (!filterInit) return 0;
        double den = filtPos.x * filtPos.x + filtPos.y * filtPos.y;
        if (den < 1e-6) return 0;
        return Math.toDegrees((filtPos.x * filtVel.y - filtPos.y * filtVel.x) / den);
    }

    /** Distance between the raw fusion and the filtered point, in (how much smoothing is on). */
    public double getFilterDeviationIn() {
        return (filterInit && fusedValid) ? filtPos.minus(aimCam).len() : 0;
    }

    /**
     * Solve required muzzle velocity for a FIXED launch angle (mechanisms that lock elevation).
     * Vacuum closed form: v^2 = g d^2 / (2 cos^2(th) (d tan(th) - dh)).
     *
     * @return velocity in/s, or Double.NaN if no vacuum solution exists at this angle.
     */
    public double solveRequiredVelocity(double rangeIn, double deltaHIn, double thetaRad) {
        double denom = 2.0 * Math.cos(thetaRad) * Math.cos(thetaRad)
                * (rangeIn * Math.tan(thetaRad) - deltaHIn);
        if (rangeIn < 1e-6 || denom <= 1e-9) return Double.NaN;
        return Math.sqrt(GRAVITY * rangeIn * rangeIn / denom);
    }

    // ---------------------------------------------------------------------
    // Trajectory solvers
    // ---------------------------------------------------------------------

    /** Vacuum low/high arc. Returns radians; fills reachable flag in res when provided. */
    private double solveElevationVacuum(double d, double dh, boolean high, AimResult res) {
        if (d < 1e-6) return high ? Math.PI / 2 : 0;
        double v = MUZZLE_VELOCITY_IN_S;
        double v2 = v * v;
        double disc = v2 * v2 - GRAVITY * (GRAVITY * d * d + 2.0 * dh * v2);
        if (disc < 0) {
            if (res != null) res.reachable = false;
            return Math.PI / 4.0;   // best effort
        }
        double s = Math.sqrt(disc);
        double tanLow = (v2 - s) / (GRAVITY * d);
        double tanHigh = (v2 + s) / (GRAVITY * d);
        return high ? Math.atan(tanHigh) : Math.atan(tanLow);
    }

    /**
     * Quadratic-drag refinement: secant iteration on the simulated trajectory so it passes
     * through (d, dh). Starts from the vacuum solution (guaranteed bracketing behaviour in
     * practice; result is sanity-clamped to +-15 deg of the vacuum angle).
     */
    private double solveElevationDrag(double d, double dh) {
        double th0 = solveElevationVacuum(d, dh, HIGH_ARC, null);
        double m0 = simMiss(d, dh, MUZZLE_VELOCITY_IN_S, th0, null);
        if (Math.abs(m0) < 0.05) return th0;                 // already good enough

        double a = th0;
        double b = th0 + Math.toRadians(2.0);
        double fa = simMiss(d, dh, MUZZLE_VELOCITY_IN_S, a, null);
        double fb = simMiss(d, dh, MUZZLE_VELOCITY_IN_S, b, null);
        double best = a, bestAbs = Math.abs(fa);

        for (int i = 0; i < 12; i++) {
            if (Math.abs(fb - fa) < 1e-9) break;
            double c = b - fb * (b - a) / (fb - fa);
            if (!Double.isFinite(c)) break;
            // clamp to a sane elevation band
            c = Math.max(Math.toRadians(5), Math.min(Math.toRadians(85), c));
            if (Math.abs(c - th0) > Math.toRadians(15)) c = th0 + Math.signum(c - th0) * Math.toRadians(15);
            double fc = simMiss(d, dh, MUZZLE_VELOCITY_IN_S, c, null);
            if (Math.abs(fc) < bestAbs) {
                bestAbs = Math.abs(fc);
                best = c;
            }
            if (bestAbs < 0.05) break;
            a = b; fa = fb;
            b = c; fb = fc;
        }
        return bestAbs < Math.abs(m0) ? best : th0;
    }

    /**
     * Simulate the trajectory with quadratic drag (a = -k |v| v) and return the miss height
     * (z at x = d, minus dh). Out = [flight time, impact speed].
     */
    private double simMiss(double d, double dh, double v0, double theta, double[] out) {
        double dt = 0.002;
        double t = 0;
        double x = 0, z = 0;
        double vx = v0 * Math.cos(theta), vz = v0 * Math.sin(theta);

        if (d <= 1e-6) {
            if (out != null) { out[0] = 0; out[1] = v0; }
            return z - dh;
        }

        for (int i = 0; i < 5000; i++) {
            // midpoint step
            double sp = Math.hypot(vx, vz);
            double mx = vx + 0.5 * dt * (-DRAG_K * sp * vx);
            double mz = vz + 0.5 * dt * (-DRAG_K * sp * vz - GRAVITY);
            double sm = Math.hypot(mx, mz);
            double nx = x + dt * mx;
            double nz = z + dt * mz;
            double nvx = vx + dt * (-DRAG_K * sm * mx);
            double nvz = vz + dt * (-DRAG_K * sm * mz - GRAVITY);

            if (nx >= d) {
                double f = (d - x) / Math.max(nx - x, 1e-9);
                double zAt = z + f * (nz - z);
                if (out != null) {
                    out[0] = t + f * dt;
                    out[1] = Math.hypot(nvx, nvz);
                }
                return zAt - dh;
            }
            if (nz < dh - 1000) break;      // plunged far below target: hopeless
            x = nx; z = nz; vx = nvx; vz = nvz;
            t += dt;
            if (vx < 1.0 && z < 0) break;   // stalled short
        }
        if (out != null) {
            out[0] = t;
            out[1] = Math.hypot(vx, vz);
        }
        return -1000.0;                     // never reached d
    }

    // ---------------------------------------------------------------------
    // IMU yaw-rate tracking
    // ---------------------------------------------------------------------

    private void trackYawRate() {
        if (imu == null) return;
        long now = System.currentTimeMillis();
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) * YAW_SIGN;
        if (lastYawMs >= 0) {
            double dt = (now - lastYawMs) / 1000.0;
            if (dt > 1e-4 && dt < 0.5) {
                double dyaw = yaw - lastYawRad;
                // wrap to (-pi, pi]
                while (dyaw > Math.PI) dyaw -= 2 * Math.PI;
                while (dyaw < -Math.PI) dyaw += 2 * Math.PI;
                double rate = dyaw / dt;
                rate = Math.max(-10, Math.min(10, rate));    // clamp spikes
                yawRate = 0.6 * yawRate + 0.4 * rate;        // light smoothing
            }
        }
        lastYawMs = now;
        lastYawRad = yaw;
    }

    /** Current yaw rate estimate, rad/s (positive = turning left, before YAW_SIGN). */
    public double getYawRate() {
        return yawRate;
    }

    // ---------------------------------------------------------------------
    // Accessors
    // ---------------------------------------------------------------------

    public boolean isValid() {
        return fusedValid;
    }

    public double getAgeMs() {
        return ageMs;
    }

    public double getStalenessMs() {
        return stalenessMs;
    }

    public double getConfidence() {
        return confidence;
    }

    public int getTagCount() {
        return fusedValid ? accepted.size() : 0;
    }

    public int[] getSeenIds() {
        return seenIds.clone();
    }

    /** Fused aim point, camera-relative in robot axes, inches (not latency compensated). */
    public Vec3 getAimPointCamera() {
        return aimCam;
    }

    /** Triangulated cluster centre (robot axes, inches, relative to camera). */
    public Vec3 getTriangulatedCentre() {
        return fusedCentre;
    }

    /** Why the last fusion attempt failed (or "ok ..."). For telemetry/debug. */
    public String getFuseNote() {
        return fuseNote;
    }

    public Vec3 getU() {
        return fusedU;
    }

    public Vec3 getV() {
        return fusedV;
    }

    public Vec3 getN() {
        return fusedN;
    }

    public AimResult getLastAim() {
        return lastAim;
    }

    /** Multi-line debug summary for telemetry. */
    public String debugString() {
        if (!fusedValid) {
            return String.format(Locale.US, "BB: no cluster (age %.0f ms)", ageMs);
        }
        AimResult a = lastAim;
        StringBuilder sb = new StringBuilder();
        sb.append(String.format(Locale.US, "BB cluster %d  tags %d  conf %.2f%n",
                clusterStart, accepted.size(), confidence));
        sb.append(String.format(Locale.US, " ids:"));
        for (int id : seenIds) sb.append(' ').append(id);
        sb.append(String.format(Locale.US, "%n aim cam %.1f %.1f %.1f in%n",
                aimCam.x, aimCam.y, aimCam.z));
        sb.append(String.format(Locale.US, " spread %.2f in  viewCos %.2f  age %.0f ms%n",
                spreadIn, viewCosMean, ageMs));
        if (a != null) {
            sb.append(String.format(Locale.US,
                    " bearing %.1f deg  elev %.1f deg (%s)  v %.0f in/s%n",
                    a.bearingDeg, a.elevationDeg, a.model, a.velocityInS));
            sb.append(String.format(Locale.US,
                    " range %.1f in  dh %.1f in  TOF %.2f s  impact %.0f in/s%s",
                    a.rangeIn, a.deltaHIn, a.flightTimeS, a.impactSpeedInS,
                    a.reachable ? "" : "  (UNREACHABLE)"));
        }
        return sb.toString();
    }

    /**
     * Push full diagnostics to telemetry (testing/tuning).
     *
     * <p>Called every loop. Everything a turret-lock test needs: whether we have the cluster,
     * which tags, fusion quality, the turret bearing command, and the launch solution.</p>
     */
    public void addTelemetry(Telemetry t) {
        t.addLine("-- BioBuzz --");
        t.addData("pipeline", "camera %d / pinned %d", lastPipelineIdx, managedPipeline);
        t.addData("fuse", "%s  (parsed %d, used %d, rejected %d)",
                fuseNote, parsedCount, usedCount, rejectedCount);
        t.addData("latency", "age %.0f ms  stale %.0f ms  yawRate %.2f rad/s",
                ageMs, stalenessMs, yawRate);
        if (filterInit) {
            t.addData("lock", "%s%s  az %.1f  el %.1f  rng %.1f  rate %.1f deg/s",
                    isLocked() ? "LOCKED" : "weak",
                    coasting ? " COAST" : "",
                    getBearingDeg(), getElevationDeg(), getRangeIn(),
                    getBearingRateDegPerSec());
            t.addData("filter dev", "%.2f in (raw vs filtered)", getFilterDeviationIn());
        } else {
            t.addData("lock", "no fix (filter not acquired)");
        }

        // --- raw per-tag angles (robot axes): triangulation inputs. Shown even when
        // fusion fails, so you can see what the detector actually has to work with. ---
        for (TagObs o : obs) {
            t.addData("T" + o.id, "az %6.1f  el %5.1f  rng %5.1f  inc %4.1f  a %.0f",
                    azDeg(o.p), elDeg(o.p), o.p.len(),
                    Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, o.viewCos)))),
                    o.area);
        }

        if (fusedValid) {
            // --- triangulation result: each tag's pose (minus its known sticker offset)
            // independently points at the same cluster centre; these lines show where it
            // landed and how well the independent measurements agreed. ---
            StringBuilder ids = new StringBuilder();
            for (int id : seenIds) ids.append(id).append(' ');
            t.addData("cluster", "%d  tags %d  conf %.2f  ids %s",
                    clusterStart, accepted.size(), confidence, ids.toString().trim());
            t.addData("tri centre", "x %.1f  y %.1f  z %.1f",
                    fusedCentre.x, fusedCentre.y, fusedCentre.z);
            t.addData("tri spherical", "az %.1f  el %.1f  rng %.1f",
                    azDeg(fusedCentre), elDeg(fusedCentre), fusedCentre.len());
            t.addData("tri spread", "%.2f in (tag-to-tag agreement)", spreadIn);
            t.addData("sticker facing", "az %.1f  el %.1f  inc %.1f",
                    azDeg(fusedN), elDeg(fusedN),
                    Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, viewCosMean)))));
            t.addData("aim point", "x %.1f  y %.1f  z %.1f",
                    aimCam.x, aimCam.y, aimCam.z);
            t.addData("aim angles", "az %.1f  el %.1f  rng %.1f",
                    azDeg(aimCam), elDeg(aimCam), aimCam.len());
        } else {
            t.addData("cluster", "none (age %.0f ms)", ageMs);
        }

        // --- solved shot (turret commands) ---
        AimResult a = lastAim;
        if (a == null) return;
        t.addData("turretBearing", "%.1f deg%s", a.bearingDeg,
                a.coasting ? " (coast)" : "");
        t.addData("elevation", "%.1f deg (%s, %s)", a.elevationDeg, a.model,
                a.reachable ? "reachable" : "UNREACHABLE");
        t.addData("velocity", "%.0f in/s", a.velocityInS);
        t.addData("range/dh", "%.1f in / %.1f in", a.rangeIn, a.deltaHIn);
        t.addData("flight", "%.2f s  impact %.0f in/s", a.flightTimeS, a.impactSpeedInS);
    }

    /** Azimuth of a robot-frame vector, degrees (+ = left of forward). */
    private static double azDeg(Vec3 v) {
        return Math.toDegrees(Math.atan2(v.y, v.x));
    }

    /** Elevation of a robot-frame vector, degrees (+ = up). */
    private static double elDeg(Vec3 v) {
        return Math.toDegrees(Math.atan2(v.z, Math.hypot(v.x, v.y)));
    }

    // ---------------------------------------------------------------------
    // Small helpers
    // ---------------------------------------------------------------------

    /** Cluster start ID for a tag ID, or -1 if not a BIOBUZZ cluster tag. */
    public static int clusterStartOf(int id) {
        for (int[] cl : CLUSTERS) {
            if (id >= cl[0] && id <= cl[3]) return cl[0];
        }
        return -1;
    }

    private static double medianCoord(Vec3[] pts, int axis) {
        double[] vals = new double[pts.length];
        for (int i = 0; i < pts.length; i++) {
            vals[i] = axis == 0 ? pts[i].x : axis == 1 ? pts[i].y : pts[i].z;
        }
        return medianOf(vals);
    }

    private static double medianOf(double[] vals) {
        double[] c = vals.clone();
        java.util.Arrays.sort(c);
        int n = c.length;
        if (n == 0) return 0;
        return (n % 2 == 1) ? c[n / 2] : 0.5 * (c[n / 2 - 1] + c[n / 2]);
    }

    /** Camera axes (X right, Y down, Z forward) -> robot axes (X forward, Y left, Z up). */
    public static Vec3 camToRobot(Vec3 v) {
        return new Vec3(v.z, -v.x, -v.y);
    }

    // ---- rotation matrix helpers ----

    static Mat3 rotX(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new Mat3(new double[][]{{1, 0, 0}, {0, c, -s}, {0, s, c}});
    }

    static Mat3 rotY(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new Mat3(new double[][]{{c, 0, s}, {0, 1, 0}, {-s, 0, c}});
    }

    static Mat3 rotZ(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new Mat3(new double[][]{{c, -s, 0}, {s, c, 0}, {0, 0, 1}});
    }

    /** Minimal row-major 3x3 rotation matrix. */
    static class Mat3 {
        final double[][] m;

        Mat3(double[][] m) {
            this.m = m;
        }

        Mat3 mul(Mat3 o) {
            double[][] r = new double[3][3];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    r[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j];
                }
            }
            return new Mat3(r);
        }

        /** Column j of the matrix (a basis axis expressed in the parent frame). */
        Vec3 col(int j) {
            return new Vec3(m[0][j], m[1][j], m[2][j]);
        }

        /** Apply this rotation to a vector. */
        Vec3 applyTo(Vec3 v) {
            return new Vec3(
                    m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                    m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                    m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
        }
    }

    // ---------------------------------------------------------------------
    // Inner types
    // ---------------------------------------------------------------------

    /** One parsed tag observation (camera space, inches). */
    private static class TagObs {
        final int id;
        final Vec3 p;        // tag centre
        final Vec3 u;        // pose X column (raw)
        final Vec3 n;        // sign-fixed pose Z column (toward camera)
        final double area;   // target area (px^2)
        final double viewCos;

        TagObs(int id, Vec3 p, Vec3 u, Vec3 n, double area, double viewCos) {
            this.id = id;
            this.p = p;
            this.u = u;
            this.n = n;
            this.area = area;
            this.viewCos = viewCos;
        }
    }

    /** Result of {@link #computeAim()}. */
    public static class AimResult {
        public boolean valid = false;
        /** True when this solution came from the filter coasting through a vision dropout. */
        public boolean coasting = false;
        public boolean reachable = true;
        public int tagCount;
        public int clusterStart;
        public int[] tagIds = new int[0];
        public double confidence;
        public double spreadIn;
        public double viewCos;
        public double ageMs;
        public double stalenessMs;
        public String model = "";

        /** Aim point, robot axes, inches, relative to the camera, after latency compensation. */
        public Vec3 aimPointCamera;

        /** Bearing of the launch (about gravity), degrees, + = left of robot forward. */
        public double bearingDeg;
        /** Launch elevation above horizontal, degrees. */
        public double elevationDeg;
        /** Muzzle velocity in/s. */
        public double velocityInS;
        /** Horizontal launch range (muzzle to aim), inches. */
        public double rangeIn;
        /** Aim height minus muzzle height, inches. */
        public double deltaHIn;
        /** Flight time, seconds. */
        public double flightTimeS;
        /** Speed at the opening, in/s. */
        public double impactSpeedInS;
        public boolean highArc;
    }

    /**
     * Minimal 3D vector (inches). Immutable.
     */
    public static class Vec3 {
        public final double x, y, z;

        public Vec3(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public Vec3 plus(Vec3 o) {
            return new Vec3(x + o.x, y + o.y, z + o.z);
        }

        public Vec3 minus(Vec3 o) {
            return new Vec3(x - o.x, y - o.y, z - o.z);
        }

        public Vec3 scale(double s) {
            return new Vec3(x * s, y * s, z * s);
        }

        public Vec3 neg() {
            return new Vec3(-x, -y, -z);
        }

        public double dot(Vec3 o) {
            return x * o.x + y * o.y + z * o.z;
        }

        public Vec3 cross(Vec3 o) {
            return new Vec3(
                    y * o.z - z * o.y,
                    z * o.x - x * o.z,
                    x * o.y - y * o.x);
        }

        public double lenSq() {
            return x * x + y * y + z * z;
        }

        public double len() {
            return Math.sqrt(lenSq());
        }

        public Vec3 norm() {
            double l = len();
            return l < 1e-12 ? new Vec3(0, 0, 0) : scale(1.0 / l);
        }

        /** Rotate about the Z axis by a (radians). */
        public Vec3 rotZ(double a) {
            double c = Math.cos(a), s = Math.sin(a);
            return new Vec3(c * x - s * y, s * x + c * y, z);
        }

        /** Apply a 3x3 row-major matrix. */
        public Vec3 apply(double[][] m) {
            return new Vec3(
                    m[0][0] * x + m[0][1] * y + m[0][2] * z,
                    m[1][0] * x + m[1][1] * y + m[1][2] * z,
                    m[2][0] * x + m[2][1] * y + m[2][2] * z);
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "(%.2f, %.2f, %.2f)", x, y, z);
        }
    }
}
