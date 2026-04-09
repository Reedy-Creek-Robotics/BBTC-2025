package org.firstinspires.ftc.teamcode.mechanisms;

/**
 * Thread-safe wrapper around {@link Camera} that runs vision processing
 * on a background thread at ~50Hz.
 *
 * The main OpMode loop reads cached values through synchronized getters,
 * so camera processing never slows down the drive loop.
 *
 * Usage:
 *   ThreadedCamera tc = new ThreadedCamera(new Camera(hardwareMap));
 *   tc.setPipelineRed();
 *   tc.start();           // Spawns background thread
 *   // ... in loop ...
 *   double tx = tc.getTx();  // Thread-safe read
 *   // ... on stop ...
 *   tc.stop();             // Clean shutdown
 *
 * SAFETY: Only the background thread calls camera.update().
 * The main thread only reads cached values through synchronized getters.
 */
public class ThreadedCamera {

    private final Camera camera;
    private final Object lock = new Object();
    private volatile boolean running = true;
    private Thread cameraThread;

    // Cached results (updated by background thread, read by main thread)
    private double  tx       = 0;
    private double  ty       = 0;
    private double  distance = -1;
    private double  area     = -1;
    private double  yaw      = 0;
    private int     tid      = -1;
    private boolean hasTarget = false;

    public ThreadedCamera(Camera camera) {
        this.camera = camera;
    }

    /** Spawn the background thread. Call once after initialization. */
    public void start() {
        running = true;
        cameraThread = new Thread(() -> {
            while (running) {
                try {
                    camera.update();

                    synchronized (lock) {
                        tx        = camera.getTx();
                        ty        = camera.getTy();
                        distance  = camera.getDistance();
                        area      = camera.getArea();
                        yaw       = camera.getYaw();
                        tid       = camera.getTid();
                        hasTarget = camera.hasTarget();
                    }

                    Thread.sleep(20); // ~50 Hz
                } catch (InterruptedException e) {
                    break; // Clean exit on interrupt
                } catch (Exception e) {
                    // Don't let camera errors crash the thread;
                    // main loop can check hasTarget() == false
                    synchronized (lock) {
                        hasTarget = false;
                    }
                }
            }
        });
        cameraThread.setDaemon(true);
        cameraThread.setName("CameraThread");
        cameraThread.start();
    }

    /** Stop the background thread. Call in your OpMode cleanup. */
    public void stop() {
        running = false;
        if (cameraThread != null) {
            cameraThread.interrupt();
        }
    }

    // --- Thread-safe getters (called from main thread) ---

    public double  getTx()       { synchronized (lock) { return tx; } }
    public double  getTy()       { synchronized (lock) { return ty; } }
    public double  getDistance()  { synchronized (lock) { return distance; } }
    public double  getArea()     { synchronized (lock) { return area; } }
    public double  getYaw()      { synchronized (lock) { return yaw; } }
    public int     getTid()      { synchronized (lock) { return tid; } }
    public boolean hasTarget()   { synchronized (lock) { return hasTarget; } }

    // --- Pipeline switching (safe to call from main thread) ---
    // Limelight handles pipeline switching internally

    public void setPipelineBlue()    { camera.setPipelineBlue(); }
    public void setPipelineRed()     { camera.setPipelineRed(); }
    public void setPipelineUseless() { camera.setPipelineUseless(); }
    public void setPipeline(int idx) { camera.setPipeline(idx); }
}
