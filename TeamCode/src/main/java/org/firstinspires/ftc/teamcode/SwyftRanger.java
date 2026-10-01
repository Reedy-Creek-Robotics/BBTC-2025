
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.LinkedList;
import java.util.Queue;

public class SwyftRanger {

    private final AnalogInput analogInput;

    // Sensor Specs: 0 - 3.3V mapped to 0 - 10 ft (120 inches / 304.8 cm)
    private double maxVoltage = 3.3;
    private double maxDistanceInches = 120.0;

    // Filter settings
    private final Queue<Double> readingBuffer = new LinkedList<>();
    private int windowSize = 5;

    public SwyftRanger(HardwareMap hardwareMap, String deviceName) {
        this.analogInput = hardwareMap.get(AnalogInput.class, deviceName);
    }

    /**
     * Reads the raw analog voltage directly from the Control/Expansion Hub port.
     */
    public double getRawVoltage() {
        return analogInput.getVoltage();
    }

    /**
     * Converts raw voltage to distance in inches using linear scaling:
     * Distance = (Voltage / V_max) * Distance_max
     */
    public double getDistanceInches() {
        double v = getRawVoltage();
        return (v / maxVoltage) * maxDistanceInches;
    }

    /**
     * Converts raw voltage to distance in centimeters.
     */
    public double getDistanceCm() {
        return getDistanceInches() * 2.54;
    }

    /**
     * Returns a noise-filtered distance in inches using a moving average window.
     */
    public double getFilteredDistanceInches() {
        double currentDistance = getDistanceInches();
        readingBuffer.add(currentDistance);

        while (readingBuffer.size() > windowSize) {
            readingBuffer.poll();
        }

        double sum = 0.0;
        for (double val : readingBuffer) {
            sum += val;
        }
        return sum / readingBuffer.size();
    }

    /**
     * Evaluates whether an object is within a target distance threshold.
     */
    public boolean isTargetDetected(double thresholdInches) {
        return getFilteredDistanceInches() <= thresholdInches;
    }

    /**
     * Configures the number of samples used for the moving average filter.
     */
    public void setFilterWindowSize(int size) {
        this.windowSize = Math.max(1, size);
        this.readingBuffer.clear();
    }

    /**
     * Sets custom linear calibration coefficients: d(V) = (V / vMax) * dMax
     */
    public void setCalibration(double maxVoltage, double maxDistanceInches) {
        this.maxVoltage = maxVoltage;
        this.maxDistanceInches = maxDistanceInches;
    }
}


