package org.firstinspires.ftc.teamcode.systems;

public class LoopTimer {

    private long lastTimeNs = 0;
    private double loopPeriodMs = 0.0;

    public void update() {
        long now = System.nanoTime();

        if (lastTimeNs != 0) {
            loopPeriodMs = (now - lastTimeNs) / 1_000_000.0; // ns → ms
        }

        lastTimeNs = now;
    }

    public double getLoopPeriodMs() {
        return loopPeriodMs;
    }
    public double getLoopHz() {
        return loopPeriodMs > 0.0 ? 1000.0 / loopPeriodMs : 0.0;
    }
    public String isSlowerThan(double maxPeriodMs) {
        String returnVal = loopPeriodMs > maxPeriodMs ?  "Yes" : "No";
        return returnVal;
    }
}
