package org.firstinspires.ftc.teamcode.systems.pureP;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PurePursuitPath {

    private static final double SMALL_NUMBER = 1e-12;

    public enum HeadingMode {
        INDEPENDENT,
        TANGENT,
        INVERTED_TANGENT
    }

    private final Point[] path;
    private double lookaheadRadius;
    private int reacquireWindow;

    private int startIndex = 0;

    private double lastLookaheadX;
    private double lastLookaheadY;
    private boolean hasLastLookahead = false;

    private HeadingMode headingMode = HeadingMode.INDEPENDENT;

    private double independentHeadingValue = 0;

    public static class PurePursuitOutput {
        public double targetX;
        public double targetY;
        public double targetHeading;
        public boolean atPathEnd;
    }

    public PurePursuitPath(
            Point[] path,
            double lookaheadRadius,
            int reacquireWindow) {

        this.path = path;
        this.lookaheadRadius = lookaheadRadius;
        this.reacquireWindow = reacquireWindow;
    }

    public void setHeadingMode(HeadingMode mode) {
        this.headingMode = mode;
    }

    PurePursuitOutput purePursuitOutput = new PurePursuitOutput();


    public PurePursuitOutput update(double poseX, double poseY) {

        if (path == null || path.length < 2) {
            return purePursuitOutput;
        }

        double rSquared = lookaheadRadius * lookaheadRadius;
        int lastIndex = path.length - 1;

        Point end = path[lastIndex];
        double dxEnd = end.getX() - poseX;
        double dyEnd = end.getY() - poseY;

        if ((dxEnd * dxEnd) + (dyEnd * dyEnd) <= rSquared) {
            purePursuitOutput.atPathEnd = true;
            purePursuitOutput.targetX = end.getX();
            purePursuitOutput.targetY = end.getY();
            switch (headingMode) {
                case INDEPENDENT: purePursuitOutput.targetHeading = path[lastIndex].getHeading();
                    break;

                case TANGENT:
                    purePursuitOutput.targetHeading = computeHeading(lastIndex - 1);
                    break;

                case INVERTED_TANGENT:
                    purePursuitOutput.targetHeading = normalizeAngle(
                            computeHeading(lastIndex-1) + Math.PI);
                    break;
            }
            return purePursuitOutput;
        }

        boolean found = findIntersection(
                poseX, poseY,
                rSquared,
                startIndex,
                lastIndex,
                purePursuitOutput);

        if (!found) {
            int begin = Math.max(0, startIndex - reacquireWindow);
            int endIdx = Math.min(lastIndex, startIndex + reacquireWindow);

            found = findIntersection(
                    poseX, poseY,
                    rSquared,
                    begin,
                    endIdx,
                    purePursuitOutput);
        }

        if (!found) {

            if (hasLastLookahead) {
                purePursuitOutput.targetX = lastLookaheadX;
                purePursuitOutput.targetY = lastLookaheadY;
            } else {
                Point start = path[0];
                purePursuitOutput.targetX = start.getX();
                purePursuitOutput.targetY = start.getY();
            }

            switch (headingMode) {
                case INDEPENDENT:
                    purePursuitOutput.targetHeading = path[lastIndex].getHeading();
                    if (purePursuitOutput.targetHeading == 0) {
                        purePursuitOutput.targetHeading = Math.toRadians(90);
                    }
                    break;
                case TANGENT:
                    purePursuitOutput.targetHeading = computeHeading(startIndex);
                    break;
                case INVERTED_TANGENT:
                    purePursuitOutput.targetHeading = normalizeAngle(computeHeading(startIndex) + Math.PI);
                    break;
            }
        }

        if (found) {
            switch (headingMode) {
                case INDEPENDENT:
                    purePursuitOutput.targetHeading = path[lastIndex].getHeading();
                    if (purePursuitOutput.targetHeading == 0) {
                        purePursuitOutput.targetHeading = Math.toRadians(90); // FORCE IT
                    }
                    break;

                case TANGENT:
                    purePursuitOutput.targetHeading = computeHeading(startIndex);
                    break;

                case INVERTED_TANGENT:
                    purePursuitOutput.targetHeading = normalizeAngle(
                            computeHeading(startIndex) + Math.PI);
                    break;
            }
        }

        return purePursuitOutput;
    }

    private boolean findIntersection(
            double robotX,
            double robotY,
            double radiusSq,
            int begin,
            int endIndex,
            PurePursuitOutput purePursuitOutput) {

        int furthestSegment = -1;
        double furthestParametricT = -1.0;

        for (int i = begin; i < endIndex; i++) {

            Point p1 = path[i];
            Point p2 = path[i + 1];

            double x1 = p1.getX();
            double y1 = p1.getY();
            double x2 = p2.getX();
            double y2 = p2.getY();

            double dx = x2 - x1;
            double dy = y2 - y1;

            double fx = x1 - robotX;
            double fy = y1 - robotY;

            double a = dx * dx + dy * dy;
            if (a < SMALL_NUMBER) continue;

            double b = 2.0 * (fx * dx + fy * dy);
            double c = (fx * fx + fy * fy) - radiusSq;

            double discriminant = b * b - 4.0 * a * c;
            if (discriminant < 0.0) continue;

            double sqrtD = Math.sqrt(discriminant);
            double invDenom = 1.0 / (2.0 * a);

            double t1 = (-b - sqrtD) * invDenom;
            double t2 = (-b + sqrtD) * invDenom;

            if (t2 >= 0.0 && t2 <= 1.0) {
                if (i > furthestSegment || (i == furthestSegment && t2 > furthestParametricT)) {
                    furthestSegment = i;
                    furthestParametricT = t2;
                }
            } else if (t1 >= 0.0 && t1 <= 1.0) {
                if (i > furthestSegment || (i == furthestSegment && t1 > furthestParametricT)) {
                    furthestSegment = i;
                    furthestParametricT = t1;
                }
            }
        }

        if (furthestSegment < 0) return false;

        Point p1 = path[furthestSegment];
        Point p2 = path[furthestSegment + 1];

        purePursuitOutput.targetX = p1.getX() + furthestParametricT * (p2.getX() - p1.getX());
        purePursuitOutput.targetY = p1.getY() + furthestParametricT * (p2.getY() - p1.getY());

        if (furthestSegment >= startIndex) {
            startIndex = furthestSegment;
        }

        lastLookaheadX = purePursuitOutput.targetX;
        lastLookaheadY = purePursuitOutput.targetY;
        hasLastLookahead = true;

        return true;
    }

    private double computeHeading(int segmentIndex) {
        segmentIndex = Math.max(0, Math.min(segmentIndex, path.length - 2));

        Point p1 = path[segmentIndex];
        Point p2 = path[segmentIndex + 1];

        return Math.atan2(
                p2.getY() - p1.getY(),
                p2.getX() - p1.getX());
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public static class Point {
        private double x = 0, y = 0, heading = 0;
        public Point(double x, double y, double heading) {
            this.x = x; this.y = y; this.heading = heading;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getHeading() {
            return heading;
        }

        public void setX(double x) {
            this.x = x;
        }

        public void setY(double y) {
            this.y = y;
        }

        public void setHeading(double heading) {
            this.heading = heading;
        }
    }
}
