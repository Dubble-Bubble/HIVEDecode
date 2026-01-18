package org.firstinspires.ftc.teamcode.systems.squid;

/**
 * The classic SquID controller. Operates off the control law of response being proportional to the (signed) square root of the error.
 */
public class SquIDController {

    /**
     * The proportional gain. This gain multiplies the signed square root of error.
     */
    private double p;

    public SquIDController(double p) {
        this.p = p;
    }

    /**
     * The function that computes the intended system response.
     * @param currentPosition The current state of the system.
     * @param targetPosition The target state of the system.
     * @return The system response.
     */
    public double calculate(double currentPosition, double targetPosition) {
        double error = targetPosition - currentPosition;
        return signedSqrt(error) * p;
    }

    /**
     * The function that computes the intended system response.
     * @param error The error of the system.
     * @return The system response.
     */
    public double calculate(double error) {
        return signedSqrt(error) * p;
    }

    /**
     * Signed square root function.
     * @return The square root of the magnitude of <code>val</code>, with the same sign as <code>val</code>.
     */
    private static double signedSqrt(double val) {
        return Math.signum(val) * Math.sqrt(Math.abs(val));
    }

    public void setP(double p) {
        this.p = p;
    }
}
