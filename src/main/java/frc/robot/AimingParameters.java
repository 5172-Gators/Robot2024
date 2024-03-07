package frc.robot;

/**
 * A container for storing robot aiming parameters.
 */
public class AimingParameters {
    private double pitchAngle;
    private double shooterRPMLeft;
    private double shooterRPMRight;

    public AimingParameters(double pitchAngle, double shooterRPMLeft, double shooterRPMRight) {
        this.pitchAngle = pitchAngle;
        this.shooterRPMLeft = shooterRPMLeft;
        this.shooterRPMRight = shooterRPMRight;
    }

    public double getPitchAngle() {
        return this.pitchAngle;
    }

    public double getShooterRPMLeft() {
        return this.shooterRPMLeft;
    }

    public double getShooterRPMRight() {
        return this.shooterRPMRight;
    }
}
