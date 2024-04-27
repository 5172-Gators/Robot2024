// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class AimingTolerances {

    public double turretTol;
    public double pitchTol;
    public double leftTol;
    public double rightTol;

    public AimingTolerances(double turretTol, double pitchTol, double leftTol, double rightTol) {
        this.turretTol = turretTol;
        this.pitchTol = pitchTol;
        this.leftTol = leftTol;
        this.rightTol = rightTol;
    }

    public AimingTolerances(AimingTolerances oldTolerances, double scaleFactor) {
        this.turretTol = oldTolerances.turretTol * scaleFactor;
        this.pitchTol = oldTolerances.pitchTol * scaleFactor;
        this.leftTol = oldTolerances.leftTol * scaleFactor;
        this.rightTol = oldTolerances.rightTol * scaleFactor;
    }
}
