// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.util.InterpolatingDoubleTable;

/** Add your docs here. */
public class ShooterInterpolatingDoubleTable {
    public InterpolatingDoubleTable tbl;

    public ShooterInterpolatingDoubleTable() {
        tbl = new InterpolatingDoubleTable(4);
    }

    public double getPitch(double dist) {
        return tbl.get(dist, 0);
    }

    public double getLeftRPM(double dist) {
        return tbl.get(dist, 1);
    }

    public double getRightRPM(double dist) {
        return tbl.get(dist, 2);
    }

    public AimingParameters getAimingParams(double dist) {
        return new AimingParameters(this.getPitch(dist), this.getLeftRPM(dist), this.getRightRPM(dist));
    }

}
