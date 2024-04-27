package frc.robot;

import frc.lib.util.InterpolatingDoubleTable;

public final class LobTables extends ShooterInterpolatingDoubleTable {
    private InterpolatingDoubleTable tbl;


    public LobTables() {
        // The 4 columns are distance, pitch angle, left rpm, and right rpm
        super();
        tbl = super.tbl;

        // First column (reference) is distance to goal (m), followed by pitch angle (deg) and left and right shooter speeds (RPM)
        tbl.addPoint(7.85, 47.4, 2780, 2390);
        tbl.addPoint(9.36, 46.4, 3122, 2540);
        tbl.addPoint(10.22, 43.1, 3560, 2630);

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
