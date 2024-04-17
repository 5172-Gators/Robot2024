package frc.robot;

import frc.lib.util.InterpolatingDoubleTable;

public final class ShootingTables {
    private InterpolatingDoubleTable tbl;


    public ShootingTables() {
        // The 4 columns are distance, pitch angle, left rpm, and right rpm
        tbl = new InterpolatingDoubleTable(4);

        // First column (reference) is distance to goal (m), followed by pitch angle (deg) and left and right shooter speeds (RPM)
        tbl.addPoint(1.77, 47.8, 3110, 2720);
        tbl.addPoint(2.34, 41.8, 336, 2730);
        tbl.addPoint(3.01, 35.1, 3400, 2840);
        tbl.addPoint(3.76, 30.8, 3540, 2940);
        tbl.addPoint(4.65, 27.5, 3830, 3170);
        tbl.addPoint(5.66, 23.7, 4360, 3420);

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
