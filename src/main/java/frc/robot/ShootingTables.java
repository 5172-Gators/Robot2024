package frc.robot;

import frc.lib.util.InterpolatingDoubleTable;

public final class ShootingTables extends ShooterInterpolatingDoubleTable {
    private InterpolatingDoubleTable tbl;


    public ShootingTables() {
        // The 4 columns are distance, pitch angle, left rpm, and right rpm
        super();
        tbl = super.tbl;
        
        tbl.addPoint(1.77, 50.0, 3131, 2719);
        tbl.addPoint(2.34, 46.48, 3353, 2716);
        tbl.addPoint(3.01, 38.78, 3440, 2815);
        tbl.addPoint(3.79, 34.28, 3599, 2995);
        tbl.addPoint(4.60, 30.48, 3868, 3182);
        tbl.addPoint(5.69, 25.07, 4432, 3134);

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
