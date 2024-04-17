package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;

public final class ShootingTables {
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> m_pitchLUT;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> m_RPMLeftLUT;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> m_RPMRightLUT;


    public ShootingTables() {
        m_pitchLUT = new InterpolatingTreeMap<>();
        m_RPMLeftLUT = new InterpolatingTreeMap<>();
        m_RPMRightLUT = new InterpolatingTreeMap<>();


        // Pitch angle LUT (floor distance estimate, pitch angle)
        addPoint(m_pitchLUT,1.77, 47.8);
        addPoint(m_pitchLUT,2.34, 41.8);
        addPoint(m_pitchLUT,3.01, 35.1);
        addPoint(m_pitchLUT,3.76, 30.8);
        addPoint(m_pitchLUT,4.65, 26.0); //27.5
        addPoint(m_pitchLUT,5.66, 23.7);
        addPoint(m_pitchLUT,7.85, 47.4);
        addPoint(m_pitchLUT,9.36, 46.4);
        addPoint(m_pitchLUT,10.22, 43.1);
       
        // Pitch angle LUT (floor distance estimate, left flywheel RPM)
        addPoint(m_RPMLeftLUT,1.77, 3110);
        addPoint(m_RPMLeftLUT,2.34, 3360);
        addPoint(m_RPMLeftLUT,3.01, 3400);
        addPoint(m_RPMLeftLUT,3.76, 3540);
        addPoint(m_RPMLeftLUT,4.65, 3830);
        addPoint(m_RPMLeftLUT,5.66, 4360);
        addPoint(m_RPMLeftLUT,7.85, 2780);
        addPoint(m_RPMLeftLUT,9.36, 3122);
        addPoint(m_RPMLeftLUT,10.22, 3560);
        
        // Pitch angle LUT (floor distance estimate, right flywheel RPM)
        addPoint(m_RPMRightLUT,1.77, 2720);
        addPoint(m_RPMRightLUT,2.34, 2730);
        addPoint(m_RPMRightLUT,3.01, 2840);
        addPoint(m_RPMRightLUT,3.76, 2940);
        addPoint(m_RPMRightLUT,4.65, 3170);
        addPoint(m_RPMRightLUT,5.66, 3420);
        addPoint(m_RPMRightLUT,7.85, 2390);
        addPoint(m_RPMRightLUT,9.36, 2540);
        addPoint(m_RPMRightLUT,10.22, 2630);

    }

    public double getPitch(double dist) {
        return m_pitchLUT.getInterpolated(new InterpolatingDouble(dist)).value;
    }

    public double getLeftRPM(double dist) {
        return m_RPMLeftLUT.getInterpolated(new InterpolatingDouble(dist)).value;
    }

    public double getRightRPM(double dist) {
        return m_RPMRightLUT.getInterpolated(new InterpolatingDouble(dist)).value;
    }

    private void addPoint(InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> map, double x, double y) {
        map.put(new InterpolatingDouble(x), new InterpolatingDouble(y));
    }

    public AimingParameters getAimingParams(double dist) {
        // SmartDashboard.putNumber("requestedPitch", this.getPitch(dist));
        // SmartDashboard.putNumber("requestedLeftRPM", this.getLeftRPM(dist));
        // SmartDashboard.putNumber("requestedRightRPM", this.getRightRPM(dist));
        
        return new AimingParameters(this.getPitch(dist), this.getLeftRPM(dist), this.getRightRPM(dist));
    }
    
}
