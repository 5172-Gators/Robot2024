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
        m_pitchLUT.put(new InterpolatingDouble(1.64), new InterpolatingDouble(2.04));
        m_pitchLUT.put(new InterpolatingDouble(2.03), new InterpolatingDouble(1.68));
        m_pitchLUT.put(new InterpolatingDouble(2.46), new InterpolatingDouble(1.33));
        m_pitchLUT.put(new InterpolatingDouble(2.94), new InterpolatingDouble(1.06));
        m_pitchLUT.put(new InterpolatingDouble(3.77), new InterpolatingDouble(0.74));
        m_pitchLUT.put(new InterpolatingDouble(4.39), new InterpolatingDouble(0.56));
        m_pitchLUT.put(new InterpolatingDouble(4.87), new InterpolatingDouble(0.38));
       
        // Pitch angle LUT (floor distance estimate, left flywheel RPM)
        m_RPMLeftLUT.put(new InterpolatingDouble(1.64), new InterpolatingDouble(2770.0));
        m_RPMLeftLUT.put(new InterpolatingDouble(2.03), new InterpolatingDouble(2770.0));
        m_RPMLeftLUT.put(new InterpolatingDouble(2.46), new InterpolatingDouble(3310.0));
        m_RPMLeftLUT.put(new InterpolatingDouble(2.94), new InterpolatingDouble(3700.0));
        m_RPMLeftLUT.put(new InterpolatingDouble(3.77), new InterpolatingDouble(3700.0));
        m_RPMLeftLUT.put(new InterpolatingDouble(4.39), new InterpolatingDouble(4000.0));
        m_RPMLeftLUT.put(new InterpolatingDouble(4.87), new InterpolatingDouble(4440.0));
        
        // Pitch angle LUT (floor distance estimate, right flywheel RPM)
        m_RPMRightLUT.put(new InterpolatingDouble(1.64), new InterpolatingDouble(2800.0));
        m_RPMRightLUT.put(new InterpolatingDouble(2.03), new InterpolatingDouble(2800.0));
        m_RPMRightLUT.put(new InterpolatingDouble(2.46), new InterpolatingDouble(3090.0));
        m_RPMRightLUT.put(new InterpolatingDouble(2.94), new InterpolatingDouble(3160.0));
        m_RPMRightLUT.put(new InterpolatingDouble(3.77), new InterpolatingDouble(3330.0));
        m_RPMRightLUT.put(new InterpolatingDouble(4.39), new InterpolatingDouble(3450.0));
        m_RPMRightLUT.put(new InterpolatingDouble(4.87), new InterpolatingDouble(3730.0));

           // Pitch angle LUT (floor distance estimate, pitch angle)
    //     m_pitchLUT.put(new InterpolatingDouble(41.5), new InterpolatingDouble(2.3));
    //     m_pitchLUT.put(new InterpolatingDouble(69.5), new InterpolatingDouble(1.5));
    //     m_pitchLUT.put(new InterpolatingDouble(90.0), new InterpolatingDouble(1.0));
    //     m_pitchLUT.put(new InterpolatingDouble(107.46), new InterpolatingDouble(0.64));
    //    // m_pitchLUT.put(new InterpolatingDouble(107.46), new InterpolatingDouble(0.4331));
       
    //     // Pitch angle LUT (floor distance estimate, left flywheel RPM)
    //     m_RPMLeftLUT.put(new InterpolatingDouble(41.5), new InterpolatingDouble(3000.0));
    //     m_RPMLeftLUT.put(new InterpolatingDouble(69.5), new InterpolatingDouble(3000.0));
    //     m_RPMLeftLUT.put(new InterpolatingDouble(90.0), new InterpolatingDouble(3000.0));
    //     m_RPMLeftLUT.put(new InterpolatingDouble(107.46), new InterpolatingDouble(4000.0));
    //     //m_RPMLeftLUT.put(new InterpolatingDouble(107.46), new InterpolatingDouble(1800.0));
        
    //     // Pitch angle LUT (floor distance estimate, right flywheel RPM)
    //     m_RPMRightLUT.put(new InterpolatingDouble(41.5), new InterpolatingDouble(3000.0));
    //     m_RPMRightLUT.put(new InterpolatingDouble(69.5), new InterpolatingDouble(3000.0));
    //     m_RPMRightLUT.put(new InterpolatingDouble(90.0), new InterpolatingDouble(3000.0));
    //     m_RPMRightLUT.put(new InterpolatingDouble(107.46), new InterpolatingDouble(3500.0));
       // m_RPMRightLUT.put(new InterpolatingDouble(107.46), new InterpolatingDouble(3000.0));

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

    public AimingParameters getAimingParams(double dist) {
        SmartDashboard.putNumber("requestedPitch", this.getPitch(dist));
        // SmartDashboard.putNumber("requestedLeftRPM", this.getLeftRPM(dist));
        // SmartDashboard.putNumber("requestedRightRPM", this.getRightRPM(dist));
        
        return new AimingParameters(this.getPitch(dist), this.getLeftRPM(dist), this.getRightRPM(dist));
    }
    
}
