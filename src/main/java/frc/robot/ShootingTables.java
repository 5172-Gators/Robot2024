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
        m_pitchLUT.put(new InterpolatingDouble(1.88), new InterpolatingDouble(1.715667));
        m_pitchLUT.put(new InterpolatingDouble(2.45), new InterpolatingDouble(1.252));
        m_pitchLUT.put(new InterpolatingDouble(3.245), new InterpolatingDouble(0.9295));
        m_pitchLUT.put(new InterpolatingDouble(4.20), new InterpolatingDouble(0.70256));
        m_pitchLUT.put(new InterpolatingDouble(5.05), new InterpolatingDouble(0.5063));
       
        // Pitch angle LUT (floor distance estimate, left flywheel RPM)
        m_RPMLeftLUT.put(new InterpolatingDouble(1.88), new InterpolatingDouble(2810.0));
        m_RPMLeftLUT.put(new InterpolatingDouble(2.45), new InterpolatingDouble(3080.0));
        m_RPMLeftLUT.put(new InterpolatingDouble(3.245), new InterpolatingDouble(3470.0));
        m_RPMLeftLUT.put(new InterpolatingDouble(4.20), new InterpolatingDouble(3640.0));
        m_RPMLeftLUT.put(new InterpolatingDouble(5.05), new InterpolatingDouble(3930.0));
        
        // Pitch angle LUT (floor distance estimate, right flywheel RPM)
        m_RPMRightLUT.put(new InterpolatingDouble(1.88), new InterpolatingDouble(2710.0));
        m_RPMRightLUT.put(new InterpolatingDouble(2.45), new InterpolatingDouble(3080.0));
        m_RPMRightLUT.put(new InterpolatingDouble(3.245), new InterpolatingDouble(2810.0));
        m_RPMRightLUT.put(new InterpolatingDouble(4.20), new InterpolatingDouble(3040.0));
        m_RPMRightLUT.put(new InterpolatingDouble(5.05), new InterpolatingDouble(3300.0));

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
