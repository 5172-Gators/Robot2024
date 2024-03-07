// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  
  private NetworkTableEntry tx;
  private NetworkTableEntry ty; // y in degrees ()
  private NetworkTableEntry ta; // area
  private NetworkTableEntry tvert; // vertical distance of the rough bounding box
  private NetworkTableEntry tv; // target valid

  private double x;
  private double y;
  private double a;
  private double vert;
  private boolean target_valid;
  private int last_id;

  private double lastDist = 50;

  private NetworkTable m_limelightTable;

  private CamMode s_camMode = CamMode.Drive;

  IntegerSubscriber m_Tid;

  public enum CamMode { Vision, Drive };

  public Limelight(CamMode camMode) {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight-vision");

    m_Tid = m_limelightTable.getIntegerTopic("tid").subscribe(-1);
    
    tx = m_limelightTable.getEntry("tx");
    ty = m_limelightTable.getEntry("ty");
    ta = m_limelightTable.getEntry("ta");
    tvert = m_limelightTable.getEntry("tvert");
    tv = m_limelightTable.getEntry("tv");

    // allows access to apriltags in code
    s_camMode = camMode;

  }

  /**
   * @return most recent tx value in degrees
   */
  public double getX() {
    return this.x;
  }

  /**
   * @return most recent ty value in degrees
   */
  public double getY() {
    return this.y;
  }

  /**
   * @return most recent ta value in degrees
   */
  public double getA() {
    return this.a;
  }

  /**
   * Calculates the vertical height of the limelight from the floor based on the pitch mechanism angle
   * @param pitch_m pitch angle of the pitch mechanism in degrees;
   * @return the vertical height in inches
   */
  public double getLimelightHeightFromFloor(double pitch_m) {
    return Constants.Limelight.floorToPivot + Constants.Limelight.limelightRadius*
                Math.sin(Math.toRadians(pitch_m+Constants.Limelight.camToShooterFrameAngle));
  }

  public double getLimelightHorizontalDistanceFromPivot(double pitch_m) {
    return Constants.Limelight.limelightRadius*Math.cos(Math.toRadians(pitch_m+Constants.Limelight.camToPivotAngle));
  }

  public double getCameraPitch(double pitch_m) {
    return pitch_m - Constants.Limelight.camToShooterFrameAngle;
  }

  /**
   * Calculates the distance to the target given the pitch angle of the shooter/limelight if target present,
   * otherwise returns the last detected distance
   * @param cam_pitch angle of the limelight in degrees from the horizontal
   * @param pitch_m pitch angle of the pitch mechanism in degrees;
   * @return the distance in inches
   */
  public double getDist(double pitch_m, int tagID) {
    double q1; // theta
    double h1;
    double d1;
    double d2;
    double cam_pitch = getCameraPitch(pitch_m);
    var alliance = DriverStation.getAlliance();
    if ((alliance.get() == DriverStation.Alliance.Blue && tagID == 7) 
            || (alliance.get() == DriverStation.Alliance.Red && tagID == 4)) {
      q1 = Math.abs(cam_pitch + y);
      h1 = Constants.Limelight.speakerAprilTagCenterHeight-getLimelightHeightFromFloor(pitch_m);
      d1 = h1/Math.tan(Math.toRadians(q1));

      d2 = getLimelightHorizontalDistanceFromPivot(pitch_m);

      lastDist = d1+d2;
      return lastDist;
    }
    return lastDist;
  }

  /**
   * Checks to see if the desired april tag is selected
   * @param id
   * @return
   */
  public boolean targetInRange(int id) {
    // checks to see if the desired apriltag is in range
    return id == m_Tid.get();
  }

  /**
   * @return ID of the most recently selected apriltag
   */
  public int currentTarget() {
    // returns the current apriltag that the limelight is reading
    int currentTarget = (int) m_Tid.get();
    return currentTarget;

  }

  // public double getLatency() {
  //   // total latency = Capture latency + pipeline latency + posting latency + update latency
  //   // Limelight posts to networktables every 10 ms, so the posting latency should be 5 ms on average.
  //   // update latency is calculated by finding time since periodic method was last called.
  //   final double now = Timer.getFPGATimestamp();
  //   final double sinceLastUpdate = (now - m_outputs.timestamp) * 1000.0; // x 1000 to convert to ms.
  //   return Constants.LIMELIGHT_CAPTURE_LATENCY + m_outputs.latency + 5.0 + sinceLastUpdate;
  //   // Removed getTimeSinceLastUpdate() - Networktables don't report update unless
  //   // the value actually changes.
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  

    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    a = ta.getDouble(0.0);
    vert = tvert.getDouble(0.0);
    target_valid = tv.getBoolean(false);
    last_id = (int) m_Tid.get();

    if (s_camMode == CamMode.Vision) {
      // SmartDashboard.putNumber("tx", x);
      // SmartDashboard.putNumber("ty", y);
      // SmartDashboard.putNumber("ta", a);
      // SmartDashboard.putNumber("tvert", vert);
      // SmartDashboard.putBoolean("tv", target_valid);
      // SmartDashboard.putNumber("tid", last_id);

      SmartDashboard.getNumber("Current Target", currentTarget());
    }

  }

}
