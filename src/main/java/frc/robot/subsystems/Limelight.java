// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants.LimelightPosition;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  NetworkTable m_limelightTable;

  public Limelight() {

    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    
    tx = m_limelightTable.getEntry("tx");
    ty = m_limelightTable.getEntry("ty");
    ta = m_limelightTable.getEntry("ta");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("tX", x);
    SmartDashboard.putNumber("tY", y);
    SmartDashboard.putNumber("tA", area);

  }

  public double getTX(){

    return tx.getDouble(0.0);

  }

  public double getTY(){

    return ty.getDouble(0.0);

  }

  public double getTA(){

    return ta.getDouble(0.0);
  }

  public boolean isInRange(LimelightPosition desiredPosition){

    /* 
      (probably) checks to see if the desired positions for shooting are the same or greater than the actual limelight positions.
       
      could be used during both auto and teleop for shooting, deploying climber, or for whatever else
       
      will definitely continue tuning this to fit with the actual positions + limelight values

    */

    // for now I'm just using it as a base for a basic auto-shooting command that will be tuned later

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    if (x >= desiredPosition.getTX()){
      
      return true;

    } else if (y >= desiredPosition.getTY()){

      return true;

    } else if (area >= desiredPosition.getTA()){

      return true;

    } else {

      return false;

    }


  }

}
