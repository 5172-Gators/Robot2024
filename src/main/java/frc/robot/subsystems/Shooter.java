// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  /* Initialize Motors */
  CANSparkMax leftShooter;
  CANSparkMax rightShooter;

  CANSparkFlex kicker;

  DigitalInput sensor;
  
  public Shooter() {
    
    /* define + configure left shooter motor */
    leftShooter = new CANSparkMax(Constants.Shooter.leftMotorID, MotorType.kBrushless);
    leftShooter.setIdleMode(IdleMode.kCoast);

    /* define + configure right shooter motor */
    rightShooter = new CANSparkMax(Constants.Shooter.rightMotorID, MotorType.kBrushless);
    rightShooter.setIdleMode(IdleMode.kCoast);

    /* define + configure kicker motor */
    kicker = new CANSparkFlex(Constants.Shooter.kickerMotorID, MotorType.kBrushless);
    kicker.setIdleMode(IdleMode.kBrake);

    /* Set Inverted */
    leftShooter.setInverted(true);
    rightShooter.setInverted(false);
    kicker.setInverted(true);

    sensor = new DigitalInput(0);

  }

  public void setShooter(double speed, double rightSpeedRatio, double leftSpeedRatio) {
    // controls the shooter + kicker motors depending on the value of the beam break
   
    boolean sensorValue = getSensor();
      
      leftShooter.set(speed * leftSpeedRatio);
      rightShooter.set(speed * rightSpeedRatio);
    
  }

  public void setKicker(double speed, double kickerSpeedRatio){

    kicker.set(speed * kickerSpeedRatio);
    
  }

  public void robotWash(){
    // runs the wheels at a low speed for cleaning purposes
    
    leftShooter.set(1 * 0.05);
    rightShooter.set(1 * 0.05);

  }

  public void stopShooter(){
    // stops the shooter

     leftShooter.set(0);
     rightShooter.set(0);

  }

  public boolean getSensor(){

    return sensor.get();

  }

  public double getRightSpeed(){

    return rightShooter.get();

  }

  public double getLeftSpeed(){

    return leftShooter.get();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Sensor Value", getSensor());

  }
}

