// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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

  RelativeEncoder rightShooterEncoder;
  RelativeEncoder leftShooterEncoder;
  RelativeEncoder kickerEncoder;

  DigitalInput kickerSensor;
  DigitalInput shooterSensor;

  SparkPIDController rightShooterPID;
  SparkPIDController leftShooterPID;
  SparkPIDController kickerPID;

  double leftSetpoint;
  double rightSetpoint;
  double kickerSetpoint;
  
  public Shooter() {
    
    /* define + configure left shooter motor */
    leftShooter = new CANSparkMax(Constants.Shooter.leftMotorID, MotorType.kBrushless);
    leftShooter.setIdleMode(IdleMode.kCoast);
    leftShooterEncoder = leftShooter.getEncoder();
    leftShooterPID = leftShooter.getPIDController();

    /* define + configure right shooter motor */
    rightShooter = new CANSparkMax(Constants.Shooter.rightMotorID, MotorType.kBrushless);
    rightShooter.setIdleMode(IdleMode.kCoast);
    rightShooterEncoder = rightShooter.getEncoder();
    rightShooterPID = rightShooter.getPIDController();

    /* configure kicker motor */
    kicker = new CANSparkFlex(Constants.Kicker.kickerMotorID, MotorType.kBrushless);
    kicker.setIdleMode(IdleMode.kCoast);
    kickerEncoder = kicker.getEncoder();
    kickerPID = kicker.getPIDController();

    /* Set Inverted */
    leftShooter.setInverted(true);
    rightShooter.setInverted(false);
    kicker.setInverted(true);

    /* Beam Break Sensors */
    kickerSensor = new DigitalInput(9);
    shooterSensor = new DigitalInput(1);

    /* Right Side PID */
    rightShooterPID.setP(Constants.Shooter.right_kP);
    rightShooterPID.setI(Constants.Shooter.right_kI);
    rightShooterPID.setD(Constants.Shooter.right_kD);
    rightShooterPID.setFF(Constants.Shooter.right_kFF);
    rightShooterPID.setIZone(Constants.Shooter.right_IZone);
    rightShooterPID.setOutputRange(Constants.Shooter.right_minOutput, Constants.Shooter.right_maxOutput);

    /* Left Side PID */
    leftShooterPID.setP(Constants.Shooter.left_kP);
    leftShooterPID.setI(Constants.Shooter.left_kI);
    leftShooterPID.setD(Constants.Shooter.left_kD);
    leftShooterPID.setFF(Constants.Shooter.left_kFF);
    leftShooterPID.setIZone(Constants.Shooter.left_IZone);
    leftShooterPID.setOutputRange(Constants.Shooter.left_minOutput, Constants.Shooter.left_maxOutput);

    /* Kicker PID */
    kickerPID.setP(Constants.Shooter.kicker_kP);
    kickerPID.setI(Constants.Shooter.kicker_kI);
    kickerPID.setD(Constants.Shooter.kicker_kD);
    kickerPID.setFF(Constants.Shooter.kicker_kFF);
    kickerPID.setOutputRange(Constants.Shooter.kicker_minOutput, Constants.Shooter.kicker_maxOutput);
    kickerPID.setIZone(Constants.Shooter.kicker_IZone);

  }

  public void setShooterRPM(double rightSpeed, double leftSpeed) {
    leftSetpoint = leftSpeed;
    rightSetpoint = rightSpeed;
    rightShooterPID.setReference(rightSpeed, CANSparkFlex.ControlType.kVelocity);
    leftShooterPID.setReference(leftSpeed, CANSparkFlex.ControlType.kVelocity);
    
  }

  public double getRightShooterRPM() {
    return rightShooterEncoder.getVelocity();
  }

  public double getLeftShooterRPM() {
    return leftShooterEncoder.getVelocity();
  }

  public double getKickerRPM() {
    return kickerEncoder.getVelocity();
  }

  public void setKickerRPM(double rpm){
    this.kickerSetpoint = rpm;
    kickerPID.setReference(rpm, CANSparkFlex.ControlType.kVelocity);
  }

  public void robotWash(){
    // runs the wheels at a low speed for cleaning purposes
    
    leftShooter.set(1 * 0.05);
    rightShooter.set(1 * 0.05);

  }

  public void stopShooter() {
    // stops the shooter

    leftShooter.set(0);
    rightShooter.set(0);

  }

  public void stopKicker() {
    kicker.set(0);
  }

  public boolean getKickerSensor(){

    return kickerSensor.get();

  }

  public boolean getShooterSensor(){

    return shooterSensor.get();

  }

  public boolean shooterIsReady() {
    if ((Math.abs(this.getLeftShooterRPM() - this.leftSetpoint) <= Constants.Shooter.left_allowableError) 
        && (Math.abs(this.getRightShooterRPM() - this.rightSetpoint) <= Constants.Shooter.right_allowableError)) {

      return true;

    } else {

      return false;

    }
  }

  public boolean kickerIsReady() {
    if (Math.abs(this.getKickerRPM() - this.kickerSetpoint) <= Constants.Shooter.kicker_allowableError) {

      return true;

    } else {

      return false;

    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Kicker Sensor Value", getKickerSensor());
    SmartDashboard.putBoolean("Shooter Sensor Value", getShooterSensor());

    SmartDashboard.putNumber("Right Side Speed", rightShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Left Side Speed", leftShooterEncoder.getVelocity());
    // SmartDashboard.putNumber("Kicker RPM", getKickerRPM());
    SmartDashboard.putBoolean("Shooter Ready", shooterIsReady());

  }
}

