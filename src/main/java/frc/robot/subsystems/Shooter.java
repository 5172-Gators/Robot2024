// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
  CANSparkFlex leftShooter;
  CANSparkFlex rightShooter;

  RelativeEncoder rightShooterEncoder;
  RelativeEncoder leftShooterEncoder;

  DigitalInput kickerSensor;
  DigitalInput shooterSensor;

  SparkPIDController rightShooterPID;
  SparkPIDController leftShooterPID;

  double leftSetpoint;
  double rightSetpoint;
  double kickerSetpoint;

  Debouncer shooterDebounce = new Debouncer(0.1, DebounceType.kRising);
  Debouncer kickerDebounce = new Debouncer(0.1, DebounceType.kRising);
  Debouncer kickerBeamBreakDebouncer = new Debouncer(0.01, DebounceType.kBoth);
  Debouncer shooterBeamBreakDebouncer = new Debouncer(0.01, DebounceType.kBoth);
  
  public Shooter() {
    
    /* define + configure left shooter motor */
    leftShooter = new CANSparkFlex(Constants.Shooter.leftMotorID, MotorType.kBrushless);
    leftShooter.setIdleMode(IdleMode.kCoast);
    leftShooterEncoder = leftShooter.getEncoder();
    leftShooterPID = leftShooter.getPIDController();

    /* define + configure right shooter motor */
    rightShooter = new CANSparkFlex(Constants.Shooter.rightMotorID, MotorType.kBrushless);
    rightShooter.setIdleMode(IdleMode.kCoast);
    rightShooterEncoder = rightShooter.getEncoder();
    rightShooterPID = rightShooter.getPIDController();

    /* Set Inverted */
    leftShooter.setInverted(false);
    rightShooter.setInverted(true);

    /* Beam Break Sensors */
    kickerSensor = new DigitalInput(0);
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

  }

  public void setShooterSpeed(double rightSpeed, double leftSpeed) {
    leftShooter.set(leftSpeed);
    rightShooter.set(rightSpeed);
  }

  public void setShooterVoltage(double rightVoltage, double leftVoltage) {
    leftShooterPID.setReference(leftVoltage, CANSparkFlex.ControlType.kVoltage);
    rightShooterPID.setReference(rightVoltage, CANSparkFlex.ControlType.kVoltage);
  }

  public void setShooterRPM(double rightRPM, double leftRPM) {
    leftSetpoint = leftRPM;
    rightSetpoint = rightRPM;
    rightShooterPID.setReference(rightRPM, CANSparkFlex.ControlType.kVelocity);
    leftShooterPID.setReference(leftRPM, CANSparkFlex.ControlType.kVelocity);
    
  }

  public double getRightShooterRPM() {
    return rightShooterEncoder.getVelocity();
  }

  public double getLeftShooterRPM() {
    return leftShooterEncoder.getVelocity();
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

  public boolean getKickerSensor(){
    return kickerSensor.get();
    // return kickerBeamBreakDebouncer.calculate(kickerSensor.get());

  }

  public boolean getShooterSensor(){
    return shooterSensor.get();
    // return shooterBeamBreakDebouncer.calculate(shooterSensor.get());

  }

  public boolean shooterIsReady() {
    double absErrorLeft = Math.abs(this.getLeftShooterRPM() - this.leftSetpoint);
    double absErrorRight = Math.abs(this.getRightShooterRPM() - this.rightSetpoint);
    if (shooterDebounce.calculate(absErrorLeft <= Constants.Shooter.left_allowableError 
        && absErrorRight <= Constants.Shooter.right_allowableError)) {

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
    SmartDashboard.putBoolean("Shooter Ready", shooterIsReady());
    // SmartDashboard.putNumber("Percent Left", leftShooter.getAppliedOutput());
    // SmartDashboard.putNumber("Percent Right", rightShooter.getAppliedOutput());

  }
}

