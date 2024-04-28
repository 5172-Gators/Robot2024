// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  CANSparkMax rotateMotor;
  RelativeEncoder rotateEncoder;
  CANcoder absoluteEncoder;
  SparkPIDController m_pidController;
  double m_goalPosition;

  PIDController autoAimPIDController;

  public enum AimMode { kManual, kAuto, kSetpoint }
  private AimMode m_currentAimMode = AimMode.kManual;

  double setpoint = 0;
  Debouncer debounce = new Debouncer(0.04, DebounceType.kRising);
  Debouncer autoAimDebounce = new Debouncer(0.04, DebounceType.kFalling);

  private boolean turretIsReady = false;

  public Turret() {

    // define + configure the rotate motor
    rotateMotor = new CANSparkMax(Constants.Turret.rotateMotorID, MotorType.kBrushless);
    rotateMotor.restoreFactoryDefaults();
    rotateMotor.setInverted(true);
    rotateMotor.setIdleMode(IdleMode.kBrake);
    rotateMotor.setSmartCurrentLimit(80);

    rotateMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Turret.maxTurretPosition);
    rotateMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Turret.minTurretPosition);
    rotateMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    rotateMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    absoluteEncoder = new CANcoder(Constants.Turret.absoluteEncoderID, "rio");
    rotateEncoder = rotateMotor.getEncoder();
    rotateEncoder.setPosition(0);
    // rotateEncoder.setPosition(this.absoluteToRelativePosition(this.getAbsolutePosition()));

    // pid controller 
    m_pidController = rotateMotor.getPIDController();

    
   // SmartDashboard.putData("Turret PID", (Sendable) m_pidController);

    // config PID
    m_pidController.setP(Constants.Turret.kP);
    m_pidController.setI(Constants.Turret.kI);
    m_pidController.setD(Constants.Turret.kD);
    m_pidController.setFF(Constants.Turret.kF);
    m_pidController.setOutputRange(-1, 1);

    // Config CAN update periods
    rotateMotor.setControlFramePeriodMs(Constants.Turret.kControlFrameUpdateMs);
    rotateMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    rotateMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    rotateMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

    rotateMotor.burnFlash();
  } 

  public void rotateTurret(double speed){
    m_currentAimMode = AimMode.kManual;
    rotateMotor.set(speed);
  }

  public double getRotatePosition() { 
    return rotateEncoder.getPosition();
  }

  private double getAbsolutePosition() {
    return absoluteEncoder.getAbsolutePosition().getValue();
  }

  public Rotation2d getTurretToChassis() {
    return Rotation2d.fromDegrees(rotateEncoder.getPosition() * 360.0 / 23.333).rotateBy(Rotation2d.fromDegrees(180));
  }

  public void setPosition(double position) {
    setAngle(Rotation2d.fromDegrees(encoderUnitsToDegrees(position)));
  }

  public double degreesToEncoderUnits(double degrees) {
    return degrees * 23.333 / 360.0;//26.0 / 360;
  }

  public double encoderUnitsToDegrees(double pos) {
    return pos * 360.0 / 23.333;//26.0;
  }

  private double absoluteToRelativePosition(double absPos) {
    return absPos * 5.0; // This is approximately the gear ratio of the turret's absolute encoder to the relative encoder
  }

  public void setAngle(Rotation2d angle) {
    m_currentAimMode = AimMode.kSetpoint;
    // position is in motor rotations
    this.setpoint = degreesToEncoderUnits(angle.getDegrees());
    double frictionFF = Constants.Turret.kFrictionFF * Math.signum(rotateEncoder.getVelocity());
    m_pidController.setReference( setpoint, 
                                  CANSparkMax.ControlType.kPosition,
                                  0,
                                  frictionFF,
                                  ArbFFUnits.kPercentOut);
  }

  /**
   * set Turret angle to a Rotation2d angle object with a given robot angular velocity for compensation
   * @param angle Rotation2d setpoint
   * @param omegaRadiansPerSecond current robot angular velocity in rad/sec
   */
  public void setAngle(Rotation2d angle, double omegaRadiansPerSecond) {
    m_currentAimMode = AimMode.kSetpoint;
    // position is in motor rotations
    this.setpoint = degreesToEncoderUnits(angle.getDegrees());
    double frictionFF = Constants.Turret.kFrictionFF * Math.signum(rotateEncoder.getVelocity());
    double robotOmegaFF = Constants.Turret.kOmegaFF * omegaRadiansPerSecond;
    m_pidController.setReference(setpoint, 
                                  CANSparkMax.ControlType.kPosition,
                                  0,
                                  frictionFF + robotOmegaFF,
                                  ArbFFUnits.kPercentOut);
  }

  public void setAngle(Rotation2d angle, double omegaRadiansPerSecond, double arbFF) {
    m_currentAimMode = AimMode.kSetpoint;
    // position is in motor rotations
    this.setpoint = degreesToEncoderUnits(angle.getDegrees());
    double frictionFF = Constants.Turret.kFrictionFF * Math.signum(rotateEncoder.getVelocity());
    double robotOmegaFF = Constants.Turret.kOmegaFF * omegaRadiansPerSecond;
    m_pidController.setReference(setpoint, 
                                  CANSparkMax.ControlType.kPosition,
                                  0,
                                  frictionFF + robotOmegaFF + arbFF,
                                  ArbFFUnits.kPercentOut);
  }

  public void setFieldRelativeAngle(Rotation2d angle, Rotation2d chassisToField) {
    m_currentAimMode = AimMode.kSetpoint;
    // angle is a rotation2d object relative to the field
    Rotation2d test = angle.minus(chassisToField).minus(Rotation2d.fromDegrees(180));
    this.setpoint = test.getDegrees() * 23.333 / 360;
    double frictionFF = Constants.Turret.kFrictionFF * Math.signum(rotateEncoder.getVelocity());
    m_pidController.setReference( setpoint, 
                                  CANSparkMax.ControlType.kPosition,
                                  0,
                                  frictionFF,
                                  ArbFFUnits.kPercentOut);
  }

  /**
   * Set turret to a field-relative Rotation2d angle object. Provide robot angular velocity for compensation.
   * @param angle Rotation2d setpoint (field-relative)
   * @param chassisToField current angle of the chassis to the field (facing red is 0 degrees)
   * @param omegaRadiansPerSecond current robot angular velocity in rad/sec
   */
  public void setFieldRelativeAngle(Rotation2d angle, Rotation2d chassisToField, double omegaRadiansPerSecond) {
    m_currentAimMode = AimMode.kSetpoint;
    // angle is a rotation2d object relative to the field
    Rotation2d test = angle.minus(chassisToField).minus(Rotation2d.fromDegrees(180));
    this.setpoint = test.getDegrees() * 23.333 / 360;
    double frictionFF = Constants.Turret.kFrictionFF * Math.signum(rotateEncoder.getVelocity());
    double robotOmegaFF = Constants.Turret.kOmegaFF * omegaRadiansPerSecond;
    m_pidController.setReference( setpoint, 
                                  CANSparkMax.ControlType.kPosition,
                                  0,
                                  frictionFF + robotOmegaFF,
                                  ArbFFUnits.kPercentOut);
  }

  public void setFieldRelativeAngle(Rotation2d angle, Rotation2d chassisToField, double omegaRadiansPerSecond, double arbFF) {
    m_currentAimMode = AimMode.kSetpoint;
    // angle is a rotation2d object relative to the field
    Rotation2d test = angle.minus(chassisToField).minus(Rotation2d.fromDegrees(180));
    this.setpoint = test.getDegrees() * 23.333 / 360;
    double frictionFF = Constants.Turret.kFrictionFF * Math.signum(rotateEncoder.getVelocity());
    double robotOmegaFF = Constants.Turret.kOmegaFF * omegaRadiansPerSecond;
    m_pidController.setReference( setpoint, 
                                  CANSparkMax.ControlType.kPosition,
                                  0,
                                  frictionFF + robotOmegaFF + arbFF,
                                  ArbFFUnits.kPercentOut);
  }

  public void updateIsReady(double tolerance) {
    if (m_currentAimMode == AimMode.kSetpoint) {

      double absError = Math.abs(this.getRotatePosition() - this.setpoint);

      if (debounce.calculate(absError <= degreesToEncoderUnits(tolerance))){

        turretIsReady = true;

      }
    }
    turretIsReady = false;
  }

  public boolean getIsReady() {
      return turretIsReady;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RelTurretPos", getRotatePosition());
    // SmartDashboard.putNumber("absEncoderPos", absoluteEncoder.getAbsolutePosition().getValue());
    // SmartDashboard.putNumber("relFromAbs", this.absoluteToRelativePosition(this.getAbsolutePosition()));
    SmartDashboard.putNumber("Turret Angle", encoderUnitsToDegrees(getRotatePosition()));
    SmartDashboard.putNumber("Turret Setpoint", encoderUnitsToDegrees(setpoint));
    // SmartDashboard.putNumber("TurretOutput", rotateMotor.getAppliedOutput());
    // SmartDashboard.putNumber("Turret To Chassis", getTurretToChassis().getDegrees());
    // SmartDashboard.putNumber("turret percent", rotateMotor.getAppliedOutput());
    SmartDashboard.putBoolean("Turret Ready", getIsReady());

  }
}

