// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
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
  Debouncer debounce = new Debouncer(0.1, DebounceType.kRising);
  Debouncer autoAimDebounce = new Debouncer(0.1, DebounceType.kFalling);

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

    // config PID
    m_pidController.setP(Constants.Turret.kP);
    m_pidController.setI(Constants.Turret.kI);
    m_pidController.setD(Constants.Turret.kD);
    m_pidController.setFF(0);
    m_pidController.setOutputRange(-1, 1);

    autoAimPIDController = new PIDController(0.01, 0.00001, 0.0005);
    autoAimPIDController.setTolerance(Constants.Turret.autoAimAllowableError);

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
    return Rotation2d.fromDegrees(rotateEncoder.getPosition() * 360.0 / 26.0).rotateBy(Rotation2d.fromDegrees(180));
  }

  public void setPosition(double position) {
    m_currentAimMode = AimMode.kSetpoint;
    // position is in motor rotations
    this.setpoint = position;
    m_pidController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  private double degreesToEncoderUnits(double degrees) {
    return degrees * 26.0 / 360;
  }

  private double encoderUnitsToDegrees(double pos) {
    return pos * 360.0 / 26.0;
  }

  private double absoluteToRelativePosition(double absPos) {
    return absPos * 5.0; //5.051348; // This is approximately the gear ratio of the turret's absolute encoder to the relative encoder
  }

  public void setAngle(Rotation2d angle) {
    m_currentAimMode = AimMode.kSetpoint;
    // position is in motor rotations
    this.setpoint = degreesToEncoderUnits(angle.getDegrees());
    double arbFF = Constants.Turret.kFF * Math.signum(rotateEncoder.getVelocity());
    m_pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition,0,arbFF,ArbFFUnits.kPercentOut);
  }

  public void setFieldRelativeAngle(Rotation2d angle, Rotation2d chassisToField) {
    m_currentAimMode = AimMode.kSetpoint;
    // angle is a rotation2d object relative to the field
    Rotation2d test = angle.minus(chassisToField).minus(Rotation2d.fromDegrees(180));
    this.setpoint = test.getDegrees() * 26.0 / 360;
    double arbFF = Constants.Turret.kFF * Math.signum(rotateEncoder.getVelocity());
    m_pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition,0,arbFF,ArbFFUnits.kPercentOut);
  }

  public void autoAimYaw(double tx, int tid, double joystickInput) {
    double control;
    m_currentAimMode = AimMode.kAuto;
    var alliance = DriverStation.getAlliance();

    if ((alliance.get() == DriverStation.Alliance.Blue && tid == 7) || (alliance.get() == DriverStation.Alliance.Red && tid == 4))

      control = -autoAimPIDController.calculate(tx, 0);

    else
    
      control = joystickInput / 2;

    rotateMotor.set(control);
  }

  public boolean isSetpointAimReady() {
    if (m_currentAimMode == AimMode.kSetpoint) {

      double absError = Math.abs(this.getRotatePosition() - this.setpoint);

      if (debounce.calculate(absError <= Constants.Turret.allowableError)){

        return true;

      }
    }
    return false;
  }

  public boolean isAutoAimReady(double tx, int tid) {

    if (m_currentAimMode == AimMode.kAuto) {

      var alliance = DriverStation.getAlliance();

      if (autoAimDebounce.calculate((alliance.get() == DriverStation.Alliance.Blue && tid == 7) 

              || (alliance.get() == DriverStation.Alliance.Red && tid == 4)))

        return Math.abs(tx) < Constants.Turret.autoAimAllowableError;
    }

    return false;
  }

  public boolean isAutoAimReady() {

    if (m_currentAimMode == AimMode.kAuto) {

      return autoAimPIDController.atSetpoint();

    }

    return false;

  }

  public boolean isReady() {

    return isAutoAimReady() || isSetpointAimReady();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("RelTurretPos", getRotatePosition());
    // SmartDashboard.putNumber("absEncoderPos", absoluteEncoder.getAbsolutePosition().getValue());
    // SmartDashboard.putNumber("relFromAbs", this.absoluteToRelativePosition(this.getAbsolutePosition()));
    // SmartDashboard.putNumber("Turret angle", encoderUnitsToDegrees(getRotatePosition()));
    // SmartDashboard.putNumber("TurretOutput", rotateMotor.getAppliedOutput());
    // SmartDashboard.putNumber("Turret To Chassis", getTurretToChassis().getDegrees());
    // SmartDashboard.putNumber("turret percent", rotateMotor.getAppliedOutput());
    SmartDashboard.putBoolean("Turret Ready", isReady());

  }
}

