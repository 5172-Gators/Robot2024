// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
//import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pitch extends SubsystemBase {
  /** Creates a new TurretPitch. */
  
  CANSparkFlex pitchMotor;

  SparkPIDController relativePID;
  RelativeEncoder relativePitchEncoder;

  PIDController pitchPID;

  double setpoint = Constants.Pitch.intakePosition;
  Debouncer debounce = new Debouncer(0.04, DebounceType.kRising);

  private boolean pitchIsReady = false;
  private boolean pitchIsReadyLob = false;
  
  public Pitch() {

    // define + configure pitch motor
    pitchMotor = new CANSparkFlex(Constants.Pitch.pitchMotorID, MotorType.kBrushless);
    pitchMotor.restoreFactoryDefaults();
    pitchMotor.setIdleMode(IdleMode.kBrake);
    pitchMotor.setSmartCurrentLimit(80);
    pitchMotor.setInverted(true);

    // relative encoder pitch PID
    relativePID = pitchMotor.getPIDController();
    relativePID.setP(Constants.Pitch.rel_kP);
    relativePID.setI(Constants.Pitch.rel_kI);
    relativePID.setD(Constants.Pitch.rel_kD);
    relativePID.setFF(Constants.Pitch.rel_kFF);
    relativePID.setIZone(Constants.Pitch.rel_IZone);
    relativePID.setIMaxAccum(Constants.Pitch.rel_IMax, 0);
    relativePID.setOutputRange(-1, 1);
  
    // define relative encoder
    relativePitchEncoder = pitchMotor.getEncoder();
    relativePitchEncoder.setPosition(0);

    // soft limits
    pitchMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Pitch.minPitchPosition);
    pitchMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    pitchMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Pitch.maxPitchPosition);
    pitchMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

    // Config CAN update periods
    pitchMotor.setControlFramePeriodMs(Constants.Pitch.kControlFrameUpdateMs);
    pitchMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1);
    pitchMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);

    pitchMotor.burnFlash();
  }

  public void movePitch(double speed){
    double arbFF = Constants.Pitch.arm_cos_kF * getPitchAngle().getCos();
    pitchMotor.set(speed + arbFF);
  }

  public void joystickPitchControl(double speed) {
    double arbFF = Constants.Pitch.arm_cos_kF * getPitchAngle().getCos();
    pitchMotor.set(speed + arbFF);
  }

  public double getRawPitchPosition(){

    return relativePitchEncoder.getPosition();

  }

  public double encoderUnitsToDegrees(double pos) {
    return (57.5 - 19.3) / 2.3401 * pos + 19.3;
  }

  public double Rotation2dToEncoderUnits(Rotation2d angle) {
    return (angle.getDegrees()-19.3)*2.3401/(57.5-19.3);
  }

  public double getPitchDegrees() {
    double rawPos = getRawPitchPosition();
    // double pos = rawPos / (70.0 / 3.0); // Gear ratio of mechanism
    return encoderUnitsToDegrees(rawPos);
  }

  public Rotation2d getPitchAngle() {
    return Rotation2d.fromDegrees(getPitchDegrees());
  }

  public void setPosition(Rotation2d position) {

    this.setpoint = Rotation2dToEncoderUnits(position);

    double arbFF = Constants.Pitch.arm_cos_kF * getPitchAngle().getCos();

    relativePID.setReference(setpoint, CANSparkBase.ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
  }

  public void setPosition(Rotation2d position, double arbFF) {

    this.setpoint = Rotation2dToEncoderUnits(position);

    double gravityFF = Constants.Pitch.arm_cos_kF * getPitchAngle().getCos();

    relativePID.setReference(setpoint, CANSparkBase.ControlType.kPosition, 0, gravityFF + arbFF, ArbFFUnits.kPercentOut);
  }

  public void setPositionRaw(double pos) {

    this.setpoint = pos;

    double arbFF = Constants.Pitch.arm_cos_kF * getPitchAngle().getCos();

    relativePID.setReference(setpoint, CANSparkBase.ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
  }

  public void setPositionRaw(double pos, double arbFF) {

    this.setpoint = pos;

    double gravityFF = Constants.Pitch.arm_cos_kF * getPitchAngle().getCos();

    relativePID.setReference(setpoint, CANSparkBase.ControlType.kPosition, 0, gravityFF + arbFF, ArbFFUnits.kPercentOut);
  }

  public void stopPitch() {
    pitchMotor.set(0);
  }

  /**
   * Returns if the pitch subsystem is within a given tolerance in degrees
   * @param tolerance the tolerance in degrees
   */
  public boolean isReady(double tolerance) {
    double absErrorDeg = Math.abs(this.getPitchDegrees() - encoderUnitsToDegrees(this.setpoint));
    return debounce.calculate(absErrorDeg <= tolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("RelPitchPos", this.getRawPitchPosition());
    SmartDashboard.putNumber("PitchDegrees", getPitchAngle().getDegrees());
    // SmartDashboard.putNumber("Pitch Setpoint", this.setpoint);
    // SmartDashboard.putNumber("PitchOutput", pitchMotor.getAppliedOutput());

    SmartDashboard.putBoolean("Pitch Ready", isReady(Constants.Targeting.kSpeakerTol.pitchTol));

  }
}
