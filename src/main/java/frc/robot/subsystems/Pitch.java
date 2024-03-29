// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pitch extends SubsystemBase {
  /** Creates a new TurretPitch. */
  
  CANcoder pitchEncoder;
  CANSparkFlex pitchMotor;

  SparkPIDController relativePID;
  RelativeEncoder relativePitchEncoder;

  PIDController pitchPID;
  double currentPitch;

  double setpoint = Constants.Pitch.intakePosition;
  Debouncer debounce = new Debouncer(0.1, DebounceType.kRising);
  
  public Pitch() {

    // define + configure pitch motor
    pitchMotor = new CANSparkFlex(Constants.Pitch.pitchMotorID, MotorType.kBrushless);
    pitchMotor.restoreFactoryDefaults();
    pitchMotor.setIdleMode(IdleMode.kBrake);
    pitchMotor.setSmartCurrentLimit(30);
    pitchMotor.setInverted(true);

    // pitch PID
    pitchPID = new PIDController(Constants.Pitch.kP, Constants.Pitch.kI, Constants.Pitch.kD);

    relativePID = pitchMotor.getPIDController();
    relativePID.setP(Constants.Pitch.kP);
    relativePID.setI(Constants.Pitch.kI);
    relativePID.setD(Constants.Pitch.kD);
    relativePID.setFF(Constants.Pitch.kFF);
    relativePID.setIZone(Constants.Pitch.IZone);
  
    // define + configure CANcoder
    pitchEncoder = new CANcoder(Constants.Pitch.tiltEncoderID, "rio");

    // configure relative encoder
    relativePitchEncoder = pitchMotor.getEncoder();
    
  }

  public double getRelativePitchPosition(){

    return relativePitchEncoder.getPosition();

  }

  public void movePitch(double speed){

    pitchMotor.set(speed);

  }

  public void joystickPitchControl(double speed) {

    // + joystick value moves down
    // - joystick value moves up
    
    if (currentPitch <= Constants.Pitch.minPitchPosition)
      speed = Math.min(speed,0); // Clipping so it's positive
    else if (currentPitch >= Constants.Pitch.maxPitchPosition)
      speed = Math.max(speed,0); // Clipping so it's negative

    pitchMotor.set(speed);


  }

  public double getPosition(){

    return relativePitchEncoder.getPosition();
    // return pitchEncoder.getAbsolutePosition().getValueAsDouble();

  }

  public double encoderUnitsToDegrees(double pos) {
    return (pos+Constants.Pitch.horizontalOffset) * 360;
  }

  public double getPitchDegrees() {
    double pos = this.getPosition();
    return encoderUnitsToDegrees(pos);
  }

  public void setPosition(double position){

    // had to change to relative encoder so we could use feedforward
    this.setpoint = position;
    currentPitch = getRelativePitchPosition(); //getPosition();

    relativePID.setReference(position, CANSparkBase.ControlType.kPosition);
    

    // positive speed moves down, negative speed moves up

    
    // uses the absolute encoder for PID controller
    // double speed = -pitchPID.calculate(currentPitch, position);

    // if (currentPitch <= Constants.Pitch.minPitchPosition)
    //   speed = Math.min(speed,0); // Clipping so it's positive
    // else if (currentPitch >= Constants.Pitch.maxPitchPosition)
    //   speed = Math.max(speed,0); // Clipping so it's negative

    // pitchMotor.set(speed);

  }

  public void stopPitch() {
    pitchMotor.set(0);
  }

  public boolean isReady() {
    double absError = Math.abs(this.getPosition() - this.setpoint);
    if (debounce.calculate(absError <= Constants.Pitch.allowableError)){

      return true;

    } else {

      return false;

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    currentPitch = getPosition();
    double relativePitchPosition = getRelativePitchPosition();
    double currentVoltage = pitchMotor.getOutputCurrent();

    SmartDashboard.putNumber("RelativePitchPosition", relativePitchPosition);
    SmartDashboard.putNumber("Pitch Encoder Value", currentPitch);
    SmartDashboard.putNumber("Pitch_m", getPitchDegrees());
    SmartDashboard.putNumber("Pitch Motor Voltage", currentVoltage);
    SmartDashboard.putBoolean("Pitch Ready", isReady());

  }
}
