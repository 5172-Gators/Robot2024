// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pitch extends SubsystemBase {
  /** Creates a new TurretPitch. */
  
  CANcoder pitchEncoder;
  CANSparkMax pitchMotor;
  PIDController pitchPID;
  double currentPitch;

  double setpoint=Constants.Pitch.P_intakingPosition;
  
  public Pitch() {

    // define + configure pitch motor
    pitchMotor = new CANSparkMax(Constants.Pitch.pitchMotorID, MotorType.kBrushed);
    pitchMotor.restoreFactoryDefaults();
    pitchMotor.setIdleMode(IdleMode.kBrake);
    pitchMotor.setSmartCurrentLimit(5);

    // pitch PID
    pitchPID = new PIDController(Constants.Pitch.kP, Constants.Pitch.kI, Constants.Pitch.kD);
  

    // define + configure CANcoder
    pitchEncoder = new CANcoder(Constants.Pitch.tiltEncoderID, "rio");
    
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

    return pitchEncoder.getPosition().getValueAsDouble();

  }

  public void setPosition(double position){

    this.setpoint = position;
    currentPitch = getPosition();
    
    // positive speed moves down, negative speed moves up

    double speed = -pitchPID.calculate(currentPitch, position);

    if (currentPitch <= Constants.Pitch.minPitchPosition)
      speed = Math.min(speed,0); // Clipping so it's positive
    else if (currentPitch >= Constants.Pitch.maxPitchPosition)
      speed = Math.max(speed,0); // Clipping so it's negative

    pitchMotor.set(speed);

  }

  public void stopPitch() {
    pitchMotor.set(0);
  }

  public boolean isReady() {
    if (Math.abs(this.getPosition() - this.setpoint) <= Constants.Pitch.allowableError){

      return true;

    } else {

      return false;

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    currentPitch = getPosition();
    // double currentVoltage = pitchMotor.getOutputCurrent();

    SmartDashboard.putNumber("Tilt Encoder Value", currentPitch);
    // SmartDashboard.putNumber("Pitch Motor Voltage", currentVoltage);
    SmartDashboard.putBoolean("Pitch Ready", isReady());

  }
}
