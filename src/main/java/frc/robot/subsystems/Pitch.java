// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
//import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pitch extends SubsystemBase {
  /** Creates a new TurretPitch. */
  
  // CANcoder absolutePitchEncoder;
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
    pitchMotor.setSmartCurrentLimit(36);
    pitchMotor.setInverted(true);

    // relative encoder pitch PID
    relativePID = pitchMotor.getPIDController();
    relativePID.setP(Constants.Pitch.rel_kP);
    relativePID.setI(Constants.Pitch.rel_kI);
    relativePID.setD(Constants.Pitch.rel_kD);
    relativePID.setFF(Constants.Pitch.rel_kFF);
    relativePID.setIZone(Constants.Pitch.rel_IZone);
    relativePID.setIMaxAccum(Constants.Pitch.rel_IMax, 0);
  
    // define + configure CANcoder
    // absolutePitchEncoder = new CANcoder(Constants.Pitch.tiltEncoderID, "rio");

    // define relative encoder
    relativePitchEncoder = pitchMotor.getEncoder();

    // soft limits
    pitchMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Pitch.minPitchPosition);
    pitchMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    pitchMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Pitch.maxPitchPosition);
    pitchMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

  }

  public void movePitch(double speed){

    pitchMotor.set(speed);

  }

  public void joystickPitchControl(double speed) {

    // + joystick value moves down
    // - joystick value moves up

    // currentPitch = getRelativePitchPosition();
    // if (currentPitch <= Constants.Pitch.minPitchPosition)
    //   speed = Math.max(speed,0); // Clipping so it's positive
    // else if (currentPitch >= Constants.Pitch.maxPitchPosition)
    //   speed = Math.min(speed,0); // Clipping so it's negative

    pitchMotor.set(speed);


  }

  public double getRelativePitchPosition(){

    return relativePitchEncoder.getPosition();

  }

  // public double getAbsolutePitchPosition(){

  //   return absolutePitchEncoder.getAbsolutePosition().getValueAsDouble();

  // }

  public double encoderUnitsToDegrees(double pos) {
    
    // pos = -1 * this.getAbsolutePitchPosition();
    // SmartDashboard.putNumber("pitchdegrees", pos);
    return (pos + Constants.Pitch.horizontalOffset) * 360;
    
    // return ((pos + Constants.Pitch.horizontalOffset) / 22.25) * 360;
  }

  public double getPitchDegrees() {
    double pos = this.currentPitch / 23.333333; //this.getRelativePitchPosition(); 23.33
    return encoderUnitsToDegrees(pos);
  }

  public void setPosition(double position){

    // changed to use the relative encoder for set positions
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
    double absError = Math.abs(this.getRelativePitchPosition() - this.setpoint);
    
    if (debounce.calculate(absError <= Constants.Pitch.allowableError)){

      return true;

    } else {

      return false;

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // double relativePitchPosition = getRelativePitchPosition();
    // double outputCurrent = pitchMotor.getOutputCurrent();
    // double absolutePosition = getAbsolutePitchPosition();


    SmartDashboard.putNumber("RelativePitchPosition", this.getRelativePitchPosition());
    SmartDashboard.putNumber("Pitch Setpoint", this.setpoint);
    // SmartDashboard.putNumber("Pitch Encoder Value", currentPitch);
    // SmartDashboard.putNumber("Pitch_m", getPitchDegrees());
    // SmartDashboard.putNumber("AbsolutePitchPosition", absolutePosition);
    // SmartDashboard.putNumber("Pitch Motor Current", outputCurrent);
    SmartDashboard.putBoolean("Pitch Ready", isReady());

  }
}
