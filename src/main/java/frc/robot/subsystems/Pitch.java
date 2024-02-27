// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pitch extends SubsystemBase {
  /** Creates a new TurretPitch. */
  
  CANcoder pitchEncoder;
  CANSparkMax pitchMotor;

  double currentPitch;
  

  public Pitch() {

    // define + configure pitch motor
    pitchMotor = new CANSparkMax(Constants.Pitch.pitchMotorID, MotorType.kBrushed);
    pitchMotor.restoreFactoryDefaults();
    pitchMotor.setIdleMode(IdleMode.kBrake);

    // define + configure CANcoder
    pitchEncoder = new CANcoder(Constants.Pitch.tiltEncoderID, "rio");
    
  }

  public void movePitch(double speed){

    pitchMotor.set(speed);

  }

  public void joystickPitchControl(double speed) {

    // + joystick value moves down
    // - joystick value moves up
    
    // if the pitch is within the max + min positions, adjust the pitch. Otherwise stop the motor
  
    // double currentPitch = pitchEncoder.getPosition().getValueAsDouble();
   
    if (/* currentPitch > Constants.Pitch.maxPosition && */ speed > 0){

      pitchMotor.set(speed);

    } else if (/* currentPitch < Constants.Pitch.minPosition && */ speed < 0){

      pitchMotor.set(speed);

    } else { 

      pitchMotor.set(0);
    
    }

  }

  public double getPosition(){

    return pitchEncoder.getPosition().getValueAsDouble();

  }

  public void setPosition(double position){

    double startingPitch = pitchEncoder.getPosition().getValueAsDouble();
    // positive speed moves down, negative speed moves up

    if (startingPitch < position){

      movePitch(-0.3);
      

    } else if (startingPitch > position){

      movePitch(0.3);
      

    } else if (currentPitch == position + Constants.Pitch.allowableError || currentPitch == position - Constants.Pitch.allowableError){

      movePitch(0);

    }

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    currentPitch = pitchEncoder.getPosition().getValueAsDouble();

    SmartDashboard.putNumber("Tilt Encoder Value", currentPitch);
  }
}
