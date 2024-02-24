// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.subsystems.Limelight;

public class LimelightTurretPitch extends SubsystemBase {
  /** Creates a new TurretPitch. */
  
  CANcoder pitchEncoder;
  CANSparkMax pitchMotor;

  Limelight s_limelight;

  public LimelightTurretPitch() {

    s_limelight = new Limelight();

    // define + configure pitch motor
    // pitchMotor = new CANSparkMax(Constants.Pitch.pitchMotorID, MotorType.kBrushed);
    // pitchMotor.restoreFactoryDefaults();
    // pitchMotor.setIdleMode(IdleMode.kBrake);

    // define + configure CANcoder
    // pitchEncoder = new CANcoder(Constants.Pitch.tiltEncoderID, "rio");

  }

  public void alignLimelight(double goalPosition) {

    // + joystick value moves down
    // - joystick value moves up

    // default value of the limelight, sets according to the speed within the function
    // double limelightSpeed = 0.0;
    // double currentPosition = s_limelight.getTY();
    
    // // if the pitch is within the max + min positions, adjust the pitch. Otherwise stop the motor
  
    // double currentPitch = pitchEncoder.getPosition().getValueAsDouble();
   
    // if (currentPitch >= Constants.Pitch.minPitchPosition && limelightSpeed > 0){

    //   pitchMotor.set(limelightSpeed);

    // } else if (currentPitch <= Constants.Pitch.maxPitchPosition && limelightSpeed < 0){

    //   pitchMotor.set(limelightSpeed);

    // } else { 

    //   pitchMotor.set(0);
    
    // }



    // // aligning to limelight
    // if (goalPosition > currentPosition){

    //   limelightSpeed = Constants.Limelight.crosshairPosition - 20.5/currentPosition;
      
    //   pitchMotor.set(limelightSpeed);

    // } else if (goalPosition < currentPosition){
      
    //   limelightSpeed = Constants.Limelight.crosshairPosition + 20.5/currentPosition;

    //   pitchMotor.set(limelightSpeed);

    // } else { 

    //   pitchMotor.set(0);
    
    // }

    

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
