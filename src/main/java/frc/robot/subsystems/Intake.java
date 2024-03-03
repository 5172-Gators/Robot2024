// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  CANSparkFlex intakeMotor;
  CANSparkFlex jointMotor;

  RelativeEncoder jointPosition;

  CANcoder armEncoder;

  PIDController intakeArmPID;

  public Intake() {

    // define + configure the intake wheels motor
    intakeMotor = new CANSparkFlex(Constants.Intake.intakeMotorID, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setInverted(false);

    // define + configure the motor that deploys + stows the intake
    jointMotor = new CANSparkFlex(Constants.Intake.armID, MotorType.kBrushless);
    jointMotor.restoreFactoryDefaults();
    jointMotor.setInverted(false);

    // create instance of absolute encoder
    armEncoder = new CANcoder(Constants.Intake.armAbsoluteEncoder, "rio");

    // PID
    intakeArmPID = new PIDController(Constants.Pitch.kP, Constants.Pitch.kI, Constants.Pitch.kD);
    

  }

  public double getIntakePosition(){

    return armEncoder.getPosition().getValueAsDouble();

  }

  public void runIntake(double speed){

    intakeMotor.set(speed);

  }

  public void moveArm(double speed){

    jointMotor.set(speed);

  }

  public void deployIntake(){

      if (getIntakePosition() < 0.06){
        
        jointMotor.set(-0.3);
      
      } else if (getIntakePosition() > 0.061){

        // sets the motor speed to zero
        jointMotor.set(0);

      } 

    }


  public void stowIntake(){

    if (getIntakePosition() > 0.09){

      jointMotor.set(0.3);

    } else if (getIntakePosition() < 0.01){

      jointMotor.set(0.1);

    } else if (getIntakePosition() <= 0){

      jointMotor.set(0);

    }

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Intake Arm Position", getIntakePosition());
  }
}
