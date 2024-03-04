// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  CANSparkMax rotateMotor;
  RelativeEncoder rotateEncoder;
  SparkPIDController m_pidController;
  double m_goalPosition;

  double setpoint = 0;

  public Turret() {

    // define + configure the rotate motor
    rotateMotor = new CANSparkMax(Constants.Turret.rotateMotorID, MotorType.kBrushless);
    rotateMotor.restoreFactoryDefaults();
    rotateMotor.setInverted(false);
    rotateMotor.setIdleMode(IdleMode.kBrake);

    rotateMotor.setSoftLimit(SoftLimitDirection.kForward, 3.28f);
    rotateMotor.setSoftLimit(SoftLimitDirection.kReverse, -3.28f);
    rotateMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    rotateMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);


    // create instances of the built-in encoders in the motors
    rotateEncoder = rotateMotor.getEncoder();

    // pid controller 
    m_pidController = rotateMotor.getPIDController();

    // config PID
    m_pidController.setP(Constants.Turret.kP);
    m_pidController.setI(Constants.Turret.kI);
    m_pidController.setD(Constants.Turret.kD);
    m_pidController.setFF(Constants.Turret.kFF);
    m_pidController.setOutputRange(Constants.Turret.minOutput, Constants.Turret.maxOutput);
    
    
  } 

  public void rotateTurret(double speed){

    rotateMotor.set(speed);
  }

  public double getRotatePosition() { 

    return rotateEncoder.getPosition();

  }

  public void setPosition(double position) {

    // position is in motor rotations
    this.setpoint = position;
    m_pidController.setReference(position, CANSparkMax.ControlType.kPosition);

  }

  public boolean isReady() {
    if (Math.abs(this.getRotatePosition() - this.setpoint) <= Constants.Turret.allowableError){

      return true;

    } else {

      return false;

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double m_currentPosition = rotateEncoder.getPosition();
    
    SmartDashboard.putNumber("Turret Rotate Positon", m_currentPosition);
    SmartDashboard.putBoolean("Turret Ready", isReady());

  }
}

