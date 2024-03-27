// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  CANSparkFlex intakeMotor;
  
  CANSparkFlex intakeMotor2;
  CANSparkFlex jointMotor;

  CANcoder armEncoder;

  PIDController intakeArmPID;
  

  SparkPIDController intakeWheelsPID;
  
  SparkPIDController intakeWheelsPID2;
  RelativeEncoder intakeWheelsEncoder;
  
  RelativeEncoder intakeWheelsEncoder2;

  double setpoint = Constants.Intake.stowedPosition;

  Debouncer debounce = new Debouncer(0.1, DebounceType.kRising);

  public Intake() {

    // define + configure the intake wheels motor
    intakeMotor = new CANSparkFlex(Constants.Intake.intakeMotorID, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setInverted(true);
    intakeMotor.setSmartCurrentLimit(39, 0);

    intakeWheelsEncoder = intakeMotor.getEncoder();

    intakeMotor2 = new CANSparkFlex(Constants.Intake.intakeMotor2ID, MotorType.kBrushless);
    intakeMotor2.restoreFactoryDefaults();
    intakeMotor2.setIdleMode(IdleMode.kBrake);
    intakeMotor2.setInverted(true);
    intakeMotor2.setSmartCurrentLimit(39, 0);

    intakeWheelsEncoder2 = intakeMotor2.getEncoder();

    // define + configure the motor that deploys + stows the intake
    jointMotor = new CANSparkFlex(Constants.Intake.armID, MotorType.kBrushless);
    jointMotor.restoreFactoryDefaults();
    jointMotor.setInverted(false);
    jointMotor.setSmartCurrentLimit(Constants.Intake.stall_current_lim, Constants.Intake.free_current_lim);

    // create instance of absolute encoder
    armEncoder = new CANcoder(Constants.Intake.armAbsoluteEncoder, "rio");

    // arm PID
    intakeArmPID = new PIDController(Constants.Intake.arm_kP, Constants.Intake.arm_kI, Constants.Intake.arm_kD);
    intakeArmPID.setIZone(Constants.Intake.arm_IZone);

    // wheels PID
    intakeWheelsPID = intakeMotor.getPIDController();

    intakeWheelsPID.setP(Constants.Intake.wheels_kP);
    intakeWheelsPID.setI(Constants.Intake.wheels_kI);
    intakeWheelsPID.setD(Constants.Intake.wheels_kD);
    intakeWheelsPID.setFF(Constants.Intake.wheels_kFF);
    intakeWheelsPID.setOutputRange(Constants.Intake.minOutput, Constants.Intake.maxOutput);
    intakeWheelsPID.setIZone(Constants.Intake.wheels_IZone);

        intakeWheelsPID2 = intakeMotor2.getPIDController();

    intakeWheelsPID2.setP(Constants.Intake.wheels_kP);
    intakeWheelsPID2.setI(Constants.Intake.wheels_kI);
    intakeWheelsPID2.setD(Constants.Intake.wheels_kD);
    intakeWheelsPID2.setFF(Constants.Intake.wheels_kFF);
    intakeWheelsPID2.setOutputRange(Constants.Intake.minOutput, Constants.Intake.maxOutput);
    intakeWheelsPID2.setIZone(Constants.Intake.wheels_IZone);

  }

  public double getIntakePosition(){

    return armEncoder.getPosition().getValueAsDouble();

  }

  public void setIntakeRPM(double speed){

    intakeWheelsPID.setReference(speed, CANSparkFlex.ControlType.kVelocity);
    
    intakeWheelsPID2.setReference(speed, CANSparkFlex.ControlType.kVelocity);

  }

  public void moveArm(double speed){

    jointMotor.set(speed);

  }

  public void setIntakeArmPosition(double setpoint){

    this.setpoint = setpoint;
    double speed = -intakeArmPID.calculate(getIntakePosition(), setpoint);

    if (this.getIntakePosition() <= Constants.Intake.stowedPosition)
      speed = Math.min(speed,0); // Clipping so it's positive
    else if (this.getIntakePosition() >= Constants.Intake.deployedPosition)
      speed = Math.max(speed,0); // Clipping so it's negative

    jointMotor.set(speed);

  }

  public boolean isReady() {
    double absError = Math.abs(this.getIntakePosition() - this.setpoint);
    if (debounce.calculate(absError <= Constants.Intake.allowableError)) {

      return true;

    } else {

      return false;

    }
  }

  public void stopIntake() {
    intakeMotor.set(0);
    
    intakeMotor2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());

     SmartDashboard.putNumber("Intake Arm Position", getIntakePosition());

    // SmartDashboard.putNumber("Intake RPM", intakeWheelsEncoder.getVelocity());

    // SmartDashboard.putNumber("Intake Arm Setpoint", this.setpoint);

     SmartDashboard.putBoolean("IntakeIsReady",this.isReady());

    // SmartDashboard.putNumber("Intake Arm Current", jointMotor.getOutputCurrent());

  }
}
