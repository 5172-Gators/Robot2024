// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
//import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkPIDController;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  CANSparkFlex winchMotor1; 
  CANSparkFlex winchMotor2; 
  RelativeEncoder winchEncoder1;
  RelativeEncoder winchEncoder2;

  SparkPIDController winch1PID;

  public enum ClimbMode { STOP, STOW, AMPSCORE, JOYSTICKCONTROL };

  public ClimbMode currentClimbMode = ClimbMode.STOP;

  public double setpoint = Constants.Climber.stowedPosition;

  public Climber() {

    /* define + configure the winch motors for the climber */
    winchMotor1 = new CANSparkFlex(Constants.Climber.winchMotorID, MotorType.kBrushless);
    winchMotor1.restoreFactoryDefaults();
    winchMotor1.setIdleMode(IdleMode.kBrake);
    winchMotor1.setSmartCurrentLimit(39);
    winchMotor1.setOpenLoopRampRate(1);
    winchMotor1.setClosedLoopRampRate(0.05);
    winchMotor1.setInverted(false);

    winchMotor2 = new CANSparkFlex(Constants.Climber.winchMotor2ID, MotorType.kBrushless);
    winchMotor2.restoreFactoryDefaults();
    winchMotor2.setIdleMode(IdleMode.kBrake);
    winchMotor2.follow(winchMotor1, true);

    // soft limits
    winchMotor1.setSoftLimit(SoftLimitDirection.kForward, Constants.Climber.maxSoftLimit);
    winchMotor1.setSoftLimit(SoftLimitDirection.kReverse, Constants.Climber.minSoftLimit);
  
    winchMotor2.setSoftLimit(SoftLimitDirection.kForward, Constants.Climber.maxSoftLimit);
    winchMotor2.setSoftLimit(SoftLimitDirection.kReverse, Constants.Climber.minSoftLimit); 
 
    /* create intstance of the climb + winch motors built-in encoders */
    winchEncoder1 = winchMotor1.getEncoder();
    winchEncoder2 = winchMotor2.getEncoder();

    winch1PID = winchMotor1.getPIDController();

    winch1PID.setP(0.5); // 1.3
    winch1PID.setI(0);
    winch1PID.setD(0);
    winch1PID.setFF(0);
    winch1PID.setOutputRange(-1, 1);
    
    this.enableSoftLimits();
    
    winchMotor1.burnFlash();
    winchMotor2.burnFlash();

  }

  public void manualClimberControl (double speed){
    // allows the climber motors to be controlled using a joystick

      winchMotor1.set(speed);
      winchMotor2.set(speed);

  }

  public double getClimberPosition() {

    return winchEncoder1.getPosition();
    
  }

  public void setClimberSensorPosition(double position) {

    winchEncoder1.setPosition(position);

  }

  public void disableSoftLimits() {

    winchMotor1.enableSoftLimit(SoftLimitDirection.kForward, false);
    winchMotor2.enableSoftLimit(SoftLimitDirection.kForward, false);

    winchMotor1.enableSoftLimit(SoftLimitDirection.kReverse, false);
    winchMotor2.enableSoftLimit(SoftLimitDirection.kReverse, false);

  }

  public void enableSoftLimits() {

    winchMotor1.enableSoftLimit(SoftLimitDirection.kForward, true);
    winchMotor2.enableSoftLimit(SoftLimitDirection.kForward, true);

    winchMotor1.enableSoftLimit(SoftLimitDirection.kReverse, true);
    winchMotor2.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
  }

  public void setClimbMode (ClimbMode climbMode){

    currentClimbMode = climbMode;

  }

  public void setClimberPositionRaw(double pos){

    setpoint = pos;

    winch1PID.setReference(pos, ControlType.kPosition);

  }

  public void stopClimber() {
    manualClimberControl(0);
  }

  public boolean isReady(){

    double absError = Math.abs(setpoint - getClimberPosition());

    if (absError <= 0.5){
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // SmartDashboard.putNumber("WinchSetPoint", rotations);
    SmartDashboard.putString("ClimbMode", currentClimbMode.name());
    SmartDashboard.putNumber("Climber Position", getClimberPosition());
    SmartDashboard.putNumber("Winch1 Percent Output", winchMotor1.getAppliedOutput());
    SmartDashboard.putNumber("Winch2 Applied Output", winchMotor2.getAppliedOutput());
    // SmartDashboard.putNumber("Winch Motor 2 Position", winchEncoder2.getPosition());
    // SmartDashboard.putNumber("ClimbSpeed", climbEncoder.getVelocity());

  }
}
