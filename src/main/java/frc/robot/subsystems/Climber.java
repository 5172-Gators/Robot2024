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
//import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkPIDController;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  CANSparkMax winchMotor; // geared down
  private RelativeEncoder winchEncoder;
  private DigitalInput magLimSwitch;

  public Climber() {

    /* define + configure the winch motor for the climber */
    winchMotor = new CANSparkMax(Constants.Climber.winchMotorID, MotorType.kBrushless);
    winchMotor.restoreFactoryDefaults();
    winchMotor.setIdleMode(IdleMode.kBrake);
    winchMotor.setSmartCurrentLimit(10);
    winchMotor.setOpenLoopRampRate(1);
    winchMotor.setInverted(true);
    winchMotor.setSoftLimit(SoftLimitDirection.kForward, 930);
    winchMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    winchMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    winchMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
     
    /* create intstance of the climb + winch motors built-in encoders */
    // climbEncoder = climbMotor.getEncoder();
    winchEncoder = winchMotor.getEncoder();

    magLimSwitch = new DigitalInput(0);
  }

  public void manualClimberControl (double speed){

    // allows the climber motors to be controlled using a joystick. runs the motors at different speeds because one is geared down
    winchMotor.set(speed);

  }

  public double getClimberPosition() {
    return winchEncoder.getPosition();
  }

  public void setClimberPosition(double position) {
    winchEncoder.setPosition(position);
  }

  public void setSpeed(double speed){
    winchMotor.set(speed);
  }

  public boolean getLimSwitch() {
    return !magLimSwitch.get();
  }

  public void disableSoftLimits() {
    winchMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    winchMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  public void enableSoftLimits() {
    winchMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    winchMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // SmartDashboard.putNumber("WinchSetPoint", rotations);

    SmartDashboard.putNumber("WinchPosition", winchEncoder.getPosition());
    SmartDashboard.putBoolean("Climb Lim Switch", getLimSwitch());
    // SmartDashboard.putNumber("ClimbPosition", climbEncoder.getPosition());
    // SmartDashboard.putNumber("ClimbSpeed", climbEncoder.getVelocity());

  }
}
