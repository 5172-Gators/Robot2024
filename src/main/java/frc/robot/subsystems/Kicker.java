// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {
  /** Creates a new Kicker. */

  CANSparkFlex kicker;
  RelativeEncoder kickerEncoder;
  SparkPIDController kickerPID;

  double kickerSetpoint;

  Debouncer kickerDebounce = new Debouncer(0.1, DebounceType.kRising);

  public Kicker() {

    /* configure kicker motor */
    kicker = new CANSparkFlex(Constants.Kicker.kickerMotorID, MotorType.kBrushless);
    kicker.setIdleMode(IdleMode.kCoast);
    kicker.setInverted(true);
    kicker.setSmartCurrentLimit(80);

    kickerEncoder = kicker.getEncoder();
    
    /* Kicker PID */
    kickerPID = kicker.getPIDController();

    kickerPID.setP(Constants.Kicker.kicker_kP);
    kickerPID.setI(Constants.Kicker.kicker_kI);
    kickerPID.setD(Constants.Kicker.kicker_kD);
    kickerPID.setFF(Constants.Kicker.kicker_kFF);
    // kickerPID.setOutputRange(Constants.Kicker.kicker_minOutput, Constants.Kicker.kicker_maxOutput);

  }

  public double getKickerRPM() {

    return kickerEncoder.getVelocity();

  }

  public void setKickerRPM(double rpm){

    this.kickerSetpoint = rpm;
    kickerPID.setReference(rpm, CANSparkFlex.ControlType.kVelocity);

  }

  public void setKickerOpenLoop(double speed) {

    kicker.set(speed);

  }

  public void setKickerVoltage(double voltage) {

    kickerPID.setReference(voltage, CANSparkFlex.ControlType.kVoltage);

  }

  public void stopKicker() {

    kicker.set(0);
    
  }

  public boolean kickerIsReady() {

  double absError = Math.abs(this.getKickerRPM() - this.kickerSetpoint);

    if (kickerDebounce.calculate(absError <= Constants.Kicker.kicker_allowableError)) {

      return true;

    } else {

      return false;

    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putNumber("Kicker RPM", getKickerRPM());
    // SmartDashboard.putNumber("Kicker current", kicker.getOutputCurrent());
    // SmartDashboard.putBoolean("Kicker Ready", kickerIsReady());

  }
}
