// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AmpScore extends Command {
  
  Shooter s_Shooter;
  Pitch s_Pitch;
  Turret s_Turret;
  LEDs s_LEDs;

  double leftRPM;
  double rightRPM;
  double pitch;
  double yaw;
  BooleanSupplier fire;
  DoubleSupplier yaw_aim;
  DoubleSupplier pitch_aim;
  Boolean calibrationMode;

  /** Creates a new ampScore. */
  public AmpScore(double leftRPM, double rightRPM, double pitch, BooleanSupplier fire, DoubleSupplier pitch_aim, Shooter m_shooter, Pitch m_pitch, Turret m_turret, LEDs m_led, Boolean calibrationMode) {
    this.rightRPM = rightRPM;
    this.leftRPM = leftRPM;
    this.pitch = pitch;
    this.fire = fire;
    this.pitch_aim = pitch_aim;

    this.s_Shooter = m_shooter;
    this.s_Pitch = m_pitch;
    this.s_Turret = m_turret;
    this.s_LEDs = m_led;
    
    this.calibrationMode = calibrationMode;
    
    addRequirements(s_Shooter, s_Pitch, s_Turret, s_LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (this.calibrationMode) {
      this.pitch += this.pitch_aim.getAsDouble()*0.0001;
      this.leftRPM = SmartDashboard.getNumber("CalibrationLeftRPM", 0);
      this.rightRPM = SmartDashboard.getNumber("CalibrationRightRPM", 0);
    }
    

    s_Pitch.setPosition(this.pitch);
    s_Turret.setPosition(0);

    if (s_Turret.isSetpointAimReady() && s_Pitch.isReady()) {
      s_LEDs.setColor(0.91);
      if (this.fire.getAsBoolean()) {
        s_Shooter.setShooterVoltage(8.5, 12.0);
        s_Shooter.setKickerVoltage(8.5);
      }
    } else {
      s_LEDs.setColor(-0.11);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.setShooterRPM(0, 0);
    s_Shooter.stopKicker();
    s_LEDs.setColor(0.99);
    s_Pitch.setPosition(Constants.Pitch.intakePosition);
    s_Turret.setPosition(Constants.Turret.R_intakingPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (s_Shooter.getShooterSensor() && s_Shooter.getKickerSensor());
  }
}
