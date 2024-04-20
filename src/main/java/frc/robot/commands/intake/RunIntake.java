// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.limelight.BlinkLimelight;
import frc.robot.commands.limelight.HumanPlayerSignal;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LEDs.LEDMode;
import frc.robot.subsystems.Shooter.NotePossession;

public class RunIntake extends Command {

  Intake s_Intake;
  Pitch s_Pitch;
  Turret s_Turret;
  Shooter s_Shooter;
  Limelight s_VisionLL;
  Limelight s_DriveLL;
  LEDs s_LEDs;
  Kicker s_Kicker;
  BooleanSupplier s_useBeamBreaks;

  boolean kickerSensorValue;
  boolean shooterSensorValue;

  int state;

  /** Creates a new RunIntake. */
  public RunIntake(Intake intake, Pitch pitch, Turret turret, Shooter shooter, Kicker kicker, Limelight visionLL, Limelight driveLL, LEDs leds, BooleanSupplier useBeamBreaks) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = intake;
    this.s_Pitch = pitch;
    this.s_Turret = turret;
    this.s_Shooter = shooter;
    this.s_VisionLL = visionLL;
    this.s_DriveLL = driveLL;
    this.s_LEDs = leds;
    this.s_Kicker = kicker;
    this.s_useBeamBreaks = useBeamBreaks;
    
    addRequirements(s_Intake, s_Pitch, s_Turret, s_Shooter, s_Kicker, s_VisionLL, s_DriveLL, s_LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    s_Shooter.setShooterRPM(-Constants.Shooter.creepRPM, -Constants.Shooter.creepRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // SmartDashboard.putNumber("Intake State", state);

    if (s_useBeamBreaks.getAsBoolean()) {

      if (state == 0) {
        s_Shooter.setShooterRPM(-Constants.Shooter.creepRPM, -Constants.Shooter.creepRPM);

        if (s_Shooter.currentNotePossession == NotePossession.NONE) {
          s_Intake.setIntakeArmPosition(Constants.Intake.deployedPosition);
          s_Pitch.setPositionRaw(Constants.Pitch.intakePosition);
          s_Turret.setPosition(Constants.Turret.R_intakingPosition);
          s_LEDs.setColor(Color.kTurquoise, LEDMode.FLASH);
        }

        s_Intake.setIntakeSpeed(1, 1); // 1
        s_Kicker.setKickerRPM(Constants.Kicker.kicker_intakeRPM);

        if (s_Shooter.getShooterSensorInverted() == false) // shooter sensor
          state = 1;
      }

      if (state == 1) {

        s_Intake.stopIntake(); // #TODO Let's question this
        s_Kicker.setKickerRPM(-Constants.Kicker.kicker_creepRPM);
        s_Shooter.setShooterRPM(-Constants.Shooter.creepRPM, -Constants.Shooter.creepRPM);

        if (s_Shooter.getShooterSensorInverted() == true)
          state = 2;

      }

      if (state == 2) {

        s_Kicker.setKickerRPM(Constants.Kicker.kicker_creepRPM);

        if (s_Shooter.getShooterSensorInverted() == false)
        state = 3;
      }
    } else {
      s_Intake.setIntakeArmPosition(Constants.Intake.deployedPosition);
      s_Pitch.setPositionRaw(Constants.Pitch.intakePosition);
      s_Turret.setPosition(Constants.Turret.R_intakingPosition);
      s_LEDs.setColor(Color.kTurquoise, LEDMode.FLASH);

      s_Shooter.setShooterRPM(-Constants.Shooter.creepRPM, -Constants.Shooter.creepRPM);
      s_Intake.setIntakeSpeed(1, 1); // 1
      s_Kicker.setKickerRPM(Constants.Kicker.kicker_intakeRPM);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_Kicker.stopKicker();
    s_Intake.stopIntake();
    s_Pitch.stopPitch();

    if (s_Shooter.kickerSensorFlag) {

      s_LEDs.setColorTimed(new Color(25,0,255), LEDMode.SOLID, 1);

    } else {

      s_LEDs.setColor(Color.kBlack);

    }

    state = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (s_Shooter.currentNotePossession == NotePossession.FULL);
  }
}
