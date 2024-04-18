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

public class IntakeAuto extends Command {

  Intake s_Intake;
  Pitch s_Pitch;
  Turret s_Turret;
  Shooter s_Shooter;
  LEDs s_LEDs;
  Kicker s_Kicker;
  BooleanSupplier s_useBeamBreaks;

  boolean kickerSensorValue;
  boolean shooterSensorValue;

  int state;

  /** Creates a new RunIntake. */
  public IntakeAuto(Intake intake, Pitch pitch, Turret turret, Shooter shooter, Kicker kicker, LEDs leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = intake;
    this.s_Pitch = pitch;
    this.s_Turret = turret;
    this.s_Shooter = shooter;
    this.s_LEDs = leds;
    this.s_Kicker = kicker;
    
    addRequirements(s_Intake, s_Pitch, s_Turret, s_Shooter, s_Kicker, s_LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    s_Shooter.setShooterRPM(-Constants.Shooter.creepRPM, -Constants.Shooter.creepRPM);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        if (s_Shooter.getKickerSensor() && s_Shooter.getShooterSensor()) {

          s_Intake.setIntakeArmPosition(Constants.Intake.deployedPosition);

          s_Pitch.setPositionRaw(Constants.Pitch.intakePosition);

          s_Turret.setPosition(Constants.Turret.R_intakingPosition);

          s_LEDs.setColor(Color.kTurquoise, LEDMode.FLASH);
       }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_Kicker.stopKicker();
    s_Intake.stopIntake();
    s_Pitch.stopPitch();

    if (s_Shooter.getKickerSensor() == false){

      s_LEDs.setColorTimed(new Color(25,0,255), LEDMode.SOLID, 1);

    } else {

      s_LEDs.setColor(Color.kBlack);

    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (s_Shooter.getKickerSensor() == false);

  }
}