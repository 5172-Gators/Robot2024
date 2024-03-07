// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.led.LEDTimedCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RunIntake extends Command {

  Intake s_Intake;
  Pitch s_Pitch;
  Turret s_Turret;
  Shooter s_Shooter;
  LEDs s_LEDs;

  boolean kickerSensorValue;
  boolean shooterSensorValue;

  int state;

  /** Creates a new RunIntake. */
  public RunIntake(Intake intake, Pitch pitch, Turret turret, Shooter shooter, LEDs leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = intake;
    this.s_Pitch = pitch;
    this.s_Turret = turret;
    this.s_Shooter = shooter;
    this.s_LEDs = leds;
    
    addRequirements(s_Intake, s_Pitch, s_Turret, s_Shooter, s_LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    s_Shooter.setShooterRPM(0,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // SmartDashboard.putNumber("Intake State", state);
    if (s_Shooter.shooterIsReady())
      s_Shooter.stopShooter();
    if (s_Shooter.getKickerSensor() && s_Shooter.getShooterSensor())
      s_Intake.setIntakeArmPosition(Constants.Intake.deployedPosition);
    s_Pitch.setPosition(Constants.Pitch.intakePosition);
    s_Turret.setPosition(Constants.Turret.R_intakingPosition);
    s_LEDs.setColor(-0.09);

    if (state == 0) {
      s_Intake.setIntakeRPM(Constants.Intake.intakeRPM);
      s_Shooter.setKickerRPM(Constants.Shooter.kicker_intakeRPM);

      if (s_Shooter.getShooterSensor() == false)
        state = 1;
    }
    if (state == 1) {
      s_Intake.stopIntake();
      s_Shooter.setKickerRPM(-Constants.Shooter.kicker_creepRPM);

      if (s_Shooter.getShooterSensor() == true)
        state = 2;
    }
    if (state == 2) {
      s_Shooter.setKickerRPM(Constants.Shooter.kicker_creepRPM);

      if (s_Shooter.getShooterSensor() == false)
        state = 3;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.stopKicker();
    s_Intake.stopIntake();
    s_Pitch.stopPitch();
    if(state == 3){
      Commands.sequence(new LEDTimedCommand(0.15, 0.5, s_LEDs));
    } else {
      s_LEDs.setColor(0.99);
    }
    state = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (state == 3);
  }
}
