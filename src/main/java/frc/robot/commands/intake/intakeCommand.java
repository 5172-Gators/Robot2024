// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Turret;


public class IntakeCommand extends Command {
  /** Creates a new intake. */

  Shooter s_Shooter;
  Intake s_Intake;
  Pitch s_Pitch;
  Turret s_Turret;

  boolean sensorValue;

  public IntakeCommand(Intake m_Intake, Shooter m_Shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Shooter = m_Shooter;
    this.s_Intake = m_Intake;
    this.s_Pitch = s_Pitch;
    this.s_Turret = s_Turret;

    addRequirements(m_Shooter, m_Intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    sensorValue = s_Shooter.getSensor();

    if (sensorValue == true){

      // deploys the intake
      s_Intake.deployIntake();

      // wait to deploy
      new WaitCommand(0.5);

      // set the position of the pitch to intake position
      // s_Pitch.pitchPosition(Constants.Pitch.P_intakingPosition);

      // wait until position is set
      new WaitCommand(0.5);

      // set the position of the turret
      s_Turret.setPosition(Constants.Turret.R_intakingPosition);

      // starts the intake and kicker to intake the note
      s_Intake.runIntake(1);

      s_Shooter.setKicker(1, 0.8);

    } else if (sensorValue == false){

      // stows the intake 
      s_Intake.stowIntake();

      // stops the kicker + the shooter
      s_Shooter.setKicker(0, 0);
      s_Intake.runIntake(0);

    }
      
    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
