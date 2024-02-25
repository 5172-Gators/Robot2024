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

      // deploys the intake
      s_Intake.deployIntake();

      // wait to deploy
      new WaitCommand(0.5);

      // stops the arm motor when the intake is deployed
      s_Intake.moveArm(0);

      // starts the intake
      s_Intake.runIntake(0.85);

      // if a piece hasn't already been intaked, set the kicker to intake a piece
      if (sensorValue == true){

        s_Shooter.setKicker(1, 0.55);

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
