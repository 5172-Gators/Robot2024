// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StopIntake extends Command {
  /** Creates a new StopIntake. */
  Shooter s_Shooter;
  Intake s_Intake;

  public StopIntake(Shooter s_Shooter, Intake s_Intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Shooter = s_Shooter;
    this.s_Intake = s_Intake;

    addRequirements(s_Shooter, s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // stows the intake
    s_Intake.stowIntake();

    // waits for intake to stow
    new WaitCommand(0.5);

    // stops the intake wheels
    s_Intake.runIntake(0);

    // stops the kicker
    s_Shooter.setKicker(0,0);

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
