// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trap;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TrapScore;

public class StopArm extends Command {
  /** Creates a new MoveTrapArm. */

  TrapScore s_TrapScore;

  public StopArm(TrapScore s_TrapScore) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_TrapScore = s_TrapScore;
    addRequirements(s_TrapScore);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

     s_TrapScore.setSpeedZero();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_TrapScore.setSpeedZero();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
