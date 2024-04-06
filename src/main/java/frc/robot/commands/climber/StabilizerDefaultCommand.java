// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Stabilizer;

public class StabilizerDefaultCommand extends Command {
  Stabilizer s_Stabilizer;
  /** Creates a new StabilizerDefaultCommand. */
  public StabilizerDefaultCommand(Stabilizer stab) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Stabilizer = stab;
    addRequirements(s_Stabilizer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Stabilizer.stabilizerControl(0);
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
