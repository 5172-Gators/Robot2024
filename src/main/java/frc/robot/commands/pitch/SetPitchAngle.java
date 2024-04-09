// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pitch;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pitch;

public class SetPitchAngle extends Command {

  Pitch s_Pitch;
  Supplier<Rotation2d> position;

  /** Creates a new SetPitchAngle. */
  public SetPitchAngle(Pitch s_Pitch, Supplier<Rotation2d> position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Pitch = s_Pitch;
    this.position = position;
    addRequirements(s_Pitch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Pitch.setPosition(position.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Pitch.stopPitch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
