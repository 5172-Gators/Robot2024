// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class SetTurretAngle extends Command {

  Turret s_Turret;
  Rotation2d angle;

  /** Creates a new SetTurretAngle. */
  public SetTurretAngle(Turret s_Turret, Rotation2d angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Turret = s_Turret;
    this.angle = angle;
    addRequirements(s_Turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_Turret.setAngle(angle);

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