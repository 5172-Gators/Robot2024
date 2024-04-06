// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

public class UpdateVisionPoseEstimation extends Command {
  /** Creates a new UpdateStateEstimation. */

  Swerve s_Swerve;
  Turret s_Turret;
  Shooter s_Shooter;

  public UpdateVisionPoseEstimation(Swerve swerve, Turret turret, Shooter shooter) {

    s_Swerve = swerve;
    s_Turret = turret;
    s_Shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // s_Swerve.updateVisionPoseEstimation(s_Turret.getTurretToChassis(), s_Shooter);

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
