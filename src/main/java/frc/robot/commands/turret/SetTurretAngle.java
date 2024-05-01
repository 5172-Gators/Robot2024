// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

public class SetTurretAngle extends Command {

  Turret s_Turret;
  Swerve s_Swerve;
  Rotation2d angle;

  boolean dontFinish = false;

  /** Creates a new SetTurretAngle. */
  public SetTurretAngle(Turret s_Turret, Swerve swerve, Rotation2d angle, boolean dontFinish) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Turret = s_Turret;
    this.s_Swerve = swerve;
    this.angle = angle;
    this.dontFinish = dontFinish;
    addRequirements(s_Turret);
  }

  public SetTurretAngle(Turret s_Turret, Swerve swerve, Rotation2d angle) {
    this(s_Turret, swerve, angle, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_Turret.setAngle(angle, Units.degreesToRadians(s_Swerve.getAngularVelocityGyro()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Turret.isAt(angle) && !dontFinish;
  }
}
