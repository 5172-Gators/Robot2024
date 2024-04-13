// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

public class SetTurretFieldRelative extends Command {

  Turret s_Turret;
  Swerve s_Swerve;
  Rotation2d s_Angle;
  DoubleSupplier s_ChassisToFieldAngleDegrees;

  /** Creates a new SetTurretFieldRelative. */
  public SetTurretFieldRelative(Turret turret, Swerve swerve, Rotation2d angle, DoubleSupplier chassisToFieldAngleDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Turret = turret;
    this.s_Swerve = swerve;
    this.s_Angle = angle;
    this.s_ChassisToFieldAngleDegrees = chassisToFieldAngleDegrees;
    addRequirements(s_Turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_Turret.setFieldRelativeAngle(s_Angle, 
                                    Rotation2d.fromDegrees(s_ChassisToFieldAngleDegrees.getAsDouble()),
                                    Units.degreesToRadians(s_Swerve.getAngularVelocityGyro()));

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
