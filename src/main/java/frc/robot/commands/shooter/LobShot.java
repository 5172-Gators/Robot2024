// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AimingParameters;
import frc.robot.Constants;
import frc.robot.ShootingTables;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

public class LobShot extends Command {

  Shooter s_Shooter;
  Pitch s_Pitch;
  Turret s_Turret;
  LEDs s_LEDs;
  Kicker s_Kicker;
  Swerve s_Swerve;

  BooleanSupplier fire;
  DoubleSupplier dist;
  ShootingTables shootingTables;
  DoubleSupplier chassisToTargetAngle;
  DoubleSupplier chassisToFieldAngle;

  /** Creates a new AutoAim. */
  public LobShot(BooleanSupplier fire, ShootingTables shootingTables, DoubleSupplier dist, DoubleSupplier chassisToTargetAngle, DoubleSupplier chassisToFieldAngle, 
                  Shooter m_shooter, Pitch m_pitch, Turret m_turret, Kicker m_kicker, LEDs m_led, Swerve m_swerve) {
    this.s_Shooter = m_shooter;
    this.s_Pitch = m_pitch;
    this.s_Turret = m_turret;
    this.s_LEDs = m_led;
    this.s_Kicker = m_kicker;
    this.s_Swerve = m_swerve;
    this.fire = fire;
    this.dist = dist;
    this.shootingTables = shootingTables;
    this.chassisToTargetAngle = chassisToTargetAngle;
    this.chassisToFieldAngle = chassisToFieldAngle;

    addRequirements(s_Shooter, s_Pitch, s_Turret, s_LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_Shooter.setShooterRPM(3109-200, 2769-200);
    s_Pitch.setPositionRaw(1.4366607);
    s_Turret.setFieldRelativeAngle(Rotation2d.fromDegrees(chassisToTargetAngle.getAsDouble()), 
                                    Rotation2d.fromDegrees(chassisToFieldAngle.getAsDouble()),
                                    Units.degreesToRadians(s_Swerve.getAngularVelocityGyro()));

    if (s_Shooter.shooterIsReadyLob() && s_Turret.isReady() && s_Pitch.isReadyLob()) {
      s_LEDs.setColor(Color.kPurple);
      if (this.fire.getAsBoolean())
        s_Kicker.setKickerRPM(Constants.Kicker.kicker_shoot);
    } else {
      s_LEDs.setColor(Color.kRed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.setShooterRPM(0, 0);
    s_Kicker.stopKicker();
    s_LEDs.setColor(Color.kBlack);
    s_Pitch.setPositionRaw(Constants.Pitch.intakePosition);
    s_Turret.setPosition(Constants.Turret.R_intakingPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (s_Shooter.getShooterSensor() && s_Shooter.getKickerSensor());
  }
}
