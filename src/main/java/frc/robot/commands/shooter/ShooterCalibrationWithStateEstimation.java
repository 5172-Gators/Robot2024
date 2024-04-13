// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

public class ShooterCalibrationWithStateEstimation extends Command {

  Shooter s_Shooter;
  Pitch s_Pitch;
  Turret s_Turret;
  LEDs s_LEDs;
  Limelight s_LL;
  Kicker s_Kicker;
  Swerve s_Swerve;

  double leftRPM;
  double rightRPM;
  double pitch;
  double yaw;
  BooleanSupplier fire;
  DoubleSupplier chassisToTargetAngle;
  DoubleSupplier chassisToFieldAngle;
  DoubleSupplier pitchAim;
  BooleanSupplier increaseLeftRPM;
  BooleanSupplier decreaseLeftRPM;
  BooleanSupplier increaseRightRPM;
  BooleanSupplier decreaseRightRPM;


  /** Creates a new ShootSetpointCalibration. */
  public ShooterCalibrationWithStateEstimation(double leftRPM, double rightRPM, double pitch, double yaw, BooleanSupplier fire, 
          DoubleSupplier chassisToTargetAngle, DoubleSupplier chassisToFieldAngle, DoubleSupplier pitchAim, Shooter m_shooter, Pitch m_pitch, Turret m_turret, Kicker m_kicker,
          LEDs m_led, Swerve m_swerve, Limelight m_ll, BooleanSupplier incLeftRPM, BooleanSupplier decLeftRPM, BooleanSupplier incRightRPM, BooleanSupplier decRightRPM) {
    this.rightRPM = rightRPM;
    this.leftRPM = leftRPM;
    this.pitch = pitch;
    this.yaw = yaw;
    this.fire = fire;
    this.chassisToTargetAngle = chassisToTargetAngle;
    this.chassisToFieldAngle = chassisToFieldAngle;
    this.pitchAim = pitchAim;
    this.increaseLeftRPM = incLeftRPM;
    this.decreaseLeftRPM = decLeftRPM;
    this.increaseRightRPM = incRightRPM;
    this.decreaseRightRPM = decRightRPM;

    s_Shooter = m_shooter;
    s_Pitch = m_pitch;
    s_Turret = m_turret;
    s_LEDs = m_led;
    s_LL = m_ll;
    s_Kicker = m_kicker;
    s_Swerve = m_swerve;

    addRequirements(s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs, s_LL);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // this.yaw += this.yawAim.getAsDouble()*Constants.Turret.aimCoefficient;
    this.pitch += this.pitchAim.getAsDouble()*0.01;
    this.leftRPM += (this.increaseLeftRPM.getAsBoolean()?10:0) + (this.decreaseLeftRPM.getAsBoolean()?-10:0);
    this.rightRPM += (this.increaseRightRPM.getAsBoolean()?10:0) + (this.decreaseRightRPM.getAsBoolean()?-10:0);

    s_Shooter.setShooterRPM(this.rightRPM, this.leftRPM);
    s_Pitch.setPositionRaw(this.pitch);
    s_Turret.setFieldRelativeAngle(Rotation2d.fromDegrees(chassisToTargetAngle.getAsDouble()), 
                                    Rotation2d.fromDegrees(chassisToFieldAngle.getAsDouble()),
                                    Units.degreesToRadians(s_Swerve.getAngularVelocityGyro()));

    if (this.fire.getAsBoolean())
      s_Kicker.setKickerRPM(Constants.Kicker.kicker_shoot);

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
    return false;
  }
}
