// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShootSetpointCalibration extends Command {

  Shooter s_Shooter;
  Pitch s_Pitch;
  Turret s_Turret;
  LEDs s_LEDs;
  Limelight s_LL;

  double leftRPM;
  double rightRPM;
  double pitch;
  double yaw;
  BooleanSupplier fire;
  DoubleSupplier yawAim;
  DoubleSupplier pitchAim;

  /** Creates a new ShootSetpointCalibration. */
  public ShootSetpointCalibration(double leftRPM, double rightRPM, double pitch, double yaw, BooleanSupplier fire, 
  DoubleSupplier yawAim, DoubleSupplier pitchAim, Shooter m_shooter, Pitch m_pitch, Turret m_turret, LEDs m_led, Limelight m_ll) {
    this.rightRPM = rightRPM;
    this.leftRPM = leftRPM;
    this.pitch = pitch;
    this.yaw = yaw;
    this.fire = fire;
    this.yawAim = yawAim;
    this.pitchAim = pitchAim;

    s_Shooter = m_shooter;
    s_Pitch = m_pitch;
    s_Turret = m_turret;
    s_LEDs = m_led;
    s_LL = m_ll;

    addRequirements(s_Shooter, s_Pitch, s_Turret, s_LEDs, s_LL);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.yaw += this.yawAim.getAsDouble()*Constants.Turret.aimCoefficient;
    this.pitch += this.pitchAim.getAsDouble()*0.0001;
    // leftRPM = SmartDashboard.getNumber("CalibrationLeftRPM", 0);
    // rightRPM = SmartDashboard.getNumber("CalibrationRightRPM", 0);

    s_Shooter.setShooterRPM(this.rightRPM, this.leftRPM);
    s_Pitch.setPosition(this.pitch);
    s_Turret.autoAimYaw(s_LL.getX(), s_LL.currentTarget(), yawAim.getAsDouble());

    if (s_Shooter.shooterIsReady() && s_Turret.isAutoAimReady(s_LL.getX(), s_LL.currentTarget()) && s_Pitch.isReady()) {
      s_LEDs.setColor(0.91);
      if (this.fire.getAsBoolean())
        s_Shooter.setKickerRPM(Constants.Shooter.kicker_shoot);
    } else {
      s_LEDs.setColor(-0.11);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.setShooterRPM(0, 0);
    s_Shooter.stopKicker();
    s_LEDs.setColor(0.99);
    s_Pitch.setPosition(Constants.Pitch.intakePosition);
    s_Turret.setPosition(Constants.Turret.R_intakingPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
