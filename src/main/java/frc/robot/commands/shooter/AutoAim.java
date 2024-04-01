// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AimingParameters;
import frc.robot.Constants;
import frc.robot.ShootingTables;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutoAim extends Command {

  Shooter s_Shooter;
  Pitch s_Pitch;
  Turret s_Turret;
  LEDs s_LEDs;
  Limelight s_LL;
  Kicker s_Kicker;

  BooleanSupplier fire;
  ShootingTables shootingTables;
  DoubleSupplier yawAim;

  /** Creates a new AutoAim. */
  public AutoAim(BooleanSupplier fire, ShootingTables shootingTables, DoubleSupplier yawAim, 
                  Shooter m_shooter, Pitch m_pitch, Turret m_turret, Kicker m_kicker, LEDs m_led, Limelight m_ll) {
    this.s_Shooter = m_shooter;
    this.s_Pitch = m_pitch;
    this.s_Turret = m_turret;
    this.s_LEDs = m_led;
    this.s_LL = m_ll;
    this.s_Kicker = m_kicker;
    this.fire = fire;
    this.shootingTables = shootingTables;
    this.yawAim = yawAim;

    addRequirements(s_Shooter, s_Pitch, s_Turret, s_LEDs, s_LL);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dist = s_LL.getDist(s_Pitch.getPitchDegrees(), s_LL.currentTarget());
    AimingParameters aimingParams = shootingTables.getAimingParams(dist);

    s_Shooter.setShooterRPM(aimingParams.getShooterRPMRight(), aimingParams.getShooterRPMLeft());
    s_Pitch.setPosition(aimingParams.getPitchAngle());
    s_Turret.autoAimYaw(s_LL.getX(), s_LL.currentTarget(), yawAim.getAsDouble());

    if (s_Shooter.shooterIsReady() && s_Turret.isAutoAimReady(s_LL.getX(), s_LL.currentTarget()) && s_Pitch.isReady()) {
      s_LEDs.setColor(0.91);
      if (this.fire.getAsBoolean())
        s_Kicker.setKickerRPM(Constants.Shooter.kicker_shoot);
    } else {
      s_LEDs.setColor(-0.11);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.setShooterRPM(0, 0);
    s_Kicker.stopKicker();
    s_LEDs.setColor(0.99);
    s_Pitch.setPosition(Constants.Pitch.intakePosition);
    s_Turret.setPosition(Constants.Turret.R_intakingPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (s_Shooter.getShooterSensor() && s_Shooter.getKickerSensor());
  }
}
