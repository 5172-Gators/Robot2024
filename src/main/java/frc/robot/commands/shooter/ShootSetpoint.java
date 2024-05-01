
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AimingTolerances;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climber.ClimbMode;

public class ShootSetpoint extends Command {

  Shooter s_Shooter;
  Pitch s_Pitch;
  Turret s_Turret;
  Kicker s_Kicker;
  LEDs s_LEDs;
  Climber s_Climber;

  double leftRPM;
  double rightRPM;
  double pitch;
  double yaw;
  BooleanSupplier fire;
  DoubleSupplier yaw_aim;
  DoubleSupplier pitch_aim;
  AimingTolerances tolerances;
  
  /** Creates a new ShootSetpoint. */
  public ShootSetpoint(double leftRPM, double rightRPM, double pitch, double yaw, BooleanSupplier fire, 
          DoubleSupplier yaw_aim, DoubleSupplier pitch_aim, AimingTolerances tolerances, Shooter m_shooter, Pitch m_pitch, Turret m_turret, Kicker m_kicker, LEDs m_led, Climber climber) {
    this.rightRPM = rightRPM;
    this.leftRPM = leftRPM;
    this.pitch = pitch;
    this.yaw = yaw;
    this.fire = fire;
    this.yaw_aim = yaw_aim;
    this.pitch_aim = pitch_aim;
    this.tolerances = tolerances;

    this.s_Shooter = m_shooter;
    this.s_Pitch = m_pitch;
    this.s_Turret = m_turret;
    this.s_Kicker = m_kicker;
    this.s_LEDs = m_led;
    this.s_Climber = climber;
    
    addRequirements(s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Climber.setClimbMode(ClimbMode.AMPSCORE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("yaw sp", this.yaw);
    // SmartDashboard.putNumber("pitch sp", this.pitch);
    this.yaw += this.yaw_aim.getAsDouble()*Constants.Turret.aimCoefficient;
    // this.pitch += this.pitch_aim.getAsDouble()*0.0001;

      s_Shooter.setShooterRPM(this.rightRPM, this.leftRPM);
      s_Pitch.setPositionRaw(this.pitch);
      s_Turret.setPosition(this.yaw);

    if (s_Shooter.isReady(tolerances.leftTol, tolerances.rightTol) && s_Turret.isReady(tolerances.turretTol) && s_Pitch.isReady(tolerances.pitchTol)){
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
    return (s_Shooter.getShooterSensorInverted() && s_Shooter.getKickerSensorInverted());
  }
}
