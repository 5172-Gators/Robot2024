// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShootSetpoint extends Command {

  Shooter s_Shooter;
  Pitch s_Pitch;
  Turret s_Turret;

  double leftRPM;
  double rightRPM;
  double pitch;
  double yaw;
  BooleanSupplier fire;
  DoubleSupplier yaw_aim;
  DoubleSupplier pitch_aim;
  
  /** Creates a new ShootSetpoint. */
  public ShootSetpoint(Double leftRPM, double rightRPM, double pitch, double yaw, BooleanSupplier fire, DoubleSupplier yaw_aim, DoubleSupplier pitch_aim, Shooter m_shooter, Pitch m_pitch, Turret m_turret) {
    this.rightRPM = rightRPM;
    this.leftRPM = leftRPM;
    this.pitch = pitch;
    this.yaw = yaw;
    this.fire = fire;
    this.yaw_aim = yaw_aim;
    this.pitch_aim = pitch_aim;

    s_Shooter = m_shooter;
    s_Pitch = m_pitch;
    s_Turret = m_turret;
    
    addRequirements(s_Shooter, s_Pitch, s_Turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("yaw sp", this.yaw);
    SmartDashboard.putNumber("pitch sp", this.pitch);
    this.yaw += this.yaw_aim.getAsDouble()*Constants.Turret.aimCoefficient;
    // this.pitch += this.pitch_aim.getAsDouble()*0.0001;

    s_Shooter.setShooterRPM(this.rightRPM, this.leftRPM);
    s_Pitch.setPosition(this.pitch);
    s_Turret.setPosition(this.yaw);

    if (s_Shooter.shooterIsReady() && s_Turret.isReady() && s_Pitch.isReady() && this.fire.getAsBoolean()) {
      s_Shooter.setKickerRPM(Constants.Shooter.kicker_shoot);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.setShooterRPM(0, 0);
    s_Shooter.stopKicker();
    s_Pitch.setPosition(Constants.Pitch.intakePosition);
    s_Turret.setPosition(Constants.Turret.R_intakingPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (s_Shooter.getShooterSensor() && s_Shooter.getKickerSensor());
  }
}
