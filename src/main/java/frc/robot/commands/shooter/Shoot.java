// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {

  Shooter s_Shooter;
  /** Creates a new Shoot. */
  public Shoot(Shooter shooter) {
    s_Shooter = shooter;
    addRequirements(s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Shooter.setKickerRPM(Constants.Shooter.kicker_shoot);
    s_Shooter.setShooterRPM(2500,2500);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.stopKicker();
    s_Shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Shooter.shooterIsReady();
  }
}