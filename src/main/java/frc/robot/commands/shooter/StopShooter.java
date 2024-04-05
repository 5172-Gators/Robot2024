// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class StopShooter extends Command {
  Shooter s_Shooter;
  Kicker s_Kicker;

  /** Creates a new StopShooter. */
  public StopShooter(Shooter shooter) {
    s_Shooter = shooter;
   
    addRequirements(s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Shooter.stopShooter();
    //s_Shooter.setShooterRPM(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_Shooter.shooterIsReady())
      s_Shooter.stopShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.stopShooter();
    // s_Kicker.stopKicker();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
