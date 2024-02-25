// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StopShooter extends Command {
  /** Creates a new StopKicker. */

  Shooter s_Shooter;

  public StopShooter(Shooter s_Shooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_Shooter = s_Shooter;
    addRequirements(s_Shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_Shooter.setKicker(0, 0);
    s_Shooter.setShooter(0, 0, 0);

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
