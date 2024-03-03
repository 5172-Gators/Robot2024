// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;

public class AutoShoot extends Command {
  /** Creates a new AutoShoot. */

  Turret s_Turret;
  Pitch s_Pitch;
  Shooter s_Shooter;
  Limelight s_Limelight;

  boolean targetInRange;

  public AutoShoot(Shooter m_Shooter, Pitch m_Pitch, Turret m_Turret) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_Shooter = m_Shooter;
    this.s_Pitch = m_Pitch;
    this.s_Turret = m_Turret;
    addRequirements(m_Shooter, m_Pitch, m_Turret);

  }

  //neon or bright purple when ready to shoot
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {








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
