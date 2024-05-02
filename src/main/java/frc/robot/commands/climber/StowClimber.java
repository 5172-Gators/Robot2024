// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climber.ClimbMode;

public class StowClimber extends Command {
  /** Creates a new StowClimber. */
  Climber s_Climber;
  
  public StowClimber(Climber climber) {

    this.s_Climber = climber;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Climber.setClimbMode(ClimbMode.STOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Climber.setClimberPositionRaw(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Climber.setClimbMode(ClimbMode.STOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return s_Climber.isReady();

  }
}
