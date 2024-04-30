// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimbMode;

public class SetClimbMode extends Command {
  /** Creates a new SetClimberPosition. */

  Climber s_Climber;
  ClimbMode climbMode;
  
  public SetClimbMode(ClimbMode desiredClimbMode, Climber climber) {
    this.s_Climber = climber;
    this.climbMode = desiredClimbMode;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_Climber.setClimbMode(climbMode);

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
