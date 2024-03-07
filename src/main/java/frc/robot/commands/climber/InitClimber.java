// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Turret;

public class InitClimber extends Command {

  Climber s_Climber;
  Pitch s_Pitch;
  Turret s_Turret;
  
  /** Creates a new InitClimber. */
  public InitClimber(Climber climber, Pitch pitch, Turret turret) {
    s_Climber = climber;
    s_Pitch = pitch;
    s_Turret = turret;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Climber,s_Pitch,s_Turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Pitch.setPosition(Constants.Pitch.climbPosition);
    s_Turret.setPosition(Constants.Turret.R_intakingPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Pitch.setPosition(Constants.Pitch.climbPosition);
    s_Turret.setPosition(Constants.Turret.R_intakingPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (s_Pitch.isReady() && s_Turret.isSetpointAimReady());
  }
}
