// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trap_scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TrapScorer;
import frc.robot.subsystems.Turret;

public class SetElevatorPosition extends Command {
  /** Creates a new SetElevatorPosition. */

  TrapScorer s_TrapScorer;
  Turret s_Turret;
  double setpoint;

  public SetElevatorPosition(TrapScorer trapScorer, Turret turret, double setpoint) {
    

    this.setpoint = setpoint;
    s_TrapScorer = trapScorer;
    s_Turret = turret;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Turret, s_TrapScorer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    s_Turret.setPosition(Constants.Turret.R_intakingPosition);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_TrapScorer.setElevatorPosition(setpoint);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_TrapScorer.setElevatorPosition(Constants.TrapScorer.stowedPosition);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return s_TrapScorer.elevatorisAtPosition();

  }
}
