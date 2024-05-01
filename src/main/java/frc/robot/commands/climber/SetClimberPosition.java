// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climber.ClimbMode;

public class SetClimberPosition extends Command {
  /** Creates a new JankyClimberPosition. */
  Climber s_Climber;
  Turret s_Turret;

  double setpoint;

  public SetClimberPosition(double setpoint, Climber climber, Turret turret) {

    this.setpoint = setpoint;
    this.s_Climber = climber;
    this.s_Turret = turret;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Climber, s_Turret);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    s_Climber.enableSoftLimits();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_Climber.setClimberPositionRaw(setpoint);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Climber.isReady();
  }
}
