// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class JoystickClimberControl extends Command {
  /** Creates a new JoystickClimberControl. */

  Climber s_Climber;

  DoubleSupplier jPosition;

  public JoystickClimberControl(Climber s_Climber, DoubleSupplier jPosition) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_Climber = s_Climber;
    this.jPosition = jPosition;

    addRequirements(s_Climber);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double joystickPosition = jPosition.getAsDouble();

    s_Climber.manualClimberControl(joystickPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_Climber.manualClimberControl(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
