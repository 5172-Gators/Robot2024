// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class MoveJoint extends Command {
  /** Creates a new Deploy. */

  Intake s_Intake;
  DoubleSupplier speed;

  public MoveJoint(Intake s_Intake, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = s_Intake;
    addRequirements(s_Intake);

    this.speed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double j_speed = speed.getAsDouble();

    s_Intake.moveArm(j_speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_Intake.moveArm(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
