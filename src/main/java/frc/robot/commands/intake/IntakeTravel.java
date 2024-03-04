// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeTravel extends Command {
  
  Intake s_Intake;
  
  /* Creates a new IntakeTravel. */
  public IntakeTravel(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_Intake.setIntakeArmPosition(Constants.Intake.travelPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_Intake.moveArm(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return s_Intake.isReady();
  }
}
