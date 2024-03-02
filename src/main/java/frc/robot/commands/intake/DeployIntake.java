// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class DeployIntake extends Command {
  /** Creates a new DeployIntake. */

  Intake s_Intake;

  public DeployIntake(Intake s_Intake) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_Intake = s_Intake;
    addRequirements(s_Intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // deploys the intake
    s_Intake.deployIntake();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (s_Intake.getIntakePosition() < 0.8){
     
      return true;

    } else {

      return false;

    }
    
  
  }
}
