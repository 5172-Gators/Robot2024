// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Pitch;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class AprilTagPositions extends Command {
  /** Creates a new AprilTagPositions. */
  Limelight s_Limelight;
  Pitch s_Pitch;
  Turret s_Turret;
  int currentTarget;
  
  public AprilTagPositions(Limelight s_Limelight, int currentTarget) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_Limelight = s_Limelight;
    this.s_Pitch = s_Pitch;
    this.s_Turret = s_Turret;
    this.currentTarget = currentTarget;

    addRequirements(s_Limelight);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentTarget = s_Limelight.currentTarget();
    
    switch(currentTarget) {
      case 7:
        

    }

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
