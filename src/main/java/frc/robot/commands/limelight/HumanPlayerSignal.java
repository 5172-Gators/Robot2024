// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

public class HumanPlayerSignal extends Command {
  
  Limelight VisionLL;
  Limelight DriveLL;
  
  /** Creates a new HumanPlayerSignal. */
  public HumanPlayerSignal(Limelight visionLL, Limelight driveLL) {

    this.VisionLL = visionLL;
    this.DriveLL = driveLL;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(VisionLL, DriveLL);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    VisionLL.blinkLED();
    DriveLL.blinkLED();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    VisionLL.turnOffLED();
    DriveLL.turnOffLED();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
