// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlinkLimelight extends ParallelDeadlineGroup {

  Limelight s_Limelight;

  /** Creates a new BlinkLimelight. */
  public BlinkLimelight(Limelight visionLL) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new WaitCommand(3));
    addCommands(new InstantCommand(() -> s_Limelight.blinkLED()));
    
    this.s_Limelight = visionLL;

    addRequirements(s_Limelight);
  }
}
