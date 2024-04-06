// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Stabilizer;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbModeRoutine extends SequentialCommandGroup {
  DoubleSupplier control;
  /** Creates a new ClimbModeRoutine. */
  public ClimbModeRoutine(DoubleSupplier control, Climber climber, Pitch pitch, Turret turret, Stabilizer stabilizer) {
    addCommands(
      new InitClimber(climber, pitch, turret),
      new ManualClimbControl(control, climber, pitch, turret, stabilizer)
    );
  }
}
