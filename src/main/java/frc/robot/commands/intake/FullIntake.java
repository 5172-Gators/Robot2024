// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullIntake extends SequentialCommandGroup {

  Intake s_Intake;
  Pitch s_Pitch;
  Turret s_Turret;
  Shooter s_Shooter;

  /** Creates a new FullIntake. */
  public FullIntake(Intake intake, Pitch pitch, Turret turret, Shooter shooter) {

    s_Intake = intake;
    s_Pitch = pitch;
    s_Turret = turret;
    s_Shooter = shooter;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InitIntake(s_Intake, s_Pitch, s_Turret),
      new RunIntake(s_Intake, s_Pitch, s_Turret, s_Shooter)
    );
  }
}
