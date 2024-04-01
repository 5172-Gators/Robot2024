// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trap_scorer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Turret;
import frc.robot.Constants;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.TrapScorer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScore extends SequentialCommandGroup {
  /** Creates a new TrapScore. */

  Turret s_Turret;
  TrapScorer s_TrapScorer;
  Kicker s_Kicker;
  
  public AmpScore() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetElevatorPosition(s_TrapScorer, s_Turret, Constants.TrapScorer.ampScorePosition),
      new RunScorerWheelsTimed(s_Kicker, 3.0, Constants.TrapScorer.scoringRPM), //TODO: make sure that this command is actually running the scoring wheels
      new SetElevatorPosition(s_TrapScorer, s_Turret, Constants.TrapScorer.stowedPosition)
    );
  }
}
