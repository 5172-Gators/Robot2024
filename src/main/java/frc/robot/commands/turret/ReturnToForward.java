// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.pitch.SetPitchPositionRaw;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReturnToForward extends SequentialCommandGroup {
  /** Creates a new ReturnToForward. */

  Pitch s_Pitch;
  Turret s_Turret;
  
  public ReturnToForward(Pitch pitch, Turret turret) {

    this.s_Pitch = pitch;
    this.s_Turret = turret;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetPitchPositionRaw(s_Pitch, Constants.Pitch.ampScoreTravelPosition),
                new SetTurretPosition(s_Turret, Constants.Turret.R_intakingPosition));
  }
}
