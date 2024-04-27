// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.climber.StowClimber;
import frc.robot.commands.intake.StowIntake;
import frc.robot.commands.kicker.ZeroNote;
import frc.robot.commands.turret.InitAmpScore;
import frc.robot.commands.turret.ReturnToForward;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScore extends SequentialCommandGroup {
  /** Creates a new AmpScore. */

  Pitch s_Pitch;
  Turret s_Turret;
  Climber s_Climber;
  Shooter s_Shooter;
  Kicker s_Kicker;
  LEDs s_LEDs;

  BooleanSupplier fireShooter;
  double leftRPM;
  double rightRPM;
  BooleanSupplier fire;
  DoubleSupplier yaw_aim;
  DoubleSupplier pitch_aim;

  
  public AmpScore(double leftRPM, double rightRPM, BooleanSupplier fire, DoubleSupplier yaw_aim, DoubleSupplier pitch_aim, Shooter shooter, Pitch pitch, Turret turret, Climber climber, Kicker kicker, LEDs leds) {

    s_Pitch = pitch;
    s_Turret = turret;
    s_Climber = climber;
    s_Kicker =  kicker;
    s_Shooter = shooter;
    s_LEDs = leds;
    
    this.fire = fire; 
    this.yaw_aim = yaw_aim;
    this.pitch_aim = pitch_aim;
    this.leftRPM = leftRPM;
    this.rightRPM = rightRPM;

    addRequirements(s_Shooter, s_Pitch, s_Turret, s_Climber, s_Kicker, s_LEDs);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ZeroNote(s_Kicker, s_Shooter),
                new InitAmpScore(s_Climber, s_Pitch, s_Turret),
                new ShootSetpoint(leftRPM, rightRPM, 1.77, Constants.Turret.turretAmpPosition, fire, 
                                  yaw_aim, pitch_aim, Constants.Targeting.kSpeakerTol, s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs),
                new StowClimber(s_Climber),
                new ReturnToForward(s_Pitch, s_Turret));
  }
}