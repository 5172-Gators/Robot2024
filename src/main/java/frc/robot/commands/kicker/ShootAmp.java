// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.kicker;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber.ClimbMode;
import frc.robot.subsystems.Shooter.NotePossession;

public class ShootAmp extends Command {
  /** Creates a new ShootAmp. */

  RobotContainer rc;

  BooleanSupplier fire;
  
  public ShootAmp(BooleanSupplier fire, RobotContainer rc) {

    this.rc = rc;
    this.fire = fire;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rc.s_Kicker, rc.s_LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (rc.s_Turret.isReady() && rc.s_Pitch.isReady()  && rc.s_Climber.isReady() && rc.s_Climber.currentClimbMode == ClimbMode.AMPSCORE && rc.s_Shooter.isReady()) {
      rc.s_LEDs.setColor(Color.kPurple);
      if (fire.getAsBoolean())
        rc.s_Kicker.setKickerRPM(Constants.Kicker.kicker_shoot);  
    } else {
      rc.s_LEDs.setColor(Color.kRed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rc.s_Kicker.stopKicker();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rc.s_Shooter.currentNotePossession == NotePossession.NONE;
  }
}
