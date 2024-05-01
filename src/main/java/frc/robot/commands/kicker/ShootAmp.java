// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.kicker;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShootAmp extends Command {
  /** Creates a new ShootAmp. */

  Kicker s_Kicker;
  LEDs s_LEDs;
  Climber s_Climber;

  boolean turretIsReady;
  boolean pitchIsReady;
  boolean shooterIsReady;
  boolean climberIsReady;

  boolean fire;
  
  public ShootAmp(BooleanSupplier fire, boolean turretIsReady, boolean pitchIsReady, boolean climberIsReady, boolean shooterIsReady, Kicker kicker, LEDs leds, Climber climber) {

    this.s_Kicker = kicker;
    this.s_Climber = climber;
    this.s_LEDs = leds;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (turretIsReady == true && pitchIsReady == true  && s_Climber.getClimberPosition() == 1.70 && shooterIsReady == true){
      s_LEDs.setColor(Color.kPurple);
      if (fire == true)
      s_Kicker.setKickerRPM(Constants.Kicker.kicker_shoot);  
    } else {
      s_LEDs.setColor(Color.kRed);
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
