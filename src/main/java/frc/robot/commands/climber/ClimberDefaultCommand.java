// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Stabilizer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climber.ClimbMode;

public class ClimberDefaultCommand extends Command {
  /** Creates a new ClimberDefaultCommand. */

  public Climber s_Climber;
  public Pitch s_Pitch;
  public Turret s_Turret;
  public Intake s_Intake;
  public Stabilizer s_Stabilizer;

  ClimbMode climbMode;
  DoubleSupplier joystickControl;

  public ClimberDefaultCommand(DoubleSupplier control, Climber climber) {

    this.s_Climber = climber;
    // this.s_Intake = s_Intake;
    this.joystickControl = control;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    climbMode = s_Climber.currentClimbMode;

    switch (climbMode) {

      case STOP :
        s_Climber.manualClimberControl(0);
        break;

      case STOW :
        s_Climber.setClimberPositionRaw(Constants.Climber.ampScorePosition);
        break;

      case AMPSCORE :
        s_Climber.setClimberPositionRaw(175);
        break;

      case JOYSTICKCONTROL :
        // new ManualClimbControl(joystickControl, s_Climber, s_Pitch, s_Turret, s_Stabilizer, s_Intake);
        s_Climber.manualClimberControl(joystickControl.getAsDouble());
        // s_Intake.setIntakeArmPosition(Constants.Intake.stowedPosition);
        break;

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
