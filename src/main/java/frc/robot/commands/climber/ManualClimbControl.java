// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.intake.StowIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Stabilizer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climber.ClimbMode;

public class ManualClimbControl extends Command {

  Climber s_Climber;
  Pitch s_Pitch;
  Turret s_Turret;
  Stabilizer s_Stabilizer;
  Intake s_Intake;

  DoubleSupplier control;

  ClimbMode climbMode;

  /** Creates a new ManualClimbControl. */
  public ManualClimbControl(DoubleSupplier control, Climber climber, Pitch pitch, Turret turret, Stabilizer stab, Intake intake) {
    s_Climber = climber;
    s_Pitch = pitch;
    s_Turret = turret;
    s_Stabilizer = stab;
    s_Intake = intake;

    this.control = control;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Climber, s_Pitch, s_Turret, s_Stabilizer, s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    s_Climber.enableSoftLimits();

    new StowIntake(s_Intake);

    s_Climber.setClimbMode(ClimbMode.JOYSTICKCONTROL);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Pitch.setPositionRaw(Constants.Pitch.intakePosition);
    s_Turret.setPosition(Constants.Turret.R_intakingPosition);
    if(s_Climber.getClimberPosition() >= 0.5*Constants.Climber.maxSoftLimit)
      s_Stabilizer.stabilizerControl(0.5);

    s_Climber.manualClimberControl(this.control.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_Climber.manualClimberControl(0);
    s_Climber.disableSoftLimits();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
