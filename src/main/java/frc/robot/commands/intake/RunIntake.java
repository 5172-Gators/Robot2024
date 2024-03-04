// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RunIntake extends Command {

  Intake s_Intake;
  Pitch s_Pitch;
  Turret s_Turret;
  Shooter s_Shooter;

  boolean kickerSensorValue;
  boolean shooterSensorValue;

  int state;

  /** Creates a new RunIntake. */
  public RunIntake(Intake intake, Pitch pitch, Turret turret, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    s_Pitch = pitch;
    s_Turret = turret;
    s_Shooter = shooter;
    addRequirements(s_Intake, s_Pitch, s_Turret, s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    s_Shooter.stopShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kickerSensorValue = s_Shooter.getKickerSensor();
    shooterSensorValue = s_Shooter.getShooterSensor();

    
    SmartDashboard.putNumber("Intake State", state);
    new InitIntake(s_Intake, s_Pitch, s_Turret);

    if (state == 0) {
      s_Intake.setIntakeRPM(Constants.Intake.intakeRPM);
      s_Shooter.setKickerRPM(Constants.Shooter.kicker_intakeRPM);

      if (kickerSensorValue == false)
        state = 1;

    }
    if (state == 1) {
      s_Intake.stopIntake();
      s_Shooter.setKickerRPM(Constants.Shooter.kicker_intakeRPM);

      if (shooterSensorValue == false)
        state = 2;
    }
    if (state == 2) {
      s_Shooter.setKickerRPM(-Constants.Shooter.kicker_creepRPM);

      if (shooterSensorValue == true)
        state = 3;
    }
    if (state == 3) {
      s_Shooter.setKickerRPM(Constants.Shooter.kicker_creepRPM);

      if (shooterSensorValue == false)
        state = 4;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.stopKicker();
    s_Intake.stopIntake();
    state = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (state == 4);
  }
}
