// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Turret;


public class IntakeCommand extends Command {
  /** Creates a new intake. */

  Shooter s_Shooter;
  Intake s_Intake;
  Pitch s_Pitch;
  Turret s_Turret;

  boolean kickerSensorValue;
  boolean shooterSensorValue;

  int shooterSensorTripCount = 0;
  int kickerSensorTripCount = 0;

  public IntakeCommand(Intake m_Intake, Shooter m_Shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Shooter = m_Shooter;
    this.s_Intake = m_Intake;

    addRequirements(m_Shooter, m_Intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    kickerSensorValue = s_Shooter.getKickerSensor();
    shooterSensorValue = s_Shooter.getKickerSensor();

    shooterSensorTripCount = 1;

      // deploys the intake
      // s_Intake.deployIntake();

      // // wait to deploy
      // new WaitCommand(0.5);

      // // stops the arm motor when the intake is deployed
      // s_Intake.moveArm(0);

      // // starts the intake
      // s_Intake.runIntake(0.85);

      // // waits to stop the intake 

      // new WaitCommand(5);

      // // stops the intake

      // s_Intake.runIntake(0);

      /* 
       if a piece hasn't already been intaked, set the kicker to intake a piece

       if the piece is too far up in the shooter, run the kicker backwards to put it in the right spot

       once the piece is in the right spot, stop the kicker completely
      */

      if (kickerSensorValue == true){

        s_Shooter.setKicker(1, 0.55);
  
      } 
      
      if (shooterSensorValue == false){
  
        s_Shooter.setKicker(-1, 0.1);
  
      } else if (shooterSensorValue == true && kickerSensorValue == false) {
  
        s_Shooter.setKicker(0, 0);

        shooterSensorTripCount = 2;
  
      }

      // wait to start the other command
      
      new WaitCommand(0.5);

      // slowly moves the kicker until the beam break goes into the right position
      
      if (shooterSensorValue == true && shooterSensorTripCount == 2){

        s_Shooter.setKicker(1, 0.1);
  
      } else {
  
        s_Shooter.setKicker(0, 0);
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
