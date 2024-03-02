// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

 
  int shooterState =0;

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

    switch (shooterState){

      CASE 0:
        if (kickerSensorValue == true){ // 1

        s_Shooter.setKicker(1, 0.55);
        SmartDashboard.putNumber("Debug", 1);
          shooterState = 1;
        }
          break;
      CASE 1:
            if (kickerSensorValue == false){ // 2
      // if the kicker is tripped, stop the wheels + up the trip count so the stupid command runs in order

         s_Shooter.setKicker(0, 0);
        SmartDashboard.putNumber("Debug", 2);
             shooterState = 2;
            }
            break;
CASE 2:
      

      // if the kicker + shooter has been tripped, run the kicker backwards until the shooter sensor hasn't been tripped
      // also up the trip count
      
     if (shooterSensorValue == false){ //3 
  
        s_Shooter.setKicker(-1, 0.1);
        shooterSensorTripCount = 1;
        SmartDashboard.putNumber("Debug", 3);
       shooterState = 3;
     }
       break;
       CASE 3:      
      // if the shooter has been tripped before, and the shooter is currently tripped, run the kicker forward to trip the sensor again to get the note into the right position
      if (shooterSensorValue == true) { // 4
  
        s_Shooter.setKicker(1, 0.1);
        SmartDashboard.putNumber("Debug", 4);
        shooterState = 4;
        
  
      }
break;
        CASE 4:
      // if the kicker is tripped again when it has been tripped before, stop the kicker + up the trip count again
      if (shooterSensorValue == false){ //5
        
        shooterSensorTripCount = 2;
        s_Shooter.setKicker(0,0);
        SmartDashboard.putNumber("Debug", 5);
        shooterState = 5;
      }
        break;

  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if the shooter has been tripped twice, end the command
    // if (shooterState = 5){
      
    //   shooterSensorTripCount = 0;
    //   kickerSensorTripCount = 0;
    //   return true;

    // } else {
    
    //  return false;

    return false;

    // }
  }

}

