// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Pitch;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class AprilTagPositions extends Command {
  /** Creates a new AprilTagPositions. */
  Limelight s_Limelight;
  Pitch s_Pitch;
  Turret s_Turret;
  Intake s_Intake;
  DriverStation.Alliance alliance;
  int currentTarget;
  
  public AprilTagPositions(Limelight s_Limelight, int currentTarget, DriverStation.Alliance alliance) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_Limelight = s_Limelight;
    this.s_Pitch = s_Pitch;
    this.s_Turret = s_Turret;
    this.s_Intake = s_Intake;


    this.currentTarget = currentTarget;
    this.alliance = alliance;

    addRequirements(s_Limelight);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentTarget = s_Limelight.currentTarget();
    
    if (alliance == DriverStation.Alliance.Blue){

    switch(currentTarget) {
      case 1:
        // deploy intake / prepare to intake from the source
       break;

      case 2:
        // deploy intake / prepare to intake
       break;

      case 6:
      // set to score in the amp
       break;

      case 7:
        // set to score in the speakers depending on the tag values
       break;

      case 8:
        // set to score in the speaker
       break;

      case 14:
        // move to climb position (maybe)
       break;

      case 15:
        // move to climb
       break;

      case 16:
        // move to climb
       break;
      
      default:
       // intaking position

    }

    if (alliance == DriverStation.Alliance.Red){

      switch(currentTarget){

        case 3:
          // set to shoot in the speaker depending on Apriltag values
         break;

        case 4:
          // set to shoot in the speaker depending on Apriltag values
         break;

        case 5: 
          // set to shoot in the amp
         break;

        case 9:
          // source intaking position
         break;

        case 10:
          // source intaking position
         break;

        case 11:
         // set to climb
         break;

        case 12:
          // set to climb
         break;

        case 13:
          // set to climb
         break;

        default:
          // intaking position

      }

    }
      


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
