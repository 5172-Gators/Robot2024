// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkPIDController;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  CANSparkMax winchMotor; // geared down
  CANSparkFlex climbMotor; // regular speed
  private RelativeEncoder winchEncoder;
  private RelativeEncoder climbEncoder;

  public Climber() {

    /* define + configure the winch motor for the climber */
    winchMotor = new CANSparkMax(Constants.Climber.winchMotorID, MotorType.kBrushless);
    winchMotor.restoreFactoryDefaults();
    winchMotor.setIdleMode(IdleMode.kBrake);
    winchMotor.setSmartCurrentLimit(10);
    winchMotor.setOpenLoopRampRate(1);
    

    /* define + configure the climber motor */
    climbMotor = new CANSparkFlex(Constants.Climber.climberMotor, MotorType.kBrushless);
    climbMotor.restoreFactoryDefaults();
    climbMotor.setIdleMode(IdleMode.kBrake);
    climbMotor.setSmartCurrentLimit(1);
    climbMotor.setOpenLoopRampRate(0.5);
     
    /* create intstance of the climb + wince motors built-in encoders */
    climbEncoder = climbMotor.getEncoder();
    winchEncoder = winchMotor.getEncoder();
  }

  public void joystickControl (double speed){
    // allows the climber motors to be controlled using a joystick. runs the motors at different speeds because one is geared down
    
    climbMotor.set(speed / 4);

    //winchMotor.set(speed);

  }

  public double getEncoderPosition(){

    SmartDashboard.putNumber("ClimbPosition2", climbEncoder.getPosition());
    return climbEncoder.getPosition();

  }

  public void setSpeed (double speed){

    climbMotor.set (speed/4);
    winchMotor.set (speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // SmartDashboard.putNumber("WinchSetPoint", rotations);

    SmartDashboard.putNumber("WinchPosition", winchEncoder.getPosition());
    SmartDashboard.putNumber("ClimbPosition", climbEncoder.getPosition());
    SmartDashboard.putNumber("ClimbSpeed", climbEncoder.getVelocity());

  }
}
