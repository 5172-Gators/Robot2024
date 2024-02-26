// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  CANSparkMax winchMotor; // geared down
  CANSparkFlex climbMotor; // regular speed

  public Climber() {

    /* define + configure the winch motor for the climber */
    winchMotor = new CANSparkMax(Constants.Climber.winchMotorID, MotorType.kBrushless);
    winchMotor.restoreFactoryDefaults();
    winchMotor.setIdleMode(IdleMode.kBrake);

    /* define + configure the climber motor */
    climbMotor = new CANSparkFlex(Constants.Climber.climberMotor, MotorType.kBrushless);
    climbMotor.restoreFactoryDefaults();
    climbMotor.setIdleMode(IdleMode.kBrake);

  }

  public void joystickControl (double speed){
    // allows the climber motors to be controlled using a joystick. runs the motors at different speeds because one is geared down
    
    climbMotor.set(speed / 2);

    winchMotor.set(speed * 2);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
