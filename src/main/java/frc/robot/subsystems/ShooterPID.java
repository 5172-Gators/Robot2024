// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

public class ShooterPID extends SubsystemBase {
  /** Creates a new Shooter. */

  /* Initialize Motors */
  CANSparkMax leftShooter;
  CANSparkMax rightShooter;
  
  private RelativeEncoder leftEncoder;  
  private RelativeEncoder rightencoder;
  private SparkPIDController leftPidController;
  private SparkPIDController rightPidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  CANSparkFlex kicker;

  DigitalInput kickerSensor;
  DigitalInput kickerSensor2;
  
  public ShooterPID() {
    
    /* define + configure left shooter motor */
    leftShooter = new CANSparkMax(Constants.Shooter.leftMotorID, MotorType.kBrushless);
    leftShooter.setIdleMode(IdleMode.kCoast);
    leftShooter.ControlType.kVelocity();
    leftPidController = leftShooter.getPIDController();
    leftEncoder = leftShooter.getEncoder();

    /* define + configure right shooter motor */
    rightShooter = new CANSparkMax(Constants.Shooter.rightMotorID, MotorType.kBrushless);
    rightShooter.setIdleMode(IdleMode.kCoast);
    rightShooter.ControlType.kVelocity();
    rightPidController = rightShooter.getPidController();
    rightEncoder = rightShooter.getEncoder();

    /* define + configure kicker motor */
    kicker = new CANSparkFlex(Constants.Shooter.kickerMotorID, MotorType.kBrushless);
    kicker.setIdleMode(IdleMode.kBrake);

    /* Set Inverted */
    leftShooter.setInverted(true);
    rightShooter.setInverted(false);
    kicker.setInverted(true);

    kickerSensor = new DigitalInput(0);
    kickerSensor2 = new DigitalInput(1);

     kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 6784;

    leftPidController.setP(kP);
    leftPidController.setI(kI);
    leftPidController.setD(kD);
    leftPidController.setIZone(kIz);
    leftPidController.setFF(kFF);
    leftPidController.setOutputRange(kMinOutput, kMaxOutput);

    rightPidController.setP(kP);
    rightPidController.setI(kI);
    rightPidController.setD(kD);
    rightPidController.setIZone(kIz);
    rightPidController.setFF(kFF);
    rightPidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);


  }

  public void setShooter(double speed, double rightSpeedRatio, double leftSpeedRatio) {
    // controls the shooter + kicker motors depending on the value of the beam break

    
    leftPidController.setReference(speed*leftSpeedRatio, CANSparkMax.ControlType.kVelocity);
    rightPidController.setReference(speed*rightSpeedRatio, CANSparkMax.ControlType.kVelocity);
    SmartDashboard.putNumber("SetPoint", speed);
    SmartDashboard.putNumber("leftSpeed", leftEncoder.getVelocity());
    
    SmartDashboard.putNumber("rightSpeed", leftEncoder.getVelocity());
  }

  public void setKicker(double speed, double kickerSpeedRatio){

    kicker.set(speed * kickerSpeedRatio);
    
  }

  public void robotWash(){
    // runs the wheels at a low speed for cleaning purposes
    
    leftShooter.set(1 * 0.05);
    rightShooter.set(1 * 0.05);

  }

  public void stopShooter(){
    // stops the shooter

     leftShooter.set(0);
     rightShooter.set(0);

  }

  public boolean getKickerSensor(){

    return kickerSensor.get();

  }
  public boolean getKickerSensor2(){

    return kickerSensor2.get();

  }

  public double getRightSpeed(){

    return rightEncoder.getVelocity();

  }

  public double getLeftSpeed(){

    return leftEncoder.getVelocity();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidController.setP(p); kP = p; }
        if((i != kI)) { m_pidController.setI(i); kI = i; }
        if((d != kD)) { m_pidController.setD(d); kD = d; }
        if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
          m_pidController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; 

    SmartDashboard.putBoolean("Sensor Value", getKickerSensor());
    SmartDashboard.putNumber("leftSpeed", leftEncoder.getVelocity());    
    SmartDashboard.putNumber("rightSpeed", rightEncoder.getVelocity());
  }
}

