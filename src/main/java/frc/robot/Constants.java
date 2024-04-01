package frc.robot;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static double maxModuleSpeed;

    public static class Limelight{

        // allowable error / deadband
        public static final double allowableError = 0.1;

        // Physical Constants for calculating distance to target
        public static final double speakerAprilTagCenterHeight = 57.125; // inches
        public static final double limelightRadius = 10.5; //10.0625; // inches
        public static final double camToShooterFrameAngle = 15;//12.1; // degrees
        public static final double camToPivotAngle = 34.08;//31.9; // degrees
        public static final double floorToPivot = 12.82;//13.875; // inches
    }


    public static final class Shooter{

        /* Motor IDs */
        public static final int leftMotorID = 41;
        public static final int rightMotorID = 17;
        public static final int kickerMotorID = 20;

        // right PID
        public static final double right_kP = 0.00020;
        public static final double right_kI = 0.000001;
        public static final double right_kD = 0;
        public static final double right_kFF = 0.00018;
        public static final double right_IZone = 200;

        public static final double right_maxOutput = 1;
        public static final double right_minOutput = -1;

        public static final double right_allowableError = 50;

        // left PID
        public static final double left_kP = 0.00046;
        public static final double left_kI = 0.000001;
        public static final double left_kD = 0.0015;
        public static final double left_kFF = 0.00015;
        public static final double left_IZone = 200;

        public static final double left_maxOutput = 1;
        public static final double left_minOutput = -1;

        public static final double left_allowableError = 30;
        
        // kicker PID
        public static final double kicker_kP = 0.0001;
        public static final double kicker_kI = 0.000001;
        public static final double kicker_kD = 0;
        public static final double kicker_kFF = 0.00014;
        public static final double kicker_IZone = 150;

        public static final double kicker_maxOutput = 1;
        public static final double kicker_minOutput = -1;

        public static final double kicker_allowableError = 50;
        public static final double kicker_intakeRPM = 3500;
        public static final double kicker_creepRPM = 500;
        public static final double kicker_shoot = 1500;

    }


    public static final class Turret {

        /* Motor IDs */
        public static final int rotateMotorID = 2;
        
        /* Max + Min Positions, Allowable Error */

        public static final double minTurretPosition = -3.1;
        public static final double maxTurretPosition = 3.1;

        public static final double allowableError = 0.05; // 0.05
        public static final double aimCoefficient = 0.03;

        public static final double autoAimAllowableError = 6; 

        /* Positions */
        public static final double R_intakingPosition = 0;
        public static final double turretAmpPosition = -13.147677; //counter
        public static final double clockwiseTurretAmpPosition = 11.809578895568848; //clockwise
        
        /* PID */
        public static final double kP = 0.15; // 0.5
        public static final double kI = 0.001;
        public static final double kD = 0; // 8
        public static final double kFF = 0; //0.015;
        public static final double IZone = 0.1;

        public static final double maxOutput = 1;
        public static final double minOutput = -1;
    
    }

    public static final class Colors {
            
        public static final double green = 0.77; //intaking
        public static final double strobeRed = (-0.11); 
        public static final double rainbow = (-0.99); 
        public static final double red = (0.61); //outtaking
        public static final double darkGreen = (0.75);
        public static final double lime = (0.73);   
        public static final double aqua = (0.81);   
        public static final double blue = (0.87); // ready to shoot
        public static final double strobeBlue = (-0.09); // looking for target

    }


    public static final class Pitch {

        /* Motor + Encoder IDs */
        public static final int pitchMotorID = 31;
        public static final int tiltEncoderID = 4;

        /* Max + Min Positions, Allowable Error */
        public static final double allowableError = 0.01;//0.002;
        public static final float minPitchPosition = 0.381f; // encoder units = 0.01
        public static final float maxPitchPosition = 1.6f; // encoder units = 0.67
        public static final double horizontalOffset = -0.36084;//-0.78;//0.1325; // Used for calculating angle of pitch mechanism 

        /* Positions */
        public static final double intakePosition = 0.86; 
        public static final double stageSetpoint = 0.450459; // -0.0466113125
        public static final double speakerSetpoint = 0.510625;
        public static final double ampSetpoint = 1.7;
        public static final double lobSetPoint = 0.61;
        public static final double climbPosition = 0.513;
        public static final double setpointAmp = 0.49823;
        public static final double loadingPosition = 0; //TODO: find this position
        
        /* Relative Encoder PID Constants */
        public static final double rel_kP = 0.16;
        public static final double rel_kI = 0.000;
        public static final double rel_kD = 0; //0.08;
        public static final double rel_kFF = 0.02;
        public static final double rel_IZone = 0;

        /* Absolute Encoder PID Constants */
        public static final double abs_kP = 0.1;
        public static final double abs_kI = 0.0;
        public static final double abs_kD = 0.0;

        public static final double minOutput = -1;
        public static final double maxOutput = 1;

    }


    public static final class Climber {

        /* Motor IDs */
        public static final int winchMotorID = 42; // right
        public static final int winchMotor2ID = 43; // left

        /* Deadband */
        public static final double deadband = 1.0;

        /* Soft Limits */
        public static final float minSoftLimit = 0f;
        public static final float maxSoftLimit = 191.0f;

        /* Positions */
        public static final double stowedPosition = 0;
        public static final double ampScorePosition = 190.33;
        public static final double climbPosition = 147.41;
        public static final double maxPosition = 191.0;
        

    }


    public static final class Intake {

        /* Motor IDs */
        public static final int intakeMotorID = 44;  
        public static final int intakeMotor2ID = 4; 
        public static final int armID = 55;
        public static final int armAbsoluteEncoder = 60;

        /* Positions */

        public static final double stowedPosition = 0;
        public static final double travelPosition = -0.792;//-0.1188;//-0.156; // -0.025
        public static final double deployedPosition = -8.29;//0.12646484375;

        public static final double intakeRPM = 3000; // 5000

        /* PID */
        public static final double arm_kP = 0.1; // 1.4
        public static final double arm_kI = 0; //0.0000003;
        public static final double arm_kD = 0;
        public static final double arm_kFF = 0.005;
        public static final double arm_IZone = 0;//0.001;

        public static final int stall_current_lim = 40;
        public static final int free_current_lim = 4;
        
        public static final double firstWheels_kP = 0.0004;//0.00085; 
        public static final double firstWheels_kI = 0.0;
        public static final double firstWheels_kD = 0.0;
        public static final double firstWheels_kFF = 0.0004;
        public static final double firstWheels_IZone = 100;

        public static final double secondWheels_kP = 0.00065;
        public static final double secondWheels_kI = 0;
        public static final double secondWheels_kD = 0.015;
        public static final double secondWheels_kFF = 0;
        public static final double secondWheels_IZone = 100;

        public static final double maxOutput = 1;
        public static final double minOutput = -1;

        public static final double allowableError = 0.02;

    }


    public static final class TrapScorer {

        /* Motor IDs */
        public static final int elevatorMotorID = 0;
        public static final int wheelsMotorID = 0;

        // elevator PID
        public static final double elevator_kP = 0.01;
        public static final double elevator_kI = 0;
        public static final double elevator_kD = 0;
        public static final double elevator_kFF = 0;

        // scoring wheels PID
        public static final double wheels_kP = 0.01;
        public static final double wheels_kI = 0;
        public static final double wheels_kD = 0;
        public static final double wheels_kFF = 0;

        /* Soft Limits */
        public static final float minElevatorPosition = 0f;
        public static final float maxElevatorPosition = 0f;

        /* Positions */
        public static final double allowableError = 0; //TODO: Find these positions, RPMs + allowable error
        public static final double stowedPosition = 0;
        public static final double loadingPosition = 0;
        public static final double ampScorePosition = 0;
        public static final double trapScorePosition = 0;

        /* Scoring Wheels RPMs */
        public static final double loadingRPM = 0;
        public static final double scoringRPM = 0;
    }


    public static final class Swerve {
        public static final int pigeonID = 0;

        public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20); 
        public static final double wheelBase = Units.inchesToMeters(20); 
        public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI;//chosenModule.wheelCircumference;

        public static final double driveBaseRadius = 0.3556;
        //25 inches = 0.64 meters
        //speed is 5.484 m/s
        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Angle Encoder  */
        //public static final double rotateEncoder = rotategetEncoder;
        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; 
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.27304; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; 

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 3;
            public static final Rotation2d FLangleOffset = Rotation2d.fromDegrees(336.885);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, FLangleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 53;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final Rotation2d FRangleOffset = Rotation2d.fromDegrees(285.732);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, FRangleOffset);
        }

        /* Back Right Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 24;
            public static final Rotation2d BRangleOffset = Rotation2d.fromDegrees(46.143);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, BRangleOffset);
        }

        /* Back Left Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 23;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 13;
            public static final Rotation2d BLangleOffset = Rotation2d.fromDegrees(17.842);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, BLangleOffset);
        }
    }

    public static final class AutoConstants { 
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
