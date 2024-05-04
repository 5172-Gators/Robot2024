package frc.robot;

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

    public static class Field {
        public static final Translation2d blueSpeaker = new Translation2d(0, 5.41); //5.3 // 5.554
        public static final Translation2d redSpeaker = new Translation2d(16.539, 5.41);
        public static final Translation2d blueAmp = new Translation2d(0, 6.89); // 7.01
        public static final Translation2d redAmp = new Translation2d(16.539, 6.89); // 7.01

        public static final double speakerHeightMeters = 2.10566;
    }

    public static class Limelight {

        // allowable error / deadband
        public static final double allowableError = 0.1;

        // Physical Constants for calculating distance to target
        public static final double speakerAprilTagCenterHeight = 57.125; // inches
        public static final double limelightRadius = 10.5; //10.0625; // inches TODO measure this
        public static final double camToShooterFrameAngle = 15;//12.1; // degrees TODO measure this
        public static final double camToPivotAngle = 34.08;//31.9; // degrees
        public static final double floorToPivot = 12.82;//13.875; // inches
        
        public static final double pivotZOffset = 3; // inches TODO measure this
        public static final Translation2d turretToRobotCenter = new Translation2d(0.5, 0); // inches TODO measure this
        public static final double floorToTurret = 10; // inches TODO measure this
    }

    public static class LEDs {

        public static final int ledPwmPort = 0;
        public static final int kNumLeds = 150;

        public static final double kDefaultFlashPeriodSeconds = 0.25;

    }

    public static final class Kicker{

        /* Motor IDs */
        public static final int kickerMotorID = 20;

        // kicker PID
        public static final double kicker_kP = 0.0001;
        public static final double kicker_kI = 0.000001;
        public static final double kicker_kD = 0;
        public static final double kicker_kFF = 0.00014;
        public static final double kicker_IZone = 150;

        public static final double kicker_allowableError = 50;
        public static final double kicker_intakeRPM = 3500;
        public static final double kicker_creepRPM = 500;
        public static final double kicker_shoot = 1500;
        public static final double kicker_creepReverse = 250;
    }

    public static final class Shooter{

        /* Motor IDs */
        public static final int leftMotorID = 41;
        public static final int rightMotorID = 17;

        // right PID
        public static final double right_kP = 0.001;
        public static final double right_kI = 0;
        public static final double right_kD = 0;
        public static final double right_kFF = 0.00017;

        // left PID
        public static final double left_kP = 0.001;
        public static final double left_kI = 0;
        public static final double left_kD = 0;
        public static final double left_kFF = 0.000165;

        public static final double creepRPM = 350;

    }

    public static final class Turret {

        /* Motor IDs */
        public static final int rotateMotorID = 2;
        public static final int absoluteEncoderID = 19;
        
        /* Max + Min Positions, Allowable Error */
        
        public static final double minTurretPosition = -17.0;
        public static final double maxTurretPosition = 17.0;

        public static final double aimCoefficient = 0.03;

        // This is added to the turret setpoint to counteract the the effect of adding spin to the note while shooting
        public static final Rotation2d noteSpinOffset = Rotation2d.fromDegrees(0);

        /* Positions */
        public static final double R_intakingPosition = 0;
        public static final double turretAmpPosition = -12.8333711; //counter
        public static final Rotation2d ampPosition = Rotation2d.fromDegrees(180);
        public static final Rotation2d intakePosition = new Rotation2d();
        public static final double clockwiseTurretAmpPosition = 11.809578895568848; //clockwise
        
        /* PID */
        public static final double kP = 0.25;//0.1;//0.15; // 0.5
        public static final double kI = 0;//0.001;
        public static final double kD = 5.0; // 8
        public static final double kF = 0.0;
        public static final double kFrictionFF = 0.01;//0.0018;//0.0018;//0.01; //0.015;
        public static final double kOmegaFF = 0.085;
        public static final double IZone = 0.1;
    
    }

    public static final class Pitch {

        /* Motor + Encoder IDs */
        public static final int pitchMotorID = 31;
        public static final int tiltEncoderID = 4;

        /* Max + Min Positions, Allowable Error */
        public static final float minPitchPosition = 0.27f;//0.65f; //0.3f; //0.381f; 0
        public static final float minSafePosition = 0.27f; //0.65f
        public static final float maxPitchPosition =  2.17f; //1.75f;//1.77f; //1.6f;
        public static final double horizontalOffset = 0.022789; //0.195833; // Used for calculating angle of pitch mechanism 

        /* Positions */
        public static final double intakePosition = 0.86; // 34.86 deg
        public static final double stageSetpoint = 0.450459; // 28.55 deg
        public static final double speakerSetpoint = 0.510625; // 29.478 deg
        public static final double ampSetpoint = 2.0;//1.7; // 47.829 deg
        public static final double lobSetPoint = 0.61; // 31.01 deg
        public static final double climbPosition = 1.75; // 48.6 deg
        public static final double setpointAmp = 0.49823; // 29.287 deg
        public static final double ampScoreTravelPosition = 0.277; // 35.534
        
        /* Relative Encoder PID Constants */
        public static final double rel_kP = 15;//3.5; //0.16;
        public static final double rel_kI = 0;//0.01;
        public static final double rel_kD = 0.01; //0.08;
        public static final double rel_kFF = 0;//0.035;
        public static final double rel_IZone = 0;//0.05;
        public static final double rel_IMax = 0;//0.5;

        public static final double arm_cos_kF = 0.016;
        public static final double teleopControlInputCoefficient = 0.1;

    }

    public static final class Targeting {
        // Used in motion compensation calculation
        // public static final double kNoteVelocityCoefficient = 0.0020833 * 1.25;
        public static final double kNoteVelocityCoefficient = 0.0020833 * 1.75;

        // Motion feed forwards
        public static final double kTargeting_dT_FF = -0.1 / 5; // flywheels
        public static final double kTargeting_dPhi_FF = 1.0; // Pitch
        public static final double kTargeting_dTheta_FF = -0.3 / 4; // Turret

        // Tolerances 
        public static final AimingTolerances kSpeakerTol = new AimingTolerances(2, 0.5, 75, 75);
        public static final AimingTolerances kLobTol = new AimingTolerances(2, 1.0, 150, 150);

    }


    public static final class Climber {

        /* Motor IDs */
        public static final int winchMotorID = 42; // right
        public static final int winchMotor2ID = 43; // left

        /* Deadband */
        public static final double deadband = 1.0;

        /* Soft Limits */
        public static final float minSoftLimit = 0f;
        public static final float maxSoftLimit = 175.0f;

        /* Positions */
        public static final double stowedPosition = 0;
        public static final double ampScorePosition = 174.2;
        public static final double climbPosition = 147.41; // unused
        public static final double maxPosition = 175.0; // unused

        
    }


    public static final class Intake {

        /* Motor IDs */
        public static final int intakeMotorID = 44;  
        public static final int intakeMotor2ID = 4; 
        public static final int armID = 55;
        public static final int armAbsoluteEncoder = 60;

        /* Positions */

        public static final double stowedPosition = 0;
        public static final double travelPosition = -4.26;
        public static final double deployedPosition = -9.5;//-10.5;//-8.29;//0.12646484375;

        public static final double intakeRPM = 3000; // 5000

        /* PID */
        public static final double arm_kP = 0.1; // 1.4
        public static final double arm_kI = 0; //0.0000003;
        public static final double arm_kD = 0;
        public static final double arm_kFF = 0.005;
        public static final double arm_IZone = 0;//0.001;

        public static final int stall_current_lim = 39;
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

        public static final double allowableError = 0.02;

    }

    public static final class Swerve {
        public static final int pigeonID = 0;

        public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20); 
        public static final double wheelBase = Units.inchesToMeters(20); 
        public static final double effWheelDiameter = Units.inchesToMeters(4.34); // 4.497734
        public static final double wheelCircumference = effWheelDiameter * Math.PI;

        public static final double driveBaseRadius = 0.35921;//0.3556;
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
        public static final double driveKS = 0.55;//0.32; 
        public static final double driveKV = 1.9;//1.51;
        public static final double driveKA = 0.4;//0.27;

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
            public static final double driveKS = 0.11228;
            public static final double driveKV = 2.1819;
            public static final double driveKA = 0.37358;
            public static final Rotation2d FLangleOffset = Rotation2d.fromDegrees(336.885);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, FLangleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 53;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final double driveKS = 0.055821;
            public static final double driveKV = 2.2053;
            public static final double driveKA = 0.45893;
            public static final Rotation2d FRangleOffset = Rotation2d.fromDegrees(285.732); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, FRangleOffset);
        }

        /* Back Right Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 24;
            public static final double driveKS = 0.089545;
            public static final double driveKV = 2.2042;
            public static final double driveKA = 0.43657;
            public static final Rotation2d BRangleOffset = Rotation2d.fromDegrees(46.143); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, BRangleOffset);
        }

        /* Back Left Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 23;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 13;
            public static final double driveKS = 0.060602;
            public static final double driveKV = 2.2114;
            public static final double driveKA = 0.53029;
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
