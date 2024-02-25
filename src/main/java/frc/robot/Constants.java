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


    public enum LimelightPosition{ // TODO: FIND LIMELIGHT POSITIONS

        // speaker positions
        AGAINSTSPEAKER(0.0, 0.0, 0.0),
        SPEAKER(0.0, 0.0, 0.0),
        AMP(0.0, 0.0, 0.0);

        private double TXposition;
        private double TYposition;
        private double TAposition;

        private LimelightPosition(double tX, double tY, double tA){

            this.TXposition = tX;
            this.TYposition = tY;
            this.TAposition = tA;

        }

        public double getTX() {
            return TXposition;
        }

        public double getTY() {

            return TYposition;
        }

        public double getTA(){

            return TAposition;
        }
    }

    public static final class Shooter{

        /* Motor IDs */
        public static final int leftMotorID = 14;
        public static final int rightMotorID = 15;
        public static final int kickerMotorID = 20;

    }

    public static final class Turret {

        /* Motor IDs */
        public static final int rotateMotorID = 2;
        
        /* Max + Min Positions, Allowable Error */
        // public static final double minTurretPosition = -24.023666381835938;
        // public static final double maxTurretPosition = 23.071300506591797;
        public static final double allowableError = 0.0; //TODO: find allowable error

        /* Positions */
        public static final double R_intakingPosition = 0; //TODO: find position

        /* PID */
        public static final double kP = 0.5;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;

        public static final double maxOutput = 1;
        public static final double minOutput = -1;
    
    }

    public static final class Pitch {

        /* Motor + Encoder IDs */
        public static final int pitchMotorID = 11;
        public static final int tiltEncoderID = 4;

        /* Max + Min Positions, Allowable Error */
        public static final double allowableError = 0.001; // TODO: find allowable error
        public static final double minPitchPosition = 0.918213; 
        public static final double maxPitchPosition = 0.969971;

        /* Positions */
        public static final double P_intakingPosition = 0; //TODO: find positions

    }

    public static final class Climber {

        /* Motor IDs */
        public static final int winchMotorID = 19;
        public static final int climberMotor = 17;

        /* Positions */
        public static final double stowedPosition = 0;
        public static final double deployedPosition = 0;
    }

    public static final class Intake {

        /* Motor IDs */
        public static final int intakeMotorID = 4;
        public static final int armID = 55;

        /* Positions */
        public static final double stowedPosition = 0.2; //TODO: find this position again
        public static final double deployedPosition = -3;

        /* PID */
        public static final double kP = 0.5;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;

        public static final double maxOutput = 1;
        public static final double minOutput = -1;

    }

    public static final class TrapScorer {

        // commented out so it isn't accidentally called

        /* Motor IDs */
        public static final int rollerMotorID = 0;
        public static final int armMotorID = 30; // 55

        /* Positions */
        public static final double tScorerDeployed = 0;
        public static final double tScorerStowed = 0;

    }

    public static final class Swerve {
        public static final int pigeonID = 16;

        public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(25); 
        public static final double wheelBase = Units.inchesToMeters(25); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        //25 inches = 0.64 meters
        //speed is 5.484 m/s
        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

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
        public static final double maxSpeed = 4.5; 
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
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 23;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 13;
            public static final Rotation2d BLangleOffset = Rotation2d.fromDegrees(17.842);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, BLangleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 24;
            public static final Rotation2d BRangleOffset = Rotation2d.fromDegrees(46.143);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, BRangleOffset);
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
