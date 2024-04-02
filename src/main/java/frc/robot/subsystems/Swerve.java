package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator swervePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants, Constants.Swerve.Mod0.driveKS, Constants.Swerve.Mod0.driveKV, Constants.Swerve.Mod0.driveKA),
            new SwerveModule(1, Constants.Swerve.Mod1.constants, Constants.Swerve.Mod1.driveKS, Constants.Swerve.Mod1.driveKV, Constants.Swerve.Mod1.driveKA),
            new SwerveModule(2, Constants.Swerve.Mod2.constants, Constants.Swerve.Mod2.driveKS, Constants.Swerve.Mod2.driveKV, Constants.Swerve.Mod2.driveKA),
            new SwerveModule(3, Constants.Swerve.Mod3.constants, Constants.Swerve.Mod3.driveKS, Constants.Swerve.Mod3.driveKV, Constants.Swerve.Mod3.driveKA)
        };

        swervePoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, 
            getGyroYaw(), 
            getModulePositions(), 
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(60)));

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(0.5, 0, 1.0), // Translation PID constants
                        new PIDConstants(1.0, 0, 0), // Rotation PID constants
                        Constants.Swerve.maxSpeed, // Max module speed, in m/s
                        Constants.Swerve.driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public double getAngularVelocity() {
        return gyro.getAngularVelocityXWorld().getValueAsDouble() / 2048.0;
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

     public void driveRobotRelative(ChassisSpeeds speeds){
        SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        this.setModuleStates(moduleStates);
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(mSwerveMods[0].getState(),
                                                               mSwerveMods[1].getState(),
                                                               mSwerveMods[2].getState(),
                                                               mSwerveMods[3].getState());
    }

    public Translation2d getTranslationToSpeaker() {
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
            if(alliance.get() == DriverStation.Alliance.Red)
                return Constants.Field.redSpeaker.minus(getPose().getTranslation());
            else
                return Constants.Field.blueSpeaker.minus(getPose().getTranslation());
        }
        return Constants.Field.blueSpeaker.minus(getPose().getTranslation());
    }

    public Translation2d getTranslationToAmp() {
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
            if(alliance.get() == DriverStation.Alliance.Red)
                return Constants.Field.redAmp.minus(getPose().getTranslation());
            else
                return Constants.Field.blueAmp.minus(getPose().getTranslation());
        }
        return Constants.Field.blueAmp.minus(getPose().getTranslation());
    }

    public void updateOdometry(Rotation2d turretToRobot) {
        // Update using swerve odometry
        swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());
        // swervePoseEstimator.update(getGyroYaw(), getModulePositions());

        // Update using vision selected vision measurements
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-vision");
        
        // Reject faulty vision measurements
        if(limelightMeasurement.pose.getX() == 0.0)
            return;
        
        // LimelightHelpers.printPoseEstimate(limelightMeasurement);

        // If any targets are in view
        if (limelightMeasurement.tagCount > 0) {
            double xyStds = 0.7;
            double degStds = 75;
            double poseDifference = swervePoseEstimator.getEstimatedPosition().getTranslation().getDistance(limelightMeasurement.pose.getTranslation());

            // multiple targets detected
            if (limelightMeasurement.tagCount >= 2) {
                    xyStds = 0.7;
                    degStds = 75;
            }
            // 1 target with large area and close to estimated pose
            else if (limelightMeasurement.tagCount == 1 && (limelightMeasurement.avgTagArea > 0.8 && poseDifference < 0.5)) {
                    xyStds = 1.0;
                    degStds = 100;
            }
            // 1 target farther away and estimated pose is close to estimated pose
            else if (limelightMeasurement.tagCount == 1 && limelightMeasurement.avgTagArea > 0.1 && poseDifference < 0.3) {
                    xyStds = 2.0;
                    degStds = 140;
            }

            swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
            // SmartDashboard.putNumber("Vision Xe", limelightMeasurement.pose.getX());
            // SmartDashboard.putNumber("Vision Ye", limelightMeasurement.pose.getY());
            swervePoseEstimator.addVisionMeasurement(
                new Pose2d(limelightMeasurement.pose.getTranslation(), limelightMeasurement.pose.getRotation().minus(turretToRobot)),
                limelightMeasurement.timestampSeconds);
        }
    }

    @Override
    public void periodic(){
        // updateOdometry();

        // SmartDashboard.putNumber("Angular Velocity", getAngularVelocity());

        // SmartDashboard.putNumber("gyro", getGyroYaw().getDegrees());
        // SmartDashboard.putNumber("RobotPose X", getPose().getX());
        // SmartDashboard.putNumber("RobotPose Y", getPose().getY());
        // SmartDashboard.putNumber("RobotPose angle", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Distance Estimate", getTranslationToSpeaker().getNorm());
        SmartDashboard.putNumber("Angle Estimate", getTranslationToSpeaker().getAngle().getDegrees());

        for(SwerveModule mod : mSwerveMods){
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

}