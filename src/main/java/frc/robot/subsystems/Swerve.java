package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.GyroTrimConfigs;
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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator swervePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public Pigeon2Configuration pigeon_config;
    private final Field2d m_field = new Field2d();

    // private final DoubleArrayLogEntry m_odometryLog;
    // private final DoubleArrayLogEntry m_pathLog;
    // private final DoubleArrayLogEntry m_pathTargetLog;
    // private final DoubleArrayLogEntry m_pathErrorLog;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        pigeon_config = new Pigeon2Configuration().withGyroTrim(new GyroTrimConfigs().withGyroScalarZ(2.35));
        gyro.getConfigurator().apply(pigeon_config);
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swervePoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, 
            getGyroYaw(), 
            getModulePositions(), 
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2.5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(60)));

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        // new PIDConstants(0.4, 0, .50), // Translation PID constants
                        // new PIDConstants(.750, 0, 0), // Rotation PID constants
                        new PIDConstants(5.0, 0, 0), // Translation PID constants
                        new PIDConstants(2.5, 0, 0), // Rotation PID constants
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

        SmartDashboard.putData("Field", m_field);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        
        Translation2d translationChiral = translation;

        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            translationChiral = translation.unaryMinus();  
        }

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translationChiral.getX(), 
                                    translationChiral.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translationChiral.getX(), 
                                    translationChiral.getY(), 
                                    rotation)
                                );
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
            for (SwerveModule mod : mSwerveMods){
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
        gyro.reset();
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        // return Rotation2d.fromDegrees(gyro.getYaw().getValue());
        return gyro.getRotation2d();
    }

    // Returns the gyro's current angular velocity in deg/sec
    public double getAngularVelocityGyro() {
        return gyro.getRate();
    }

    public double getVelocityOdometry() {
        ChassisSpeeds currentSpeed = this.getRobotRelativeSpeeds();
        double xvel = currentSpeed.vxMetersPerSecond;
        double yvel = currentSpeed.vyMetersPerSecond;
        return Math.sqrt(Math.pow(xvel, 2) + Math.pow(yvel, 2));
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

    public boolean getInLobZone() {
        // Check if robot is in an allowable position to lob to avoid accruing penalty points
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue)
                return this.getPose().getX() < 10.15;
            else if (alliance.get() == DriverStation.Alliance.Red)
                return this.getPose().getX() > 6.27;
        }
        return true;
    }

    public void setRobotPose(Pose2d pose) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void updateOdometryPoseEstimation() {
        // Update using swerve odometry
        // swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());
        swervePoseEstimator.update(getGyroYaw(), getModulePositions());
    }

    public void updateVisionPoseEstimation(RobotContainer rc) {
        // Update robot orientation for limelights
        LimelightHelpers.SetRobotOrientation("limelight-shleft", 
                                            getPose().getRotation().getDegrees(), 
                                            gyro.getRate(), 
                                            0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-shright", 
                                            getPose().getRotation().getDegrees(), 
                                            gyro.getRate(), 
                                            0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-drive", 
                                            getPose().getRotation().getDegrees(), 
                                            gyro.getRate(), 
                                            0, 0, 0, 0);


        // Update using vision selected vision measurements
        LimelightHelpers.PoseEstimate leftPoseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shleft");
        LimelightHelpers.PoseEstimate rightPoseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shright");
        LimelightHelpers.PoseEstimate frontPoseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-drive");
        SmartDashboard.putNumber("Shleft Xe", leftPoseEst.pose.getX());
        SmartDashboard.putNumber("Shleft Ye", leftPoseEst.pose.getY());

        SmartDashboard.putNumber("Shright Xe", rightPoseEst.pose.getX());
        SmartDashboard.putNumber("Shright Ye", rightPoseEst.pose.getY());

        SmartDashboard.putNumber("Drive Xe", frontPoseEst.pose.getX());
        SmartDashboard.putNumber("Drive Ye", frontPoseEst.pose.getY());


        boolean rejectLeft = false;
        boolean rejectRight = false;
        boolean rejectFront = false;

        // Reject faulty vision measurements (pose is outside of the field)
        if(leftPoseEst.pose.getX() <= 0.0 || leftPoseEst.pose.getX() >= 16.46
            || leftPoseEst.pose.getY() <= 0.0 || leftPoseEst.pose.getY() >= 8.23)
            rejectLeft = true;

        if(rightPoseEst.pose.getX() <= 0.0 || rightPoseEst.pose.getX() >= 16.46
            || rightPoseEst.pose.getY() <= 0.0 || rightPoseEst.pose.getY() >= 8.23)
            rejectRight = true;

        if(frontPoseEst.pose.getX() <= 0.0 || frontPoseEst.pose.getX() >= 16.46
            || frontPoseEst.pose.getY() <= 0.0 || frontPoseEst.pose.getY() >= 8.23)
            rejectFront = true;
        

        // if(limelightMeasurement.avgTagDist >= 6.5)
        //     return;
        
        if(Math.abs(this.getAngularVelocityGyro()) >= 135) // 90
            return;

        // if(this.getVelocityOdometry() >= 4)
        //     return;

        // LimelightHelpers.printPoseEstimate(limelightMeasurement);

        // If any targets are in view
        if (leftPoseEst.tagCount > 0 && !rejectLeft) {
            double xyStds = 0.9; //0.5
            double degStds = 100000;
            swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
            // swervePoseEstimator.addVisionMeasurement(
            //     new Pose2d(leftPoseEst.pose.getTranslation(), leftPoseEst.pose.getRotation()),
            //                 leftPoseEst.timestampSeconds);
        }

        if (rightPoseEst.tagCount > 0 && !rejectRight) {
            double xyStds = 0.9; //0.5
            double degStds = 100000;
            swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
            // swervePoseEstimator.addVisionMeasurement(
            //     new Pose2d(rightPoseEst.pose.getTranslation(), rightPoseEst.pose.getRotation()),
            //                 rightPoseEst.timestampSeconds);
        }

        if (frontPoseEst.tagCount > 0 && !rejectFront) {
            double xyStds = 0.9;
            double degStds = 100000;
            swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));

            Pose3d visionPose3d = new Pose3d(frontPoseEst.pose);
            var r = Units.inchesToMeters(Constants.Limelight.limelightRadius);
            var z1 = Units.inchesToMeters(Constants.Limelight.pivotZOffset);
            var z2 = r * rc.s_Pitch.getPitchAngle().getCos();
            var rxy = r * rc.s_Pitch.getPitchAngle().getSin();
            var x = rxy * rc.s_Turret.getTurretAngle().getCos();
            var y = rxy * rc.s_Turret.getTurretAngle().getSin();
            Rotation3d camMountDerotation = new Rotation3d(0, -Units.degreesToRadians(Constants.Limelight.camToShooterFrameAngle) - rc.s_Pitch.getPitchAngle().getRadians(), -rc.s_Turret.getTurretToChassis().getRadians());
            Transform3d camToTurretFrame = new Transform3d(x, -y, -(z1 + z2), camMountDerotation);
            Transform3d turretToBotFrame = new Transform3d(Constants.Limelight.turretToRobotCenter.getX(), Constants.Limelight.turretToRobotCenter.getY(), -Constants.Limelight.floorToTurret, new Rotation3d());
            Transform3d botToFieldFrame = new Transform3d(0, 0, 0, new Rotation3d(0, 0, getGyroYaw().getRadians()));
            Pose2d estimatedRobotPose = visionPose3d.transformBy(camToTurretFrame).transformBy(turretToBotFrame).transformBy(botToFieldFrame).toPose2d();

            SmartDashboard.putNumber("Drive Xe Adjusted", estimatedRobotPose.getX());
            SmartDashboard.putNumber("Drive Ye Adjusted", estimatedRobotPose.getY());

            swervePoseEstimator.addVisionMeasurement(
                new Pose2d(estimatedRobotPose.getTranslation(), estimatedRobotPose.getRotation()),
                            frontPoseEst.timestampSeconds);
        }
    }

    @Override
    public void periodic(){

        // updateOdometry();

        // SmartDashboard.putNumber("Angular Velocity", getAngularVelocity());

        m_field.setRobotPose(getPose());

        SmartDashboard.putNumber("gyro", getGyroYaw().getDegrees());
        // ChassisSpeeds currentSpeeds = getRobotRelativeSpeeds();
        // SmartDashboard.putNumber("RobotAngularVelocityOdometry", currentSpeeds.omegaRadiansPerSecond);
        // SmartDashboard.putNumber("RobotAngularVelocityGyro", getAngularVelocityGyro());
        // SmartDashboard.putNumber("RobotVelocityOdometry", getVelocityOdometry());

        // SmartDashboard.putNumber("Swerve0 Rotations", mSwerveMods[0].getRotations());
        // SmartDashboard.putNumber("Swerve1 Rotations", mSwerveMods[1].getRotations());
        // SmartDashboard.putNumber("Swerve2 Rotations", mSwerveMods[2].getRotations());
        // SmartDashboard.putNumber("Swerve3 Rotations", mSwerveMods[3].getRotations());f

        SmartDashboard.putNumber("RobotPose X", swervePoseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("RobotPose Y", swervePoseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("RobotPose angle", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("gyroRate", getAngularVelocityGyro());

        m_field.setRobotPose(this.getPose());

        // SmartDashboard.putNumber("Distance Estimate Inches", Units.metersToInches(getTranslationToSpeaker().getNorm()));
        SmartDashboard.putNumber("Distance Estimate", getTranslationToSpeaker().getNorm());
        // SmartDashboard.putNumber("Angle Estimate", getTranslationToSpeaker().getAngle().getDegrees());

        for(SwerveModule mod : mSwerveMods){
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

}