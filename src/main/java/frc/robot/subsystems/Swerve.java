package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private SwerveModule[] modules = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.RF_CONSTANTS),
            new SwerveModule(1, SwerveConstants.LF_CONSTANTS),
            new SwerveModule(2, SwerveConstants.LB_CONSTANTS),
            new SwerveModule(3, SwerveConstants.RB_CONSTANTS) };
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_TRANSLATOIN_METERS);
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getGyroYaw(), getModulePositions());

    private Field2d field = new Field2d();

    public Swerve() {

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (config == null) {
            throw new RuntimeException("Failed to load config");
        }

        AutoBuilder.configure(
                this::getOdometryPosition,
                this::setOdometryPosition,
                this::getRobotRelativSpeeds,
                (speeds, feedforwards) -> driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(10.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });

        odometry.resetPose(new Pose2d());

        gyro.reset();
    }

    // !
    public void correctAlignment(double x, double y, String mode) {
        if (mode.equals("rotate")) {
            double turnSpeed = 0.02 * x;
            if (Math.abs(x) < 1.5) {
                turnSpeed = 0;
            }
            drive(new Translation2d(0, 0), turnSpeed, true);
        } else if (mode.equals("translate")) {
            double forwardSpeed = 0.5 * y;
            if (Math.abs(y) < 0.1) {
                forwardSpeed = 0;
            }
            drive(new Translation2d(forwardSpeed, 0), 0, true);
        }
    }

    public void followPath(int index) {
        System.out.println("Following path: " + index);
        PathPlannerAuto autoPath = new PathPlannerAuto("Path" + index);
        autoPath.schedule();
    }

    public void stop() {
        System.out.println("Stopping Swerve movement.");
        drive(new Translation2d(0, 0), 0, true);
    }

    // public void drive(Translation2d translation, double rotation, boolean
    // fieldRelative) {
    // System.out.println("Driving: X=" + translation.getX() + ", Y=" +
    // translation.getY() + ", Rot=" + rotation);
    // ChassisSpeeds chassisSpeeds = fieldRelative
    // ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(),
    // translation.getY(), rotation, getGyroYaw())
    // : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    // SwerveModuleState[] moduleStates =
    // kinematics.toSwerveModuleStates(chassisSpeeds);
    // setModuleStates(moduleStates);
    // }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        double maxSpeed = 3.0;
        Translation2d limitedTranslation = translation.times(0.5);
        double limitedRotation = rotation * 0.5;

        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(limitedTranslation.getX(), limitedTranslation.getY(),
                        limitedRotation, getGyroYaw())
                : new ChassisSpeeds(limitedTranslation.getX(), limitedTranslation.getY(), limitedRotation);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.01);
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public void setGyroYaw(double yaw) {
        gyro.reset();
        gyro.setAngleAdjustment(yaw);
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public Pose2d getOdometryPosition() {
        return odometry.getPoseMeters();
    }

    public Pose2d getCurrentPose() {
        return odometry.getPoseMeters();
    }

    public void setOdometryPosition(Pose2d pose) {
        odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public ChassisSpeeds getRobotRelativSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(getModuleStates()), getGyroYaw());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getModuleState();
        }
        return states;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (desiredStates.length != 4) {
            throw new IllegalArgumentException("desiredStates must have length 4");
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_MODULE_SPEED);
        for (SwerveModule mod : modules) {
            mod.setDesiredState(desiredStates[mod.ModuleNumber]);
        }
    }

    @Override
    public void periodic() {
        odometry.update(getGyroYaw(), getModulePositions());
        field.setRobotPose(getOdometryPosition());

        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("gyro (deg)", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("swerve odometry x", getOdometryPosition().getX());
        SmartDashboard.putNumber("swerve odometry y", getOdometryPosition().getY());

    }

    public AHRS getGyro() {
        return gyro;
    }
}