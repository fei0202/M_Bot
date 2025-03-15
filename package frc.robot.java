package frc.robot.commandFactory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class Offset extends Command {
    private final Swerve swerve;
    private final double xSpeed, ySpeed;
    private final double duration;
    private final Timer timer = new Timer();

    public Offset(Swerve swerve, double xSpeed, double ySpeed, double duration) {
        this.swerve = swerve;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.duration = duration;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(xSpeed, ySpeed), 0, true);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true);
    }
}


package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.commandFactory.AlignToTag;
import frc.robot.commandFactory.AutoIntake;
import frc.robot.commandFactory.AutoL2;
import frc.robot.commandFactory.AutoL3;
import frc.robot.commandFactory.AutoL4;
import frc.robot.commandFactory.AutonomousRoutine;
import frc.robot.commandFactory.ForwardL1;
import frc.robot.commandFactory.Offset;
import frc.robot.commands.ButtonPath;
import frc.robot.commands.TeleopArm;
import frc.robot.commands.TeleopClimb;
import frc.robot.commands.TeleopElevator;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TestSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ObstacleDetect;
import frc.robot.subsystems.PathL;
import frc.robot.subsystems.PathR;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    public CommandXboxController joystick = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT0);
    public CommandXboxController joystick1 = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT1);
    public CommandXboxController joystick2 = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT2);

    private final SendableChooser<Command> autoChooser;

    private final Climb climb = new Climb();
    private final Intake intake = new Intake();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Swerve swerve = new Swerve();
    private final ObstacleDetect obstacledetector = new ObstacleDetect();
    private final Vision vision = new Vision();

    private final TeleopClimb teleopClimb = new TeleopClimb(climb, joystick.getHID());
    private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, joystick);
    private final TeleopElevator teleopElevator = new TeleopElevator(elevator, joystick1.getHID(), joystick.getHID());
    private final TestSwerve testSwerve = new TestSwerve(swerve);
    private final TeleopArm teleopArm = new TeleopArm(arm, joystick1.getHID(), joystick.getHID());
    private final TeleopIntake teleopIntake = new TeleopIntake(intake, joystick1.getHID());
    private final PathL pathL = new PathL(joystick2.getHID());
    private final PathR pathR = new PathR(joystick2.getHID());
    private final ButtonPath buttonPath = new ButtonPath(swerve, pathR, pathL, joystick2.getHID());

    public RobotContainer() {
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
        swerve.setDefaultCommand(buttonPath);
        swerve.setDefaultCommand(teleopSwerve);
        elevator.setDefaultCommand(teleopElevator);
        arm.setDefaultCommand(teleopArm);
        intake.setDefaultCommand(teleopIntake);
        climb.setDefaultCommand(teleopClimb);

        generateNamedCommands();

        loadPathPlannerAutos();

        SmartDashboard.putString("Selected Auto Path", autoChooser.getSelected().getName());
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureButtonBindings();
    }

    private void loadPathPlannerAutos() {
        File pathFolder = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");

        if (pathFolder.exists() && pathFolder.isDirectory()) {
            File[] files = pathFolder.listFiles((dir, name) -> name.endsWith(".auto"));
            if (files != null) {
                System.out.println("Found " + files.length + " auto files in pathplanner/autos:");
                for (File file : files) {
                    String pathName = file.getName().replace(".auto", "");
                    autoChooser.addOption(pathName, new PathPlannerAuto(pathName));
                    System.out.println("Added Auto Path: " + pathName);
                }
            } else {
                System.out.println("No .auto files found in pathplanner/autos directory!");
            }
        } else {
            System.out.println("PathPlanner autos directory not found: " + pathFolder.getAbsolutePath());
        }
    }

    public void configureButtonBindings() {
        joystick2.start().onTrue(
                new SequentialCommandGroup(
                        new AlignToTag(vision, swerve),
                        new Offset(swerve, 0.5, 0, 2)));
    }

    public Command getAutonomousCommand() {
        swerve.setGyroYaw(0);
        swerve.setOdometryPosition(new Pose2d());

        return autoChooser.getSelected();

        // return new AutonomousRoutine(swerve);
    }

    public void generateNamedCommands() {

        NamedCommands.registerCommand("AprilTagDetection", new AutonomousRoutine(vision, swerve));
        NamedCommands.registerCommand("ForwardL1", new ForwardL1(elevator, arm, swerve, intake));
        NamedCommands.registerCommand("INTAKE", new AutoIntake(elevator, arm, intake));
        NamedCommands.registerCommand("L2", new AutoL2(swerve, intake, elevator, arm));
        NamedCommands.registerCommand("L4", new AutoL4(swerve, intake, elevator, arm));
        NamedCommands.registerCommand("L3", new AutoL3(swerve, intake, elevator, arm));
    }
}
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final NetworkTable limelightTable;

    public Vision() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getTagYaw() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public double getTagDistance() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }
}
package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
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

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getGyroYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
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
}package frc.robot.commandFactory;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class LimelightAprilTag extends Command {
    private final Swerve swerve;
    private final NetworkTable limelightTable;
    private boolean right = false; // left

    public LimelightAprilTag(Swerve swerve) {
        this.swerve = swerve;
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double tagId = limelightTable.getEntry("tid").getDouble(-1);
        double xOffset = limelightTable.getEntry("tx").getDouble(0);
        double yOffset = limelightTable.getEntry("ty").getDouble(0);

        if (tagId != -1) {
            System.out.println("AprilTag ID: " + tagId);
            System.out.println("X offset: " + xOffset);
            System.out.println("Y offset: " + yOffset);

            if (tagId == 1 || tagId == 6 || tagId == 7 || tagId == 8 || tagId == 9 ||
                    tagId == 10 || tagId == 11 || tagId == 17 || tagId == 18 || tagId == 19 ||
                    tagId == 20 || tagId == 21 || tagId == 22) {

                if (!right) {
                    swerve.correctAlignment(xOffset, yOffset, "Left");
                } else {
                    swerve.correctAlignment(-xOffset, -yOffset, "Right");
                }

            } else if (tagId == 2 || tagId == 12 || tagId == 13) {
                swerve.correctAlignment(xOffset, yOffset, "CoralStationIntake");
            }
        } else {
            System.out.println("NO AprilTag");
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
package frc.robot.commandFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AutonomousRoutine extends SequentialCommandGroup {
  public AutonomousRoutine(Vision vision, Swerve swerve) {
    addCommands(
        new InstantCommand(() -> System.out.println("Start align")),

        new WaitCommand(0.5),
        new AlignToTag(vision, swerve),
        new WaitCommand(0.1),
        new LimelightAprilTag(swerve),

        new InstantCommand(() -> swerve.setGyroYaw(0), swerve),
        new InstantCommand(() -> swerve.setOdometryPosition(new Pose2d()), swerve),

        new InstantCommand(() -> System.out.println("End")));
  }
}
package frc.robot.commandFactory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AlignToTag extends Command {
    private final Vision vision;
    private final Swerve swerve;
    private final double kP = 0.02;

    public AlignToTag(Vision vision, Swerve swerve) {
        this.vision = vision;
        this.swerve = swerve;
        addRequirements(vision, swerve);
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double tx = vision.getTagYaw();
            double ty = vision.getTagDistance();

            swerve.correctAlignment(tx, ty, "rotate");
            if (Math.abs(tx) < 1.5) {
                swerve.correctAlignment(tx, ty, "translate");
            }
        }
    }

    @Override
    public boolean isFinished() {
        return vision.hasTarget() && Math.abs(vision.getTagYaw()) < 1.5;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true);

    }
}
