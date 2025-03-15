package frc.robot;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.commandFactory.AutonomousRoutine;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ObstacleDetect;
import frc.robot.subsystems.Swerve; // 引入 ChaseTagCommand
import frc.robot.subsystems.Vision;

public class RobotContainer {
    public CommandXboxController joystick = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT0);
    public CommandXboxController joystick1 = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT1);
    public CommandXboxController joystick2 = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT2);

    private final SendableChooser<Command> autoChooser;

    private final Swerve swerve = new Swerve();
    private final ObstacleDetect obstacledetector = new ObstacleDetect();
    private final Vision vision = new Vision();

    private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, joystick);

    // **新增 ChaseTagCommand**
    private final Supplier<Pose2d> poseSupplier = () -> swerve.getOdometryPosition();
    private final ChaseTagCommand chaseTagCommand = new ChaseTagCommand(swerve, poseSupplier);

    public RobotContainer() {
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Do Nothing", new InstantCommand());

        swerve.setDefaultCommand(teleopSwerve);

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
        joystick.a().onTrue(chaseTagCommand);
    }

    public Command getAutonomousCommand() {
        swerve.setGyroYaw(0);
        swerve.setOdometryPosition(new Pose2d());

        return autoChooser.getSelected();
    }

    public void generateNamedCommands() {
        NamedCommands.registerCommand("AprilTagDetection", new AutonomousRoutine(vision, swerve));

        NamedCommands.registerCommand("ChaseTag", chaseTagCommand);
    }
}
