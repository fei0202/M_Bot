package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.Teleop.TeleopArm;
import frc.robot.Teleop.TeleopClimb;
import frc.robot.Teleop.TeleopElevator;
import frc.robot.Teleop.TeleopIntake;
import frc.robot.Teleop.TeleopSwerve;
import frc.robot.commandFactory.AprilTag.AutonomousRoutine;
import frc.robot.commandFactory.AutoRun.AutoIntake;
import frc.robot.commandFactory.AutoRun.AutoL2;
import frc.robot.commandFactory.AutoRun.AutoL3;
import frc.robot.commandFactory.AutoRun.AutoL4;
import frc.robot.commandFactory.AutoRun.ForwardL1;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    public CommandXboxController joystick = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT0);
    public CommandXboxController joystick1 = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT1);
    public XboxController joystick2 = new XboxController(RobotConstants.DRIVE_CONTROLLER_PORT2);

    private final SendableChooser<Command> autoChooser;

    private final Climb climb = new Climb();
    private final Intake intake = new Intake();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Swerve swerve = new Swerve();
    private final Vision vision = new Vision();

    private final TeleopClimb teleopClimb = new TeleopClimb(climb, joystick.getHID());
    private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, joystick, joystick2);
    private final TeleopElevator teleopElevator = new TeleopElevator(elevator, joystick.getHID(), joystick1.getHID(),
            joystick2);
    private final TeleopArm teleopArm = new TeleopArm(arm, joystick1.getHID(), joystick.getHID());
    private final TeleopIntake teleopIntake = new TeleopIntake(intake, joystick1.getHID());

    public RobotContainer() {

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Do Nothing", new InstantCommand());

        swerve.setDefaultCommand(teleopSwerve);
        elevator.setDefaultCommand(teleopElevator);
        arm.setDefaultCommand(teleopArm);
        intake.setDefaultCommand(teleopIntake);
        climb.setDefaultCommand(teleopClimb);

        // swerve.setOdometryPosition(new Pose2d(8, 7,Rotation2d.fromDegrees(0)));
        generateNamedCommands();

        loadPathPlannerAutos();

        SmartDashboard.putString("Selected Auto Path", autoChooser.getSelected().getName());
        SmartDashboard.putData("Auto Chooser", autoChooser);
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

    public Command getAutonomousCommand() {
        swerve.setGyroYaw(0);
        swerve.setOdometryPosition(new Pose2d());
        return autoChooser.getSelected();

        // return new AutonomousRoutine(swerve);
    }

    public void generateNamedCommands() {
        NamedCommands.registerCommand("AprilTagDetection", new AutonomousRoutine(vision, swerve));
        NamedCommands.registerCommand("ForwardL1", new ForwardL1(elevator, arm, swerve, intake));
        NamedCommands.registerCommand("INTAKE", new AutoIntake(swerve, intake, elevator, arm));
        NamedCommands.registerCommand("L2", new AutoL2(swerve, intake, elevator, arm));
        NamedCommands.registerCommand("L4", new AutoL4(swerve, intake, elevator, arm));
        NamedCommands.registerCommand("L3", new AutoL3(swerve, intake, elevator, arm));
    }
}
