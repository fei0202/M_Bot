package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.FSLib2025.utils.LocalADStarAK;
import frc.robot.Constants.RobotConstants;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    private NetworkTable photonTable;

    public Robot() {
        super(RobotConstants.PERIODIC_INTERVAL);
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStarAK());
        CameraServer.startAutomaticCapture();
        photonTable = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("MyCamera");
        DataLogManager.start();
        DataLogManager.logNetworkTables(false);

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        boolean hasTarget = photonTable.getEntry("hasTarget").getBoolean(false);
        double targetYaw = photonTable.getEntry("targetYaw").getDouble(0.0);
        double targetPitch = photonTable.getEntry("targetPitch").getDouble(0);

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        // new PathPlannerAuto("L1").schedule();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        CommandScheduler.getInstance().cancelAll();

    }

    @Override
    public void teleopPeriodic() {
    }
}
