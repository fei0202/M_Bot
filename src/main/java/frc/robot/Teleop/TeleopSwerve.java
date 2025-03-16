package frc.robot.Teleop;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commandFactory.PathplannerFactory;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {
    private final Swerve swerve;
    private final CommandXboxController joystick;
    private final XboxController joystick2;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(2.5);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(2.5);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(2);

    private double xSpeed = 0.0;
    private double ySpeed = 0.0;
    private double rotSpeed = 0.0;

    private boolean isRedAlliance = false; // dufault blue

    private final Pose2d[] blueWaypoints = {
            new Pose2d(1.403, 6.980, Rotation2d.fromDegrees(-0)),
            new Pose2d(2.649, 4.199, Rotation2d.fromDegrees(0)),
            new Pose2d(2.673, 3.851, Rotation2d.fromDegrees(0)),
            new Pose2d(3.428, 2.533, Rotation2d.fromDegrees(60)),
            new Pose2d(3.692, 2.305, Rotation2d.fromDegrees(60)),
            new Pose2d(5.263, 2.365, Rotation2d.fromDegrees(120)),
            new Pose2d(5.610, 2.485, Rotation2d.fromDegrees(120)),
            new Pose2d(6.377, 3.863, Rotation2d.fromDegrees(-180)),
            new Pose2d(6.389, 4.199, Rotation2d.fromDegrees(-180)),
            new Pose2d(5.490, 5.529, Rotation2d.fromDegrees(-120)),
            new Pose2d(5.275, 5.793, Rotation2d.fromDegrees(-120)),
            new Pose2d(3.773, 5.614, Rotation2d.fromDegrees(-60)),
            new Pose2d(3.476, 5.458, Rotation2d.fromDegrees(-60)),
            new Pose2d(1.415, 1.094, Rotation2d.fromDegrees(-30)),
    };

    private final Pose2d[] redWaypoints = {
            new Pose2d(6.807, 6.980, Rotation2d.fromDegrees(180)),
            new Pose2d(5.561, 4.199, Rotation2d.fromDegrees(180)),
            new Pose2d(5.537, 3.851, Rotation2d.fromDegrees(180)),
            new Pose2d(4.782, 2.533, Rotation2d.fromDegrees(120)),
            new Pose2d(4.518, 2.305, Rotation2d.fromDegrees(120)),
            new Pose2d(2.947, 2.365, Rotation2d.fromDegrees(60)),
            new Pose2d(2.600, 2.485, Rotation2d.fromDegrees(60)),
            new Pose2d(1.833, 3.863, Rotation2d.fromDegrees(0)),
            new Pose2d(1.821, 4.199, Rotation2d.fromDegrees(0)),
            new Pose2d(2.720, 5.529, Rotation2d.fromDegrees(60)),
            new Pose2d(2.935, 5.793, Rotation2d.fromDegrees(60)),
            new Pose2d(4.437, 5.614, Rotation2d.fromDegrees(120)),
            new Pose2d(4.734, 5.458, Rotation2d.fromDegrees(120)),
            new Pose2d(6.795, 1.094, Rotation2d.fromDegrees(210)),
    };

    public TeleopSwerve(Swerve swerve, CommandXboxController joystick, XboxController joystick2) {
        this.swerve = swerve;
        this.joystick = joystick;
        this.joystick2 = joystick2;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (joystick.getHID().getRawButtonPressed(8)) {
            swerve.setGyroYaw(0);
            swerve.setOdometryPosition(new Pose2d());
        }
        // B
        if (joystick.getHID().getRawButtonPressed(2)) {
            isRedAlliance = !isRedAlliance;
            System.out.println("change into " + (isRedAlliance ? "Red" : "Blue"));
        }

        Pose2d[] activeWaypoints = isRedAlliance ? redWaypoints : blueWaypoints;
        for (int i = 0; i < 14; i++) {
            if (joystick2.getRawButtonPressed(i + 1)) {
                driveToWaypoint(activeWaypoints, i);
            }
        }

        double joystickMagnitude = Math.hypot(joystick.getLeftX(), joystick.getLeftY());
        double reduction = 0.3 + 0.7 * Math.pow(joystickMagnitude, 3);

        xSpeed = xLimiter.calculate(-joystick.getLeftY()) * reduction * 0.8;
        ySpeed = yLimiter.calculate(-joystick.getLeftX()) * reduction * 0.8;

        double rightX = joystick.getRightX();
        rotSpeed = Math.abs(rightX) > 0.2 ? rotLimiter.calculate(-rightX) * 0.6 : 0;

        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);

        swerve.drive(
                new Translation2d(xSpeed, ySpeed).times(SwerveConstants.MAX_MODULE_SPEED),
                rotSpeed * SwerveConstants.MAX_MODULE_ROTATIONAL_SPEED,
                true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, false);
    }

    private void driveToWaypoint(Pose2d[] waypoints, int index) {
        Pose2d targetPose = waypoints[index];
        System.out.println("set" + targetPose);
        Command pathCommand = PathplannerFactory.driveToSetpointCommand(targetPose);
        if (pathCommand != null) {
            pathCommand.schedule();
        } else {
            System.err.println("can't set " + targetPose);
        }
    }
}