// package frc.robot.subsystems;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.commandFactory.PathplannerFactory;

// public class PathController extends SubsystemBase {
// private final Swerve swerve;
// private final XboxController controller;

// private final Pose2d[] waypoints = {
// new Pose2d(1, 1, Rotation2d.fromDegrees(0)),
// new Pose2d(2, 1, Rotation2d.fromDegrees(30)),
// new Pose2d(3, 1, Rotation2d.fromDegrees(60)),
// new Pose2d(1, 2, Rotation2d.fromDegrees(90)),
// new Pose2d(2, 2, Rotation2d.fromDegrees(120)),
// new Pose2d(3, 2, Rotation2d.fromDegrees(150)),
// new Pose2d(1, 3, Rotation2d.fromDegrees(180)),
// new Pose2d(2, 3, Rotation2d.fromDegrees(210)),
// new Pose2d(3, 3, Rotation2d.fromDegrees(240)),
// new Pose2d(1, 4, Rotation2d.fromDegrees(270)),
// new Pose2d(2, 4, Rotation2d.fromDegrees(300)),
// new Pose2d(3, 4, Rotation2d.fromDegrees(330))
// };

// public PathController(Swerve swerve, XboxController controller) {
// this.swerve = swerve;
// this.controller = controller;
// }

// @Override
// public void periodic() {
// for (int i = 0; i < 12; i++) {
// int buttonNumber = i + 1;
// if (controller.getRawButtonPressed(buttonNumber)) {
// driveToWaypoint(i);
// }
// }

// if (controller.getRawButtonPressed(13)) {
// swerve.setOdometryPosition(new Pose2d(1.5, 3.0, new Rotation2d(Math.PI)));
// System.out.println("reset！");
// }
// }

// private void driveToWaypoint(int index) {
// Pose2d targetPose = waypoints[index];
// System.out.println("set：" + targetPose);
// Command pathCommand = PathplannerFactory.driveToSetpointCommand(targetPose);

// if (pathCommand != null) {
// pathCommand.schedule();
// } else {
// System.err.println("nocommadn " + targetPose);
// }
// }
// }
