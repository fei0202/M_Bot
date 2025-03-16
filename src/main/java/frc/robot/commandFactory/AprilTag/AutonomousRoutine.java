package frc.robot.commandFactory.AprilTag;

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
