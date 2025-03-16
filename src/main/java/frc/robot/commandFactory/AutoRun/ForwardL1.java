package frc.robot.commandFactory.AutoRun;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CombinedControlConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class ForwardL1 extends SequentialCommandGroup {
        private final Elevator elevator;
        private final Arm arm;
        private final Swerve swerve;
        private final Intake intake;

        public ForwardL1(Elevator elevator, Arm arm, Swerve swerve, Intake intake) {
                this.elevator = elevator;
                this.arm = arm;
                this.swerve = swerve;
                this.intake = intake;
                addRequirements(elevator, arm, swerve, intake);

                addCommands(
                                new InstantCommand(() -> swerve.setGyroYaw(0), swerve),
                                new InstantCommand(() -> swerve.setOdometryPosition(new Pose2d()), swerve),

                                new RunCommand(() -> arm.setArmSpeed(CombinedControlConstants.ARM_SPEED), arm)
                                                .withTimeout(2),
                                new RunCommand(() -> arm.setArmSpeed(0), arm)
                                                .withTimeout(1),
                                new RunCommand(() -> intake.setIntakeSpeed(CombinedControlConstants.PLACE_SPEED),
                                                intake)
                                                .withTimeout(0.8),
                                new RunCommand(() -> intake.setIntakeSpeed(0),
                                                intake)
                                                .withTimeout(2),
                                new RunCommand(() -> arm.setArmSpeed(-CombinedControlConstants.ARM_BACKSPEED), arm)
                                                .withTimeout(2),
                                new RunCommand(() -> swerve.drive(new Translation2d(-0.5, 0), 0, false), swerve)
                                                .withTimeout(0.5),

                                new InstantCommand(elevator::stop, elevator),
                                new InstantCommand(arm::stopArm, arm),
                                new InstantCommand(intake::stopIntake, intake));
        }
}
