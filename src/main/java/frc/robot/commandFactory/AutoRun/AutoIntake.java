package frc.robot.commandFactory.AutoRun;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CombinedControlConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class AutoIntake extends SequentialCommandGroup {
        private final Elevator elevator;
        private final Swerve swerve;
        private final Arm arm;
        private final Intake intake;

        public AutoIntake(Swerve swerve, Intake intake, Elevator elevator, Arm arm) {
                this.arm = arm;
                this.elevator = elevator;
                this.intake = intake;
                this.swerve = swerve;
                addRequirements(swerve);

                addCommands(
                                new RunCommand(() -> elevator
                                                .setDesiredHeight(ElevatorConstants.ELEVATOR_CORAL_STATION_HEIGHT),
                                                elevator).withTimeout(2),
                                new RunCommand(() -> intake.setIntakeSpeed(CombinedControlConstants.INTAKING_SPEED),
                                                intake)
                                                .withTimeout(4),
                                new RunCommand(() -> intake.setIntakeSpeed(0),
                                                intake)
                                                .withTimeout(1),

                                new InstantCommand(elevator::stop, elevator),
                                new InstantCommand(arm::stopArm, arm),
                                new InstantCommand(intake::stopIntake, intake),

                                new InstantCommand(() -> swerve.setOdometryPosition(new Pose2d()), swerve));
        }
}
