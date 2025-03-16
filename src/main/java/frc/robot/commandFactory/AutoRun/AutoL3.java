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

public class AutoL3 extends SequentialCommandGroup {
        private final Swerve swerve;
        private final Intake intake;
        private final Elevator elevator;
        private final Arm arm;

        public AutoL3(Swerve swerve, Intake intake, Elevator elevator, Arm arm) {
                this.arm = arm;
                this.elevator = elevator;
                this.intake = intake;
                this.swerve = swerve;
                addRequirements(swerve);

                addCommands(
                                new RunCommand(() -> arm.setArmSpeed(CombinedControlConstants.ARM_SPEED), arm)
                                                .withTimeout(0.5),
                                new RunCommand(() -> elevator.setDesiredHeight(ElevatorConstants.ELEVATOR_L3_HEIGHT),
                                                arm)
                                                .withTimeout(2),
                                new RunCommand(() -> arm.setArmSpeed(CombinedControlConstants.ARM_SPEED), arm)
                                                .withTimeout(0.5),
                                new RunCommand(() -> intake.setIntakeSpeed(CombinedControlConstants.INTAKING_SPEED),
                                                intake)
                                                .withTimeout(1),

                                new InstantCommand(elevator::stop, elevator),
                                new InstantCommand(arm::stopArm, arm),
                                new InstantCommand(intake::stopIntake, intake),
                                new InstantCommand(() -> swerve.setOdometryPosition(new Pose2d()), swerve));

        }
}
