package frc.robot.commandFactory.AutoRun;
// package frc.robot.commandFactory;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Vision;

// public class AutoIntakeByTag extends Command {
// private final Intake intake;
// private final Vision vision;
// private final Elevator Elevator;
// private final Arm arm;

// public AutoIntakeByTag(Intake intake, Elevator elevator, Arm arm, Vision
// vision) {
// this.intake = intake;
// this.vision = vision;
// this.arm = arm;
// this.elevator = elevator;
// addRequirements(intake, vision, arm, elevator);
// }

// @Override
// public void execute() {
// if (vision.hasTarget() && vision.getCurrentTagID() == 2) {
// intake.runIntake(); // 啟動 Intake
// } else {
// intake.stopIntake(); // 停止 Intake
// }
// }

// @Override
// public void end(boolean interrupted) {
// intake.stopIntake();
// }

// @Override
// public boolean isFinished() {
// return false; // 讓它持續運行
// }
// }
