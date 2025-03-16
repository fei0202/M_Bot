package frc.robot.Teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CombinedControlConstants;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends Command {
    private final Intake intake;
    private final XboxController joystick1;
    private final Timer intakeTimer = new Timer();
    private boolean isIntakeActive = false;
    private boolean isContinuousIntake = false;
    // private static final double CURRENT_LIMIT =
    // CombinedControlConstants.INTAKE_CURRENT;

    public TeleopIntake(Intake intake, XboxController joystick1) {
        this.intake = intake;
        this.joystick1 = joystick1;
        addRequirements(intake);
    }

    @Override
    public void execute() {

        // if (joystick1.getYButton()) {
        // intake.setIntakeSpeed(CombinedControlConstants.PLACE_SPEED);
        // intakeTimer.reset();
        // intakeTimer.start();
        // isIntakeActive = true;
        // } else if (joystick1.getAButton()) {
        // intake.setIntakeSpeed(CombinedControlConstants.INTAKING_SPEED);
        // intakeTimer.reset();
        // intakeTimer.start();
        // isIntakeActive = true;
        // }
        // if (isIntakeActive && intakeTimer.get() >=
        // CombinedControlConstants.Intake_TIME) {
        // intake.stopIntake();
        // isIntakeActive = false;
        // }

        // 7back
        if (joystick1.getRawButton(7) || joystick1.getBButton() || joystick1.getXButton()) {
            intake.stopIntake();
        }

        if (joystick1.getAButton()) {
            intake.setIntakeSpeed(CombinedControlConstants.INTAKING_SPEED);
        } else if (joystick1.getYButton()) {
            intake.setIntakeSpeed(CombinedControlConstants.PLACE_SPEED);
        } else {
            intake.setIntakeSpeed(0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}