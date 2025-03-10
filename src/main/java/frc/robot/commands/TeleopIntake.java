package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CombinedControlConstants;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends Command {
    private final Intake intake;
    private final XboxController joystick1;
    private Timer intakeTimer = new Timer();
    private boolean isIntakeActive = false;
    private static final double CURRENT_LIMIT = 35.0;

    public TeleopIntake(Intake intake, XboxController joystick1) {
        this.intake = intake;
        this.joystick1 = joystick1;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (intake.getIntakeCurrent() > CURRENT_LIMIT) {
            intake.stopIntake();
            isIntakeActive = false;
            return;
        }

        // Y A Intake
        if (joystick1.getYButton()) {
            intake.setIntakeSpeed(CombinedControlConstants.PLACE_SPEED);
            intakeTimer.reset();
            intakeTimer.start();
            isIntakeActive = true;
        } else if (joystick1.getAButton()) {
            intake.setIntakeSpeed(CombinedControlConstants.INTAKING_SPEED);
            intakeTimer.reset();
            intakeTimer.start();
            isIntakeActive = true;
        }

        if (isIntakeActive && intakeTimer.get() >= CombinedControlConstants.Intake_TIME) {
            intake.stopIntake();
            isIntakeActive = false;
        }

        // MOVE Intake
        // if (joystick1.getRawButton(4)) { // Y PLACE
        // intake.setIntakeSpeed(CombinedControlConstants.PLACE_SPEED);
        // } else if (joystick1.getRawButton(1)) { // A IN
        // intake.setIntakeSpeed(CombinedControlConstants.INTAKING_SPEED);
        // } else {
        // intake.stopIntake();
        // }
    }
}
