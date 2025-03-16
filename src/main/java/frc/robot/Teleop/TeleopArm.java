package frc.robot.Teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.CombinedControlConstants;
import frc.robot.subsystems.Arm;

public class TeleopArm extends Command {

    private final Arm arm;
    private final XboxController joystick1;
    private final XboxController joystick;
    private final double ANGLE_CHANGE_LIMIT = 0.02;
    private static final double ARM_MIN_ANGLE = 0.0;
    private static final double ARM_MAX_ANGLE = 1.0;
    private double armAngleTarget;

    public TeleopArm(Arm arm, XboxController joystick1, XboxController joystick) {
        this.arm = arm;
        this.joystick1 = joystick1;
        this.joystick = joystick;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.stopArm();
    }

    @Override
    public void execute() {
        // if (joystick.getRawButton(7)) {
        // arm.stopArm();
        // return;
        // }

        // if (joystick.getRawButtonPressed(5)) {
        // armAngleTarget = CombinedControlConstants.ARM_DEFAULT_OFFSET;
        // }
        // if (joystick.getRawButtonPressed(6)) {
        // armAngleTarget = CombinedControlConstants.ARM_CORAL_STATION_OFFSET;
        // }

        // double joystickY = -joystick1.getLeftY();
        // double angleChange = joystickY * ANGLE_CHANGE_LIMIT;

        // if (Math.abs(joystickY) > 0.4) {
        // armAngleTarget = MathUti3l.clamp(armAngleTarget + angleChange, ARM_MIN_ANGLE,
        // ARM_MAX_ANGLE);
        // }

        // arm.setArmAngle(armAngleTarget);

        if (joystick1.getRightY() > 0.5) {
            arm.setArmSpeed(CombinedControlConstants.ARM_SPEED);
        } else if (joystick1.getRightY() < -0.5) {
            arm.setArmSpeed(-CombinedControlConstants.ARM_BACKSPEED);
        } else {
            arm.setArmSpeed(0);
        }

        if (joystick1.getRawButtonPressed(3)) {
            new RunCommand(() -> arm.setArmSpeed(CombinedControlConstants.ARM_AUTO_SPEED), arm)
                    .withTimeout(0.3)
                    .andThen(() -> arm.setArmSpeed(0))
                    .schedule();
        }

        if (joystick1.getRawButtonPressed(2)) {
            new RunCommand(() -> arm.setArmSpeed(-CombinedControlConstants.ARM_AUTO_SPEED), arm)
                    .withTimeout(0.3)
                    .andThen(() -> arm.setArmSpeed(0))
                    .schedule();
        }

    }

    @Override
    public void end(boolean interrupted) {
        arm.stopArm();
    }
}