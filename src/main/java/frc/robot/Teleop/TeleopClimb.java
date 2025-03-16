package frc.robot.Teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CombinedControlConstants;
import frc.robot.subsystems.Climb;

public class TeleopClimb extends Command {
    private final Climb climb;
    private final XboxController joystick;

    public TeleopClimb(Climb climb, XboxController joystick) {
        this.climb = climb;
        this.joystick = joystick;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        // LB5 RB6 climb
        if (joystick.getRawButton(5)) {
            climb.setClimbSpeed(CombinedControlConstants.CLIMB_SPEED);
        } else if (joystick.getRawButton(6)) {
            climb.setClimbSpeed(-CombinedControlConstants.CLIMB_SPEED);
        } else {
            climb.stopClimb();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimb();
    }
}
