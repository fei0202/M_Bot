package frc.robot.Teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class TeleopElevator extends Command {
    private final Elevator elevator;
    private final XboxController joystick;
    private final XboxController joystick1;
    private final XboxController joystick2;
    private double targetHeight;
    private final double HEIGHT_CHANGE_LIMIT = 0.5;

    public TeleopElevator(Elevator elevator, XboxController joystick, XboxController joystick1,
            XboxController joystick2) {
        this.elevator = elevator;
        this.joystick = joystick;
        this.joystick1 = joystick1;
        this.joystick2 = joystick2;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double joystickY = -joystick1.getLeftY();
        double heightChange = joystickY * HEIGHT_CHANGE_LIMIT;

        if (Math.abs(joystickY) > 0.4) {
            targetHeight += heightChange;
        }

        if (joystick2.getRawButtonPressed(15)) {
            targetHeight = ElevatorConstants.ELEVATOR_DEFAULT_HEIGHT;
        }
        if (joystick2.getRawButtonPressed(16)) {
            targetHeight = ElevatorConstants.ELEVATOR_CORAL_STATION_HEIGHT;
        }
        if (joystick2.getRawButtonPressed(17)) {
            targetHeight = ElevatorConstants.ELEVATOR_L2_HEIGHT;
        }
        if (joystick2.getRawButtonPressed(18)) {
            targetHeight = ElevatorConstants.ELEVATOR_L3_HEIGHT;
        }
        if (joystick2.getRawButtonPressed(19)) {
            targetHeight = ElevatorConstants.ELEVATOR_L4_HEIGHT;
        }

        elevator.setDesiredHeight(targetHeight);

        if (joystick.getRawButtonPressed(7) || joystick1.getRawButtonPressed(7) || joystick2.getRawButtonPressed(20)) {
            targetHeight = elevator.getLeftPosition();
            // elevator.stop();
        }

        SmartDashboard.putNumber("Target LElevator Height", targetHeight);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

}