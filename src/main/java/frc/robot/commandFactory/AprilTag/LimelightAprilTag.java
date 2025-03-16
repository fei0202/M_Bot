package frc.robot.commandFactory.AprilTag;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class LimelightAprilTag extends Command {
    private final Swerve swerve;
    private final NetworkTable limelightTable;
    public boolean left = true;

    public LimelightAprilTag(Swerve swerve) {
        this.swerve = swerve;
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double tagId = limelightTable.getEntry("tid").getDouble(-1);
        double xOffset = limelightTable.getEntry("tx").getDouble(0);
        double yOffset = limelightTable.getEntry("ty").getDouble(0);

        if (tagId != -1) {
            System.out.println("AprilTag ID: " + tagId);
            System.out.println("X offset: " + xOffset);
            System.out.println("Y offset: " + yOffset);

            if (tagId == 1 || tagId == 6 || tagId == 7 || tagId == 8 || tagId == 9 ||
                    tagId == 10 || tagId == 11 || tagId == 17 || tagId == 18 || tagId == 19 ||
                    tagId == 20 || tagId == 21 || tagId == 22) {

                if (!left) {
                    swerve.correctAlignment(xOffset, yOffset, "Right");
                } else {
                    swerve.correctAlignment(-xOffset, -yOffset, "Left");
                }

            } else if (tagId == 2 || tagId == 12 || tagId == 13) {
                swerve.correctAlignment(xOffset, yOffset, "CoralStationIntake");
            }
        } else {
            System.out.println("NO AprilTag");
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
