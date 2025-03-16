package frc.robot.commandFactory.AprilTag;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AlignToTag extends Command {
    private final Vision vision;
    private final Swerve swerve;
    private final double kP = 0.02;

    public AlignToTag(Vision vision, Swerve swerve) {
        this.vision = vision;
        this.swerve = swerve;
        addRequirements(vision, swerve);
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double tx = vision.getTagYaw();
            double ty = vision.getTagDistance();

            swerve.correctAlignment(tx, ty, "rotate");
            if (Math.abs(tx) < 1.5) {
                swerve.correctAlignment(tx, ty, "translate");
            }
        }
    }

    @Override
    public boolean isFinished() {
        return vision.hasTarget() && Math.abs(vision.getTagYaw()) < 1.5;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true);

    }
}
