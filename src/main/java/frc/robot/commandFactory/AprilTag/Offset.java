package frc.robot.commandFactory.AprilTag;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class Offset extends Command {
    private final Swerve swerve;
    private final double xSpeed, ySpeed;
    private final double duration;
    private final Timer timer = new Timer();

    public Offset(Swerve swerve, double xSpeed, double ySpeed, double duration) {
        this.swerve = swerve;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.duration = duration;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(xSpeed, ySpeed), 0, true);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true);
    }
}
