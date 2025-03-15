package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class ChaseTagCommand extends Command {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private final Swerve drivetrainSubsystem;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

    private final NetworkTable limelightTable;
    private final NetworkTableEntry tx, ty, ta;

    private boolean hasTarget = false;

    public ChaseTagCommand(Swerve drivetrainSubsystem, Supplier<Pose2d> poseProvider) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.poseProvider = poseProvider;

        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limelightTable.getEntry("tx"); // X 偏移角度
        ty = limelightTable.getEntry("ty"); // Y 偏移角度
        ta = limelightTable.getEntry("ta"); // 目標的面積 (可用於距離估計)

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Math.toRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        var robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        double xOffset = tx.getDouble(0.0); // X 偏移
        double yOffset = ty.getDouble(0.0); // Y 偏移
        double targetArea = ta.getDouble(0.0); // 目標面積 (越大代表距離越近)

        if (targetArea > 0) { // 代表有偵測到 AprilTag
            if (!hasTarget) { // 只在第一次偵測到目標時設定 PID 目標
                xController.setGoal(1); // 目標是 x 偏移為 0
                yController.setGoal(1); // 目標是 y 偏移為 0
                omegaController.setGoal(0); // 讓機器人朝向 AprilTag
                hasTarget = true;
            }

            // 計算 PID 輸出
            double xSpeed = xController.calculate(xOffset);
            double ySpeed = yController.calculate(yOffset);
            double omegaSpeed = omegaController.calculate(poseProvider.get().getRotation().getRadians());

            // 檢查是否達到目標
            if (xController.atGoal())
                xSpeed = 0;
            if (yController.atGoal())
                ySpeed = 0;
            if (omegaController.atGoal())
                omegaSpeed = 0;

            // **修改這裡** 使用 Translation2d 和旋轉速度
            drivetrainSubsystem.drive(new Translation2d(xSpeed, ySpeed), omegaSpeed, true);
        } else {
            hasTarget = false;
            drivetrainSubsystem.stop(); // 如果沒有偵測到 AprilTag，則停止機器人
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }
}
