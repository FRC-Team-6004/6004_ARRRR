package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Vision;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;

public class Pathing extends SubsystemBase {

    private final SwerveDrivePoseEstimator poseEstimator;
    private final AutoBuilder autoBuilder;
    private final Vision vision;

    public Pathing(
            SwerveDriveKinematics kinematics,
            Rotation2d initialGyroAngle,
            SwerveModulePosition[] initialModulePositions,
            Pose2d initialPose,
            AutoBuilder autoBuilder,
            Vision vision
    ) {
        var stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.01);
        var visionStdDevs = VecBuilder.fill(0.1, 0.1, 0.05);

        this.poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                initialGyroAngle,
                initialModulePositions,
                initialPose,
                stateStdDevs,
                visionStdDevs
        );

        this.autoBuilder = autoBuilder;
        this.vision = vision;
    }

    @Override
    public void periodic() {
        // Update odometry from drivetrain (call updateOdometry somewhere else)

        // Get vision updates automatically
        if (vision != null) {
            Pose2d visionPose = vision.getLatestFieldPose2d();
            double timestamp = vision.getPoseTimestamp();

            if (visionPose != null) {
                poseEstimator.addVisionMeasurement(visionPose, timestamp);
            }
        }
    }

    public void addPhotonVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
    }

    public void updateOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(gyroAngle, modulePositions);
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(gyroAngle, modulePositions, newPose);
    }

    public Command pathfindToPose(Pose2d targetPose, PathConstraints constraints) {
        return autoBuilder.pathfindToPose(targetPose, constraints); // call instance method
    }
}
