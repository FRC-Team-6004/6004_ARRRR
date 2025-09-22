
/* 
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Vision;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class Pathing extends SubsystemBase {

    private final SwerveDrivePoseEstimator poseEstimator;
    private final AutoBuilder autoBuilder;
    private final Vision vision;

    // Low-pass filter state
    private Pose2d lastFilteredVision = null;

    // Reefscape field dimensions (meters)
    private static final double FIELD_LENGTH_METERS = 17.54;
    private static final double FIELD_WIDTH_METERS = 8.05;

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
        // Update odometry elsewhere via updateOdometry()

        if (vision != null) {
            vision.getLatestFieldPose().ifPresent(visionPose -> {
                double timestamp = vision.getPoseTimestamp();
                Pose2d currentPose = poseEstimator.getEstimatedPosition();

                // --- Sanity checks ---
                boolean insideField =
                        visionPose.getX() >= 0 && visionPose.getX() <= FIELD_LENGTH_METERS &&
                        visionPose.getY() >= 0 && visionPose.getY() <= FIELD_WIDTH_METERS;

                double jumpDistance = currentPose.getTranslation().getDistance(visionPose.getTranslation());
                boolean closeEnough = jumpDistance < 2.0; // reject jumps > 2 meters

                // Logging for debugging
                Logger.recordOutput("Pathing/OdometryPose", currentPose);
                Logger.recordOutput("Pathing/VisionPose", visionPose);
                Logger.recordOutput("Pathing/JumpDistance", jumpDistance);
                Logger.recordOutput("Pathing/VisionTimestamp", timestamp);
                Logger.recordOutput("Pathing/FPGATimestamp", Timer.getFPGATimestamp());
                Logger.recordOutput("Pathing/InsideField", insideField);
                Logger.recordOutput("Pathing/CloseEnough", closeEnough);

                if (insideField && closeEnough) {
                    // Low-pass filter for smoothing
                    double alpha = 0.2;
                    if (lastFilteredVision == null) {
                        lastFilteredVision = visionPose;
                    } else {
                        lastFilteredVision = new Pose2d(
                                lastFilteredVision.getX() + alpha * (visionPose.getX() - lastFilteredVision.getX()),
                                lastFilteredVision.getY() + alpha * (visionPose.getY() - lastFilteredVision.getY()),
                                lastFilteredVision.getRotation().plus(
                                        visionPose.getRotation().minus(lastFilteredVision.getRotation()).times(alpha)
                                )
                        );
                    }

                    poseEstimator.addVisionMeasurement(lastFilteredVision, timestamp);
                }
            });
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
        return autoBuilder.pathfindToPose(targetPose, constraints);
    }
}
*/