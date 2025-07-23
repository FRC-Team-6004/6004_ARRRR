package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.OfficialReefscapeFieldLayout;

import java.util.Optional;

public class Vision2 extends SubsystemBase {

    private Pose3d currentRobotPose = new Pose3d();  // Real-time robot pose

    private final PhotonCamera leftCamera = new PhotonCamera("Front Left");
    private final PhotonCamera rightCamera = new PhotonCamera("Front Right");

    private final Transform3d leftCameraOffset = new Transform3d(
        new Translation3d(-0.2956, 0.2572, 0.2),
        new Rotation3d(0.0, Math.toRadians(15), Math.toRadians(-60))
    );

    private final Transform3d rightCameraOffset = new Transform3d(
        new Translation3d(0.295, 0.2508, 0.2),
        new Rotation3d(0.0, Math.toRadians(15), Math.toRadians(60))
    );

    private final AprilTagFieldLayout fieldLayout;

    public Vision2() {
        fieldLayout = new OfficialReefscapeFieldLayout(
            OfficialReefscapeFieldLayout.FieldType.WELDED
        ).getWpilibLayout();    }

    @Override
    public void periodic() {
        Optional<Pose3d> leftPose = getEstimatedPoseFromCamera(leftCamera, leftCameraOffset);
        Optional<Pose3d> rightPose = getEstimatedPoseFromCamera(rightCamera, rightCameraOffset);

        if (leftPose.isPresent() && rightPose.isPresent()) {
            // Simple average if both are available (more sophisticated fusion is also possible)
            currentRobotPose = averagePose3d(leftPose.get(), rightPose.get());
        } else if (leftPose.isPresent()) {
            currentRobotPose = leftPose.get();
        } else if (rightPose.isPresent()) {
            currentRobotPose = rightPose.get();
        }
        // else keep the previous pose
    }

    private Optional<Pose3d> getEstimatedPoseFromCamera(PhotonCamera camera, Transform3d cameraOffset) {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) return Optional.empty();

        PhotonTrackedTarget bestTarget = result.getBestTarget();
        int fiducialId = bestTarget.getFiducialId();

        Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(fiducialId);
        if (tagPoseOptional.isEmpty()) return Optional.empty();

        Pose3d tagPose = tagPoseOptional.get();
        Transform3d camToTag = bestTarget.getBestCameraToTarget();  // 3D transform

        // Invert the transform to get Camera pose in field space
        Pose3d cameraPose = tagPose.transformBy(camToTag.inverse());

        // Transform camera pose to robot pose using known offset
        Pose3d robotPose = cameraPose.transformBy(cameraOffset.inverse());

        return Optional.of(robotPose);
    }

    public Pose3d getCurrentRobotPose() {
        return currentRobotPose;
    }

    private Pose3d averagePose3d(Pose3d a, Pose3d b) {
        Translation3d avgTranslation = new Translation3d(
            (a.getX() + b.getX()) / 2.0,
            (a.getY() + b.getY()) / 2.0,
            (a.getZ() + b.getZ()) / 2.0
        );

        Rotation3d avgRotation = new Rotation3d(
            (a.getRotation().getX() + b.getRotation().getX()) / 2.0,
            (a.getRotation().getY() + b.getRotation().getY()) / 2.0,
            (a.getRotation().getZ() + b.getRotation().getZ()) / 2.0
        );

        return new Pose3d(avgTranslation, avgRotation);
    }
}
