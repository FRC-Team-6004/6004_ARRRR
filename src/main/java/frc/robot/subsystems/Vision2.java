package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.Optional;
import frc.robot.subsystems.vision.OfficialReefscapeFieldLayout; // Ensure this is the correct package for the class




    // Initialize Vision2 with the provided field layout

public class Vision2 {

    // Cameras
    private final PhotonCamera leftCamera = new PhotonCamera("Front Left");
    private final PhotonCamera rightCamera = new PhotonCamera("Front Right");

    private OfficialReefscapeFieldLayout fieldLayout;

    // Camera offsets relative to the robot's center (adjust as needed)
    private final Transform3d leftCameraOffset = new Transform3d(
        new Translation3d(-11.625, 10.125, 7.875),
        new Rotation3d(0.0, Math.toRadians(15), Math.toRadians(-60))
    );

    private final Transform3d rightCameraOffset = new Transform3d(
        new Translation3d(-11.625, 9.875, 7.875),
        new Rotation3d(0.0, Math.toRadians(15), Math.toRadians(60))
    );

    public Vision2(OfficialReefscapeFieldLayout fieldLayout) {
        this.fieldLayout = fieldLayout;
    }

    /**
     * Gets the robot's field position using AprilTags detected by the cameras.
     *
     * @return An Optional containing the robot's Pose3d on the field, or empty if no valid tag is detected.
     */
    public Optional<Pose3d> getFieldPosition() {
        Optional<Pose3d> leftPose = getCameraPose(leftCamera, leftCameraOffset);
        Optional<Pose3d> rightPose = getCameraPose(rightCamera, rightCameraOffset);

        if (leftPose.isPresent()) {
            return leftPose;
        } else if (rightPose.isPresent()) {
            return rightPose;
        }

        return Optional.empty();
    }

    /**
     * Gets the robot's pose from a specific camera.
     *
     * @param camera The PhotonCamera to use.
     * @param cameraOffset The Transform3d offset of the camera relative to the robot's center.
     * @return An Optional containing the robot's Pose3d, or empty if no valid tag is detected.
     */
    private Optional<Pose3d> getCameraPose(PhotonCamera camera, Transform3d cameraOffset) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            int tagId = target.getFiducialId();

            Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(tagId);
            if (tagPoseOptional.isPresent()) {
                Pose3d tagPose = tagPoseOptional.get();
                Transform3d cameraToTag = target.getBestCameraToTarget();

                // Compute the robot's pose on the field
                Pose3d robotPose = tagPose.transformBy(cameraToTag.inverse())
                                           .transformBy(cameraOffset.inverse());

                return Optional.of(robotPose);
            }
        }

        return Optional.empty();
    }
}
