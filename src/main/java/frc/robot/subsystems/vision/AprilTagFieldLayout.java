package frc.robot.subsystems.vision;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AprilTagFieldLayout {

    // Map to store AprilTag IDs and their corresponding field positions
    private final Map<Integer, Pose2d> tagPositions;

    public AprilTagFieldLayout() {
        tagPositions = new HashMap<>();
        initializeFieldLayout();
    }

    /**
     * Initializes the field layout with AprilTag IDs and their positions.
     * These positions are based on the 2025 field layout.
     */
    private void initializeFieldLayout() {
        // Example positions (replace with actual 2025 field data)
        tagPositions.put(1, new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(0)));
        tagPositions.put(2, new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(90)));
        tagPositions.put(3, new Pose2d(5.0, 6.0, Rotation2d.fromDegrees(180)));
        tagPositions.put(4, new Pose2d(7.0, 8.0, Rotation2d.fromDegrees(270)));
        // Add more tags as needed
    }

    /**
     * Gets the position of an AprilTag by its ID.
     *
     * @param tagId The ID of the AprilTag.
     * @return The Pose2d of the tag, or null if the tag ID is not found.
     */
    public Pose2d getTagPosition(int tagId) {
        return tagPositions.get(tagId);
    }

    /**
     * Checks if a given AprilTag ID exists in the layout.
     *
     * @param tagId The ID of the AprilTag.
     * @return True if the tag exists, false otherwise.
     */
    public boolean hasTag(int tagId) {
        return tagPositions.containsKey(tagId);
    }
    /**
    * Gets the position of an AprilTag if it exists.
     *
     * @param tagId The ID of the AprilTag.
     * @return An Optional containing the Pose2d of the tag if it exists, or an empty Optional otherwise.
     */
    public Optional<Pose2d> getTagPose(int tagId) {
    return Optional.ofNullable(tagPositions.get(tagId));
    }

}
