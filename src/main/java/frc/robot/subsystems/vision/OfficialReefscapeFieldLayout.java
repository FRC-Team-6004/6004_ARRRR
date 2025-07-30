package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.io.IOException;
import java.util.Optional;

public class OfficialReefscapeFieldLayout {

    private final AprilTagFieldLayout wpilibLayout;

    public enum FieldType {
        WELDED,
        ANDYMARK
    }

    /**
     * Loads the official 2025 Reefscape layout JSON automatically.
     * @param type Welded or AndyMark field version
     */
    public OfficialReefscapeFieldLayout(FieldType type) {
        AprilTagFields selected = (type == FieldType.WELDED)
            ? AprilTagFields.k2025ReefscapeWelded
            : AprilTagFields.k2025ReefscapeAndyMark;
        wpilibLayout = AprilTagFieldLayout.loadField(selected);
        // Optionally adjust the origin if on red alliance:
        // wpilibLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
    }

    /**
     * Returns the official Pose3d of a tag, if known.
     */
    public Optional<Pose3d> getTagPose(int tagId) {
        return wpilibLayout.getTagPose(tagId);
    }

    public static AprilTagFieldLayout load() {
        try {
            return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
        } catch (IOException e) {
            throw new RuntimeException("Failed to load Reefscape layout", e);
        }
    }

    /**
     * Returns the full WPILib layout, e.g., for multi-tag fusion APIs.
     */
    public AprilTagFieldLayout getWpilibLayout() {
        return wpilibLayout;
    }
}
