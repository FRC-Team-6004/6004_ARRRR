package frc.robot.subsystems.vision.AprilTag;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class VisionConstants {
    // CHANGE THESE?
    public static final double fieldBorderMargin = 0.5;
    public static final double zMargin = 0.5;

    // public static final double xyStdDevSingleTag = 0.03;
    // public static final double xyStdDevMultiTag = 0.018;

    public static final double xyStdDevSingleTag = 0.08;
    public static final double xyStdDevMultiTag = 0.04;

    private static LoggedTunableNumber targetPoseXOffset = new LoggedTunableNumber("AutoAlign/targetPoseXOffset", .48);

    public static final String[] cameraIds =
    new String[] {
        "Front Right",
        "Front Left"
      };

    // Based on sim. Change once actually mounted
    public static final Transform3d[] cameraPoses =
    new Transform3d[] {
        // Front Camera Right
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(10),
                Units.inchesToMeters(-12),
                Units.inchesToMeters(5)), 
            new Rotation3d(
                Units.degreesToRadians(0), 
                Units.degreesToRadians(0), 
                Units.degreesToRadians(0))), 

        // Front Camera Left
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(10.5),
                Units.inchesToMeters(11),
                Units.inchesToMeters(5)), 
            new Rotation3d(
                Units.degreesToRadians(0), 
                Units.degreesToRadians(0), 
                Units.degreesToRadians(0)))

        };

    public static AprilTagFieldLayout aprilTagFieldLayout;

    static {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
            aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static final int[] redReefIds = {6, 7, 8, 9, 10, 11};
    public static final int[] blueReefIds = {17, 18, 19, 20, 21, 22};

    public static final int[] allReefIds = {6,7,8,9,10,11,17,18,19,20,21,22};

    public static final int[] nonReefIds = {1,2,3,4,5,12,13,14,15,16};

    private static final Pose2d[] redReefTag =
    new Pose2d[] {
        aprilTagFieldLayout.getTagPose(6).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(7).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(8).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(9).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(10).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(11).get().toPose2d()
    };


    public static final Pose2d[] redReefScoringPoses =
    new Pose2d[] {
        redReefTag[0].plus(new Transform2d(targetPoseXOffset.get(), 0, new Rotation2d(Math.PI))),
        redReefTag[1].plus(new Transform2d(targetPoseXOffset.get(), 0, new Rotation2d(Math.PI))),
        redReefTag[2].plus(new Transform2d(targetPoseXOffset.get(), 0, new Rotation2d(Math.PI))),
        redReefTag[3].plus(new Transform2d(targetPoseXOffset.get(), 0, new Rotation2d(Math.PI))),
        redReefTag[4].plus(new Transform2d(targetPoseXOffset.get(), 0, new Rotation2d(Math.PI))),
        redReefTag[5].plus(new Transform2d(targetPoseXOffset.get(), 0, new Rotation2d(Math.PI))),
    };

    private static final Pose2d[] blueReefTag =
    new Pose2d[] {
        aprilTagFieldLayout.getTagPose(17).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(18).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(19).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(20).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(21).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(22).get().toPose2d()
    };

    
    public static final Pose2d[] blueReefScoringPoses =
    new Pose2d[] {
        blueReefTag[0].plus(new Transform2d(targetPoseXOffset.get(), 0, new Rotation2d(Math.PI))),
        blueReefTag[1].plus(new Transform2d(targetPoseXOffset.get(), 0, new Rotation2d(Math.PI))),
        blueReefTag[2].plus(new Transform2d(targetPoseXOffset.get(), 0, new Rotation2d(Math.PI))),
        blueReefTag[3].plus(new Transform2d(targetPoseXOffset.get(), 0, new Rotation2d(Math.PI))),
        blueReefTag[4].plus(new Transform2d(targetPoseXOffset.get(), 0, new Rotation2d(Math.PI))),
        blueReefTag[5].plus(new Transform2d(targetPoseXOffset.get(), 0, new Rotation2d(Math.PI))),
    };

    public static final double areaCutoff = 11;
    public static final double timeDelay = 0.5;
    public static final double intakeCamOffset = 0.3;
    public static final double degreeTolerance = 6.5;
}