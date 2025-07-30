package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.OfficialReefscapeFieldLayout;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.swerve.*;
import java.util.Optional;
import java.util.stream.Collectors;

public class Vision2 extends SubsystemBase {
    private final PhotonCamera leftCam = new PhotonCamera("Front_Left");
    private final PhotonCamera rightCam = new PhotonCamera("Front_Right");

    private final Transform3d robotToLeftCam = new Transform3d(
        new edu.wpi.first.math.geometry.Translation3d(-0.2953, 0.2572, 0.2000),
        new edu.wpi.first.math.geometry.Rotation3d(0.0, Math.toRadians(20), Math.toRadians(15))
    );

    private final Transform3d robotToRightCam = new Transform3d(
        new edu.wpi.first.math.geometry.Translation3d(0.2953, 0.2508, 0.2000),
        new edu.wpi.first.math.geometry.Rotation3d(0.0, Math.toRadians(20), Math.toRadians(-15))
    );

    private final PhotonPoseEstimator leftEstimator;
    private final PhotonPoseEstimator rightEstimator;

    private Optional<EstimatedRobotPose> latestPose = Optional.empty();

    public Vision2() {
        var fieldLayout = OfficialReefscapeFieldLayout.load();

        leftEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToLeftCam);
        rightEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToRightCam);
    }

    @Override
    public void periodic() {
        Pose2d currentPose = Swerve.getInstance().getPose();

        leftEstimator.setReferencePose(currentPose);
        rightEstimator.setReferencePose(currentPose);

        PhotonPipelineResult leftRaw = leftCam.getLatestResult();
        PhotonPipelineResult rightRaw = rightCam.getLatestResult();

        Logger.recordOutput("Vision2/Left/RawNumTargets", leftRaw.getTargets().size());
        Logger.recordOutput("Vision2/Right/RawNumTargets", rightRaw.getTargets().size());
        leftCam.getLatestResult().getTargets().forEach(t ->
    Logger.recordOutput("Vision2/Left/RawTagID", t.getFiducialId()));
    leftCam.getLatestResult().getTargets().forEach(t ->
    Logger.recordOutput("Vision2/Left/Ambiguity", t.getPoseAmbiguity()));

        var leftResult = leftEstimator.update(leftRaw);
        var rightResult = rightEstimator.update(rightRaw);

        if (leftResult.isPresent() && rightResult.isPresent()) {
            if (leftResult.get().targetsUsed.size() >= rightResult.get().targetsUsed.size()) {
                latestPose = leftResult;
            } else {
                latestPose = rightResult;
            }
        } else if (leftResult.isPresent()) {
            latestPose = leftResult;
        } else if (rightResult.isPresent()) {
            latestPose = rightResult;
        }

        log();
    }

    public Optional<EstimatedRobotPose> getLatestEstimatedPose() {
        return latestPose;
    }

    public Optional<Pose2d> getLatestFieldPose() {
        return latestPose.map(pose -> pose.estimatedPose.toPose2d());
    }

    public boolean seesTag() {
        return leftCam.getLatestResult().hasTargets() || rightCam.getLatestResult().hasTargets();
    }

    public double getPoseTimestamp() {
        return latestPose.map(pose -> pose.timestampSeconds).orElse(Timer.getFPGATimestamp());
    }

    private void log() {
        Logger.recordOutput("Vision2/HasTag", seesTag());

        if (latestPose.isPresent()) {
            var estPose = latestPose.get();
            Logger.recordOutput("Vision2/Pose2d", estPose.estimatedPose.toPose2d());
            Logger.recordOutput("Vision2/Latency", Timer.getFPGATimestamp() - estPose.timestampSeconds);
            Logger.recordOutput("Vision2/NumTargetsUsed", estPose.targetsUsed.size());
        } else {
            Logger.recordOutput("Vision2/Pose2d", new Pose2d());
            Logger.recordOutput("Vision2/NumTargetsUsed", 0);
        }
    }
}
