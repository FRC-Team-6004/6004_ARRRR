package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import frc.robot.subsystems.vision.OfficialReefscapeFieldLayout;
import frc.robot.subsystems.vision.OfficialReefscapeFieldLayout.FieldType;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.Swerve;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("Front_Cam");
    private Optional<PhotonTrackedTarget> bestTarget = Optional.empty();
    private OfficialReefscapeFieldLayout lay= new OfficialReefscapeFieldLayout(FieldType.WELDED);
    private Swerve swerve= new Swerve(
      TunerConstants.DrivetrainConstants,
      50, // odometry update frequency
      TunerConstants.FrontLeft,
      TunerConstants.FrontRight,
      TunerConstants.BackLeft,
      TunerConstants.BackRight
  );

    public Vision() {}

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
 
        if (result.hasTargets()) {
            bestTarget = Optional.of(result.getBestTarget());
        } else {
            bestTarget = Optional.empty();
        }

        // Logging
        Logger.recordOutput("Vision/HasTarget", hasTarget());
        Logger.recordOutput("Vision/TargetID", getTargetID());
        Logger.recordOutput("Vision/TargetYaw", getTargetYaw());
        Logger.recordOutput("Vision/TargetPitch", getTargetPitch());
        Logger.recordOutput("Vision/TargetSkew", getTargetSkew());
         
    }

    /** Returns true if the camera sees a tag. */
    public boolean hasTarget() {
        return bestTarget.isPresent();
    }

    /** Returns yaw (horizontal offset in degrees) to the tag. */
    public double getTargetYaw() {
        return bestTarget.map(PhotonTrackedTarget::getYaw).orElse(0.0);
    }

    /** Returns pitch (vertical offset in degrees) to the tag. */
    public double getTargetPitch() {
        return bestTarget.map(PhotonTrackedTarget::getPitch).orElse(0.0);
    }

    /** Returns fiducial ID of the tag, or -1 if none. */
    public int getTargetID() {
        return bestTarget.map(PhotonTrackedTarget::getFiducialId).orElse(-1);
    }

    public double getTargetSkew() {
        return bestTarget.map(PhotonTrackedTarget::getSkew).orElse(0.0);
    }
    
    public double getRotOffset() {
        double tagrot = lay.getTagPose(getTargetID()).get().toPose2d().getRotation().getDegrees();
        double robotrot = Swerve.getInstance().getPose().getRotation().getDegrees();
        return (tagrot - robotrot);
    }
}
