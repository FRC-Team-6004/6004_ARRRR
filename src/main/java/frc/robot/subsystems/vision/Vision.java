package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("Front_Cam");
    private Optional<PhotonTrackedTarget> bestTarget = Optional.empty();

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

}
