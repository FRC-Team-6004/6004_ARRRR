
package frc.robot.commands;

import frc.robot.constants;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout; // Add this import for AprilTagFieldLayout
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.OIConstants;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.AprilTag.Vision;
import frc.robot.util.LoggedTunableNumber; 

public class barebonesvision extends Command {
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final double kCameraHeight = 0.5; // Example value in meters, update as needed
    private final edu.wpi.first.wpilibj.ADXRS450_Gyro gyro = new edu.wpi.first.wpilibj.ADXRS450_Gyro(); // Example gyro initialization
    private PhotonPoseEstimator photonPoseEstimator; // Declare photonPoseEstimator as a class member
    private static final double kTargetHeight = 2.0; // Example value in meters, update as needed
    private static final double kCameraPitch = 0.0; // Example value in radians, update as needed
    private static final double kTargetPitch = 0.0; // Example value in radians, update as needed
    private final PhotonCamera camera = new PhotonCamera("Front Left");
    private final frc.robot.subsystems.vision.AprilTagFieldLayout aprilTagFieldLayout = new frc.robot.subsystems.vision.AprilTagFieldLayout(); // Initialize appropriately
    private final Transform3d cameraToRobot = new Transform3d(
        new Translation3d(0.2, 0.0, 0.0), // Example translation (x = 0.2m, y = 0.0m, z = 0.0m)
        new Rotation3d(0.0, 0.0, 0.0)     // Example rotation (roll = 0, pitch = 0, yaw = 0 radians)
    );

    @Override
    public void initialize() {
        // Initialization logic here
    }

    private final Pigeon2 pigeon = new Pigeon2(0); // Initialize Pigeon2 with appropriate CAN ID

    @Override
    public void execute() {
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = result.getBestTarget();
        double yaw = target.getYaw();
        double pitch = target.getPitch();
        double area = target.getArea();
        double skew = target.getSkew();
        Transform2d pose = new Transform2d(
            target.getBestCameraToTarget().getTranslation().toTranslation2d(),
            target.getBestCameraToTarget().getRotation().toRotation2d()
        );
        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
             Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                 target.getBestCameraToTarget(),
                 new Pose3d(
                     aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getX(),
                     aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getY(),
                     0.0, // Assuming z = 0.0, update as needed
                     new Rotation3d()
                 ),
                 cameraToRobot
             );
        }
        // Calculate robot's field relative pose
        double cameraHeight = kCameraHeight;
        double targetHeight = kTargetHeight;
        double cameraPitch = kCameraPitch;
        double targetPitch = kTargetPitch;
        Rotation2d targetYaw = Rotation2d.fromDegrees(-target.getYaw());
        Rotation2d robotYaw = new Rotation2d(Math.toRadians(pigeon.getYaw().getValueAsDouble())); // Convert yaw to radians
        Pose2d robotPose = new Pose2d(
            target.getBestCameraToTarget().getTranslation().toTranslation2d(),
            target.getBestCameraToTarget().getRotation().toRotation2d()
        );
        Transform2d cameraToRobotTransform = new Transform2d(
            cameraToRobot.getTranslation().toTranslation2d(),
            cameraToRobot.getRotation().toRotation2d()
        );
        Pose2d targetPose = new Pose2d(
            target.getBestCameraToTarget().getTranslation().toTranslation2d(),
            target.getBestCameraToTarget().getRotation().toRotation2d()
        );
        double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, targetPose);
        // Calculate a translation from the camera to the target.
        double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
            kCameraHeight, kTargetHeight, kCameraPitch, Units.degreesToRadians(target.getPitch())
        );
        Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
            distanceMeters, Rotation2d.fromDegrees(-target.getYaw()));
        
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); // Ensure proper initialization

        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)); // Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        photonPoseEstimator.setReferencePose(new Pose2d()); // Example: Set a reference pose if needed
        photonPoseEstimator.setRobotToCameraTransform(robotToCam);
        photonPoseEstimator.setRobotToCameraTransform(robotToCam);
        
        // Set driver mode to on.
        camera.setDriverMode(true);

        // Change pipeline to 2
        camera.setPipelineIndex(2);

        if (hasTargets && targetID != -1) {
            // Calculate the angle to turn towards the target
            double angleToTurn = MathUtil.angleModulus(Math.toRadians(yaw));

            // Use a simple proportional controller to turn towards the target
            double kP = 0.02; // Proportional gain, tune this value as needed
            double turnSpeed = MathUtil.clamp(kP * angleToTurn, -0.5, 0.5); // Clamp speed to avoid excessive turning

            // Command the robot to turn

            // Define driveSpeed, direction, and omega
            double driveSpeed = 0.0; // Set to 0.0 since we are only turning
            Rotation2d direction = new Rotation2d(0.0); // No translational movement
            double omega = turnSpeed; // Use the calculated turnSpeed for rotational rate
            
            Swerve.getInstance().setControl(
                drive.withVelocityX(driveSpeed * direction.getCos())
                .withVelocityY(driveSpeed * direction.getSin())
                .withRotationalRate(omega));
            
        } else {
            // Stop the robot if no valid target is found
            Swerve.getInstance().stop();
        }

    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(camera.getLatestResult());
    }

    @Override
    public void end(boolean interrupted) {
        // Cleanup logic here
    }

    @Override
    public boolean isFinished() {
        return false; // Update this condition as needed
    }
}