package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotVisualizer extends SubsystemBase {
    private LoggedMechanism2d robotVisualizer = new LoggedMechanism2d(2, 2.5);    
    private LoggedMechanismRoot2d robotRoot = robotVisualizer.getRoot("Robot Visualizer", 1, 0.1);
  

    public RobotVisualizer() {

    }

    @Override
    public void periodic() {
        Logger.recordOutput("RobotVisualizer", robotVisualizer);
        Pose2d fakePose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(7).get().toPose2d().plus(new Transform2d(1, 0, new Rotation2d(Math.PI)));
        Logger.recordOutput("Fake robot pose", fakePose);
    }
}
