

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;

public class AutoAlignAndDrive extends Command {
    private final Swerve swerve;
    private final Vision vision;

    private final PIDController turnPID = new PIDController(0.02, 0, 0.001);
    private final PIDController forwardPID = new PIDController(0.05, 0, 0);

    private final double desiredPitch = 15.0; // pitch at "scoring distance"

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    public AutoAlignAndDrive(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve, vision);

        turnPID.setTolerance(1.0);     // degrees
        forwardPID.setTolerance(1.0);  // pitch tolerance
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double yaw = vision.getTargetYaw();
            double pitch = vision.getTargetPitch();

            // Convert PID output into speeds
            double omega = MathUtil.clamp(turnPID.calculate(yaw, 0.0), -1.0, 1.0);
            double vx = MathUtil.clamp(forwardPID.calculate(pitch, desiredPitch), -1.0, 1.0);

            // Scale by your robot's max velocity constants
            vx *= frc.robot.subsystems.swerve.SwerveConstants.MaxSpeed; // m/s
            omega *= frc.robot.subsystems.swerve.SwerveConstants.MaxAngularRate; // rad/s

            // Apply with Phoenix
            swerve.setControl(
                driveRequest
                    .withVelocityX(vx)     // forward (m/s)
                    .withVelocityY(0.0)    // no strafe
                    .withRotationalRate(omega) // rad/s
            );
        } else {
            swerve.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return vision.hasTarget()
            && turnPID.atSetpoint()
            && forwardPID.atSetpoint();
    }
}
