package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class strafe extends Command {
    private final Swerve swerve;
    private final double targetSpeed;    // m/s (positive = right, negative = left)
    private final double duration;       // seconds
    private final Timer timer = new Timer();
    
    private double currentSpeed = 0.0;   // for manual ramp
    private final double rampRate = 1.0; // m/s per second

    public strafe(Swerve swerve, double speed, double duration) {
        this.swerve = swerve;
        this.targetSpeed = speed;
        this.duration = duration;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        currentSpeed = 0.0;
    }

    @Override
    public void execute() {
        // Gradually ramp up velocity to target
        double dt = 0.02; // approximate loop time
        if (currentSpeed < targetSpeed) {
            currentSpeed += rampRate * dt;
            if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
        } else if (currentSpeed > targetSpeed) {
            currentSpeed -= rampRate * dt;
            if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
        }

        swerve.applyRequest(() -> 
        new SwerveRequest.FieldCentric()
            .withVelocityX(0.0)
            .withVelocityY(currentSpeed)
            .withRotationalRate(0.0)
    );
    
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        timer.stop();
    }
}
