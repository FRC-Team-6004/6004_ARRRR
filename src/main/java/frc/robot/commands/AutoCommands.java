package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.GenericRequirement;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.util.LoggedTunableNumber;

public class AutoCommands {
    private static Swerve drive = Swerve.getInstance();

    private static LoggedTunableNumber maxAccel = new LoggedTunableNumber("AutoAlign/maxAccel", 3.2);


    public static Command alignAlgae() { 
        return new AutoAlignAlgae( 
            new ProfiledPIDController(5,
            0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)), 
            new ProfiledPIDController(7.5, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3))
       );
    }

    public static Command alignReefUntil() {
        return new AutoAlignReef(
            new ProfiledPIDController(5,
             0, 0, new Constraints(SwerveConstants.MaxSpeed, maxAccel.get())), 
            new ProfiledPIDController(7.5, 0, 0, new Constraints(SwerveConstants.MaxAngularRate, 3))
        ).until(() -> OIConstants.aligned);
    }

    public static Command scoreLeftReef() {
        return Commands.runOnce(() -> Swerve.getInstance().setScoringLeft());
    }

    public static Command scoreRightReef() {
        return Commands.runOnce(() -> Swerve.getInstance().setScoringRight());
    }
}