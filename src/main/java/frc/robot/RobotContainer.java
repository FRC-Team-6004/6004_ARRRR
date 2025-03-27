// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.GenericRequirement;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.vision.AprilTag.Vision;
import frc.robot.subsystems.vision.AprilTag.VisionIOReal;
import frc.robot.subsystems.vision.AprilTag.VisionIOSim;
import frc.robot.util.NamedCommandManager;

public class RobotContainer {
  private RobotVisualizer visualizer;
  private Vision vision;

  // private final Vision vision;
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(SwerveConstants.MaxSpeed * 0.04).withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.04) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

  public final Swerve drivetrain;

  public int autoScoreMode = 1;

  LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() throws IOException, ParseException {
    GenericRequirement.initialize();
    switch (Constants.currentMode) {
      case REAL:
        drivetrain = Swerve.initialize(new Swerve(TunerConstants.DrivetrainConstants, 50, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight));
        vision = Vision.initialize(
          new VisionIOReal(0), 
          new VisionIOReal(1)
        );  
        break;

      case SIM:
        drivetrain = Swerve.initialize(TunerConstants.createDrivetrain());
        visualizer = new RobotVisualizer();
        if(Constants.visonSimEnabled) {
          vision = Vision.initialize(new VisionIOSim());
        }
        break;

      default:
        drivetrain = Swerve.initialize(TunerConstants.createDrivetrain());
        break;
    }
    

    NamedCommandManager.registerNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser("Driver Forward Straight"));
    
    configureBindings();
    
  }

  private void configureBindings() {
    // Drive command
    drivetrain.setDefaultCommand(
      drivetrain
          .applyRequest(() -> drive.withVelocityX(-Constants.OIConstants.driverController.getLeftY() * SwerveConstants.MaxSpeed * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier : 1))
              .withVelocityY(-Constants.OIConstants.driverController.getLeftX() * SwerveConstants.MaxSpeed * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier : 1))
              .withRotationalRate(-Constants.OIConstants.driverController.getRightX() * SwerveConstants.MaxAngularRate * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier : 1))));

    // Zero heading
    Constants.OIConstants.driverController.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    

    // Algae align
    Constants.OIConstants.driverController.rightBumper().whileTrue(
      AutoCommands.alignAlgae()
    );

    // Slow mode
    Constants.OIConstants.driverController.leftTrigger().onTrue(Commands.runOnce(() -> drivetrain.setSlowMode(true)));
    Constants.OIConstants.driverController.leftTrigger().onFalse(Commands.runOnce(() -> drivetrain.setSlowMode(false)));

      
    // Change reef scoring stem
    OIConstants.driverController.x().onTrue(
        Commands.runOnce(() -> drivetrain.setScoringLeft()
      ));
    OIConstants.driverController.y().onTrue(
        Commands.runOnce(() -> drivetrain.setScoringRight()
      ));



    

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
     return autoChooser.get();
  }

  public void periodic() {
    Logger.recordOutput("Score/isLeftL4", OIConstants.autoScoreMode == 4 && OIConstants.isScoringLeft);
    Logger.recordOutput("Score/isLeftL3", OIConstants.autoScoreMode == 3 && OIConstants.isScoringLeft);
    Logger.recordOutput("Score/isLeftL2", OIConstants.autoScoreMode == 2 && OIConstants.isScoringLeft);
    Logger.recordOutput("Score/isRightL4", OIConstants.autoScoreMode == 4 && !OIConstants.isScoringLeft);
    Logger.recordOutput("Score/isRightL3", OIConstants.autoScoreMode == 3 && !OIConstants.isScoringLeft);
    Logger.recordOutput("Score/isRightL2", OIConstants.autoScoreMode == 2 && !OIConstants.isScoringLeft);
  }
}
