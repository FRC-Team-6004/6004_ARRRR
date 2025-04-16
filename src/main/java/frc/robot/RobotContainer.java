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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.OIConstants;
import frc.robot.commands.AlgaeHold;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.Barge;
import frc.robot.commands.ElevatorSetPos1;
import frc.robot.commands.ElevatorSetPos2;
import frc.robot.commands.ElevatorSetPos3;
import frc.robot.commands.ElevatorSetPos4;
import frc.robot.commands.ElevatorSetPos5;
import frc.robot.commands.ElevatorSetPos6;
import frc.robot.commands.GrabIn;
import frc.robot.commands.GrabOut;
import frc.robot.commands.PivotPos1;
import frc.robot.commands.PivotPos2;
import frc.robot.commands.PivotPos3;
import frc.robot.commands.PivotTimedRev;
import frc.robot.commands.ThrowForFun;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GenericRequirement;
import frc.robot.subsystems.GrabSub;
import frc.robot.subsystems.PivotSub;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.vision.AprilTag.Vision;
import frc.robot.subsystems.vision.AprilTag.VisionIOReal;
import frc.robot.subsystems.vision.AprilTag.VisionIOSim;
import frc.robot.util.NamedCommandManager;

public class RobotContainer {
  private RobotVisualizer visualizer;
  private Vision vision;
  
  
  private final Elevator elevatorSubsystem = new Elevator();
  private CommandXboxController op = new CommandXboxController(1);
    public final PivotSub pivotSubsystem = new PivotSub();
    public final GrabSub grabSubsystem = new GrabSub();


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
    switch (constants.currentMode) {
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
        if(constants.visonSimEnabled) {
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
          .applyRequest(() -> drive.withVelocityX(-constants.OIConstants.driverController.getLeftY() * .9 * SwerveConstants.MaxSpeed * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier : 1))
              .withVelocityY(-constants.OIConstants.driverController.getLeftX() * 0.3 * SwerveConstants.MaxSpeed * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier : 1))
              .withRotationalRate(-constants.OIConstants.driverController.getRightX() * .3 * SwerveConstants.MaxAngularRate * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier : 1))));

    // field center
    constants.OIConstants.driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    
    // Change reef scoring stem
    OIConstants.driverController.leftBumper().onTrue(
        Commands.runOnce(() -> drivetrain.setScoringLeft()
      ));
    OIConstants.driverController.rightBumper().onTrue(
        Commands.runOnce(() -> drivetrain.setScoringRight()
      ));

    // Right reef align
    constants.OIConstants.driverController.rightBumper().whileTrue(
      AutoCommands.alignReefUntil()
      );
    
    // Left reef align
    constants.OIConstants.driverController.leftBumper().whileTrue(
       AutoCommands.alignReefUntil()
    );

    // Algea align
   constants.OIConstants.driverController.a().whileTrue(
    AutoCommands.alignAlgae()
   );

    // Slow mode
    constants.OIConstants.driverController.rightTrigger(0.5).onTrue(Commands.runOnce(() -> drivetrain.setSlowMode(true)));
    constants.OIConstants.driverController.rightTrigger(0.5).onFalse(Commands.runOnce(() -> drivetrain.setSlowMode(false)));

   op.povDown().whileTrue(new ElevatorSetPos1(elevatorSubsystem));
   op.povLeft().whileTrue(new ElevatorSetPos2(elevatorSubsystem));
   op.povRight().whileTrue(new ElevatorSetPos3(elevatorSubsystem));
   op.povUp().whileTrue(new ElevatorSetPos4(elevatorSubsystem));
   op.povDown().whileTrue(new PivotPos1(pivotSubsystem));
   op.povLeft().whileTrue(new PivotPos1(pivotSubsystem));
   op.povRight().whileTrue(new PivotPos1(pivotSubsystem));
   op.povUp().whileTrue(new PivotPos2(pivotSubsystem));
   op.povDown().onFalse(new PivotTimedRev(pivotSubsystem));


    op.a().whileTrue(new ElevatorSetPos2(elevatorSubsystem));
    op.y().whileTrue(new ElevatorSetPos4(elevatorSubsystem));
    op.x().whileTrue(new ElevatorSetPos5(elevatorSubsystem));
    op.b().whileTrue(new ElevatorSetPos6(elevatorSubsystem));
    op.a().whileTrue(new PivotPos3(pivotSubsystem));
    op.b().whileTrue(new PivotPos3(pivotSubsystem));
    op.x().whileTrue(new PivotPos3(pivotSubsystem));
    op.y().whileTrue(new PivotPos3(pivotSubsystem));       
    //op.a().whileTrue(new AlgaeHold(grabSubsystem));
    //op.b().whileTrue(new AlgaeHold(grabSubsystem));
    //op.x().whileTrue(new AlgaeHold(grabSubsystem));
    //op.y().whileTrue(new AlgaeHold(grabSubsystem));
    
    op.leftStick().onTrue(new Barge(grabSubsystem, pivotSubsystem));
    op.rightStick().onTrue(new ThrowForFun(grabSubsystem, pivotSubsystem));


    op.leftTrigger(0.05).whileTrue(new GrabIn(grabSubsystem));
    op.leftTrigger(0.99).whileTrue(new AlgaeHold(grabSubsystem));
    op.rightTrigger(0.5).whileTrue(new GrabOut(grabSubsystem));

    

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
     return autoChooser.get();
  }

  public void periodic() {

  }
}
