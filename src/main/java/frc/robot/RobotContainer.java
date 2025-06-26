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

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.OIConstants;
import frc.robot.commands.AlgaeHold;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.Barge;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbPos1;
import frc.robot.commands.ClimbPos2;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.ElevatorSetPos1;
import frc.robot.commands.ElevatorSetPos2;
import frc.robot.commands.ElevatorSetPos3;
import frc.robot.commands.ElevatorSetPos4;
import frc.robot.commands.ElevatorSetPos5;
import frc.robot.commands.ElevatorSetPos6;
import frc.robot.commands.GrabIn;
import frc.robot.commands.GrabOut;
import frc.robot.commands.PivotPos0;
import frc.robot.commands.PivotPos1;
import frc.robot.commands.PivotPos2;
import frc.robot.commands.PivotPos3;
import frc.robot.commands.PivotTimedRev;
import frc.robot.commands.RunL1;
import frc.robot.commands.Testthrow;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
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
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.LED;

public class RobotContainer {
  private RobotVisualizer visualizer;
  private Vision vision;
  
  
  private final Elevator elevatorSubsystem = new Elevator();
  private CommandXboxController op = new CommandXboxController(1);
  private CommandXboxController joystick = new CommandXboxController(0);

    public final PivotSub pivotSubsystem = new PivotSub();
    public final GrabSub grabSubsystem = new GrabSub();
    public final Climb climbSubsystem = new Climb();


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

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private boolean isRed = false;

  LoggedDashboardChooser<Command> autoChooser;



  public RobotContainer() throws IOException, ParseException {
            // Initialize the LED on PWM port 9
        m_led = new AddressableLED(9);

        // Reuse buffer
        // Default to a length of 150, start empty output
        m_ledBuffer = new AddressableLEDBuffer(300 - 38);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
        //setColor(255, 0, 0); // Set to red
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 255, 255, 255);
      }
      m_led.setData(m_ledBuffer);
    GenericRequirement.initialize();
    switch (constants.currentMode) {
      case REAL:
        drivetrain = Swerve.initialize(new Swerve(TunerConstants.DrivetrainConstants, 50, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight));
        //vision = Vision.initialize(
        //  new VisionIOReal(0), 
        //  new VisionIOReal(1)
        //);  
        break;

      case SIM:
        drivetrain = Swerve.initialize(TunerConstants.createDrivetrain());
        visualizer = new RobotVisualizer();
        if(constants.visonSimEnabled) {
        //  vision = Vision.initialize(new VisionIOSim());
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
          .applyRequest(() -> drive.withVelocityX(-constants.OIConstants.driverController.getLeftY() * -.5 * SwerveConstants.MaxSpeed * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier : 1))
              .withVelocityY(-constants.OIConstants.driverController.getLeftX() * -0.5 * SwerveConstants.MaxSpeed * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier : 1))
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

   op.povDown().onTrue(new ElevatorSetPos1(elevatorSubsystem));
   op.povLeft().onTrue(new ElevatorSetPos2(elevatorSubsystem));
   op.povRight().onTrue(new ElevatorSetPos3(elevatorSubsystem));
   op.povUp().onTrue(new ElevatorSetPos4(elevatorSubsystem));
   op.povDown().onTrue(new PivotPos1(pivotSubsystem));
   op.povLeft().onTrue(new PivotPos1(pivotSubsystem));
   op.povRight().onTrue(new PivotPos1(pivotSubsystem));
   op.povUp().onTrue(new PivotPos2(pivotSubsystem));
   op.leftStick().onTrue(new PivotPos0(pivotSubsystem));

    op.a().onTrue(new ElevatorSetPos2(elevatorSubsystem));
    op.y().onTrue(new ElevatorSetPos4(elevatorSubsystem));
    op.x().onTrue(new ElevatorSetPos5(elevatorSubsystem));
    op.b().onTrue(new ElevatorSetPos6(elevatorSubsystem));
    op.a().onTrue(new PivotPos3(pivotSubsystem));
    op.b().onTrue(new PivotPos3(pivotSubsystem));
    op.x().onTrue(new PivotPos3(pivotSubsystem));
    op.y().onTrue(new PivotPos3(pivotSubsystem));       
    
    op.rightBumper().onTrue(new Barge(grabSubsystem, pivotSubsystem));
    op.leftBumper().onTrue(new Testthrow(grabSubsystem, pivotSubsystem));


    op.leftTrigger(0.05).whileTrue(new GrabIn(grabSubsystem));
    op.leftTrigger(0.99).whileTrue(new AlgaeHold(grabSubsystem));
    op.rightTrigger(0.05).whileTrue(new GrabOut(grabSubsystem));

    //joystick.povDown().whileTrue(new ClimbPos1(climbSubsystem));
    //joystick.povUp().whileTrue(new ClimbPos2(climbSubsystem));
    joystick.povDown().whileTrue(new ClimbDown(climbSubsystem));
    joystick.povUp().whileTrue(new ClimbUp(climbSubsystem));


    

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
     return autoChooser.get();
  }

  public void setColor(int r, int g, int b) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
}
 int c = 0;
  public void periodic() {
    if (edu.wpi.first.wpilibj.DriverStation.getMatchTime() < 15 && 
        edu.wpi.first.wpilibj.DriverStation.getMatchTime() > 0) {
      rainbow();
    } else {
      if (grabSubsystem.CoralDetect) {
        if ((c < 5) || (c < 15 && c > 10)) {
          setColor(0, 0, 0);
        } else {
          setColor(0, 255, 0);
        }
        c++;
      } else {
        setColor(255, 0, 0);
        c = 0;
      }
    }
  }

  public int ranI() {
    return (int) (Math.random() * 255);
  }

  public void rainbow() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      int hue = (c * 3 + (i * 360 / m_ledBuffer.getLength())) % 360; // Faster and smoother rainbow effect
      double wave = Math.sin((c + i) * 0.2) * 0.5 + 0.5; // Wave-like pulsating brightness
      double sparkle = Math.random() < 0.02 ? 1.0 : wave; // Add occasional sparkles
      int r = (int) (Math.sin(0.024 * hue + 0) * 100 * sparkle + 100); // Reduced brightness
      int g = (int) (Math.sin(0.024 * hue + 2) * 100 * sparkle + 100); // Reduced brightness
      int b = (int) (Math.sin(0.024 * hue + 4) * 100 * sparkle + 100); // Reduced brightness
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
    c += 1;
  }
}
