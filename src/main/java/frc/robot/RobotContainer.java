// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.OIConstants;
import frc.robot.commands.AlgaeHold;
import frc.robot.commands.AutoAlignAndDrive;
import frc.robot.commands.Barge;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.GrabIn;
import frc.robot.commands.GrabOut;
import frc.robot.commands.PivotPos0;
import frc.robot.commands.PivotPos1;
import frc.robot.commands.PivotPos2;
import frc.robot.commands.PivotPos3;
import frc.robot.commands.Testthrow;
import frc.robot.commands.strafe;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GenericRequirement;
import frc.robot.subsystems.GrabSub;
//import frc.robot.subsystems.Pathing;
import frc.robot.subsystems.PivotSub;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.util.NamedCommandManager;
import frc.robot.subsystems.vision.OfficialReefscapeFieldLayout;
import frc.robot.subsystems.vision.Vision;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.Cover;
import org.photonvision.EstimatedRobotPose;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;

public class RobotContainer {
  private RobotVisualizer visualizer;

  //Controllers
  private CommandXboxController op = new CommandXboxController(1);
  private CommandXboxController joystick = new CommandXboxController(0);
  private CommandXboxController tesController = new CommandXboxController(2);

  //Subsystems
  private final Elevator elevatorSubsystem = new Elevator();
  public final PivotSub pivotSubsystem = new PivotSub();
  public final GrabSub grabSubsystem = new GrabSub();
  public final Climb climbSubsystem = new Climb();
  public final Cover coverSubsystem = new Cover();
  private final Vision vision = new Vision();
  
  // Swerve drivetrain
  public final Swerve swerve = new Swerve(
      TunerConstants.DrivetrainConstants,
      50, // odometry update frequency
      TunerConstants.FrontLeft,
      TunerConstants.FrontRight,
      TunerConstants.BackLeft,
      TunerConstants.BackRight
  );
  
  //public final Pathing pathing;

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
  
  LoggedDashboardChooser<Command> autoChooser;

  double speedDecay = .8;
  double maxN = speedDecay / (1 - speedDecay);

  public RobotContainer() throws IOException, ParseException {

  
            // Initialize the LED on PWM port 9
        m_led = new AddressableLED(9);

        // Reuse buffer
        // Default to a length of 150, start empty output
        m_ledBuffer = new AddressableLEDBuffer(300 - 120);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
        //setColor(255, 0, 0); // Set to red
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 255, 255, 255);
          
      }
      m_led.setData(m_ledBuffer);

        CommandScheduler.getInstance().registerSubsystem(coverSubsystem);

    GenericRequirement.initialize();
    drivetrain = Swerve.initialize(new Swerve(TunerConstants.DrivetrainConstants, 50, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight));

    NamedCommandManager.registerNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser("Driver Forward Straight"));
    



  configureBindings();
    
  }

  private void configureBindings() {
    // Drive command
    drivetrain.setDefaultCommand(
      drivetrain
          .applyRequest(() -> drive.withVelocityX(xs * (1 / maxN) * 1 * SwerveConstants.MaxSpeed * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier : 1))
              .withVelocityY(ys * (1 / maxN) * 1 * SwerveConstants.MaxSpeed * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier : 1))
              .withRotationalRate(-rs * .8 * SwerveConstants.MaxAngularRate * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier : 1))));

    // field center
    constants.OIConstants.driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Slow mode
    constants.OIConstants.driverController.rightTrigger(0.5).onTrue(Commands.runOnce(() -> drivetrain.setSlowMode(true)));
    constants.OIConstants.driverController.rightTrigger(0.5).onFalse(Commands.runOnce(() -> drivetrain.setSlowMode(false)));

   op.povDown().onTrue((new PivotPos1(pivotSubsystem)).andThen(ElevatorCommands.setElevatorToPosition(elevatorSubsystem, 1)
  .andThen(new PivotPos0(pivotSubsystem))));
   op.povLeft().onTrue((new PivotPos1(pivotSubsystem)).andThen(ElevatorCommands.setElevatorToPosition(elevatorSubsystem, 2)));
   op.povRight().onTrue((new PivotPos1(pivotSubsystem)).andThen(ElevatorCommands.setElevatorToPosition(elevatorSubsystem, 3)));
   op.povUp().onTrue((new PivotPos1(pivotSubsystem)).andThen(ElevatorCommands.setElevatorToPosition(elevatorSubsystem, 4)
   .andThen(new PivotPos2(pivotSubsystem))));

    op.a().onTrue(new PivotPos3(pivotSubsystem).andThen(ElevatorCommands.setElevatorToPosition(elevatorSubsystem, 2)));
    op.y().onTrue(new PivotPos3(pivotSubsystem).andThen(ElevatorCommands.setElevatorToPosition(elevatorSubsystem, 4)));
    op.x().onTrue(new PivotPos3(pivotSubsystem).andThen(ElevatorCommands.setElevatorToPosition(elevatorSubsystem, 5)));
    op.b().onTrue(new PivotPos3(pivotSubsystem).andThen(ElevatorCommands.setElevatorToPosition(elevatorSubsystem, 6)));
     
    
    op.rightBumper().onTrue(new Barge(grabSubsystem, pivotSubsystem));
    op.leftBumper().onTrue(new Testthrow(grabSubsystem, pivotSubsystem));


    op.leftTrigger(0.05).whileTrue(new GrabIn(grabSubsystem));
    op.leftTrigger(0.99).whileTrue(new AlgaeHold(grabSubsystem));
    op.rightTrigger(0.05).whileTrue(new GrabOut(grabSubsystem));

    //joystick.povDown().whileTrue(new ClimbPos1(climbSubsystem));
    //joystick.povUp().whileTrue(new ClimbPos2(climbSubsystem));
    joystick.povDown().whileTrue(new ClimbDown(climbSubsystem));
    joystick.povUp().whileTrue(new ClimbUp(climbSubsystem));

    joystick.rightBumper().onTrue(Commands.runOnce(() -> startStrafe(-0.25, 0.25)));
    joystick.leftBumper().onTrue(Commands.runOnce(() -> startStrafe(0.25, 0.25)));

    joystick.a().onTrue(Commands.runOnce(() -> startAutoAlign()));
    joystick.a().onFalse(Commands.runOnce(() -> stopAutoAlign()));

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
 double c = 0;
 double xs = 0;
 double ys = 0;
 double rs = 0;
private boolean isStrafing = false;
private double strafeStartTime = 0.0;
private double strafeDuration = 0.0;
private double strafeSpeed = 0.0; // positive = left, negative = right
private boolean isAutoAligning = false;
private double desiredPitch = 10;

private final PIDController turnPID = new PIDController(0.02, 0, 0.001);
private final PIDController strafePID = new PIDController(0.0125, 0, 0);
private final PIDController forwardPID = new PIDController(0.05, 0, 0);

public void startStrafe(double speed, double durationSeconds) {
  if (!isStrafing) {
      isStrafing = true;
      strafeSpeed = speed;
      strafeDuration = durationSeconds;
      strafeStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
  }
}

public void startAutoAlign() {
  if (!isAutoAligning) {
      isAutoAligning = true;

      turnPID.setTolerance(1.0);     // degrees
      forwardPID.setTolerance(1.0);  // pitch tolerance
  }
}

public void stopAutoAlign() {
  isAutoAligning = false;
}

  public void periodic() {
    int mode = 1;
    //mode 1: trapezoid profile
    //mode default: reg
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    if (isStrafing) {
        // Override joystick inputs while strafing
        xs = -swerve.getSin() * maxN * strafeSpeed;          // no forward/back motion
        ys = swerve.getCos() * maxN * strafeSpeed;  // left/right
        rs = 0.0;          // no rotation
    
        // End strafe after specified time
        if (currentTime - strafeStartTime >= strafeDuration) {
            isStrafing = false;
            xs = 0.0;
            ys = 0.0;
            rs = 0.0;
        }
    } else {
        // Normal joystick control
        switch(mode) {
            case 1 : 
                xs += joystick.getLeftY();
                ys += joystick.getLeftX();
                xs *= speedDecay;
                ys *= speedDecay;
                rs = constants.OIConstants.driverController.getRightX();
                break;
            default : 
                xs = joystick.getLeftY() * maxN;
                ys = joystick.getLeftX() * maxN;
                rs = constants.OIConstants.driverController.getRightX();
                break;
        }
    }

    autoAlignPeriodic();

    if (edu.wpi.first.wpilibj.DriverStation.getMatchTime() < 15 && 
        edu.wpi.first.wpilibj.DriverStation.getMatchTime() > -1) {
      fox();
    } else {
      if (grabSubsystem.CoralDetect) {
        if ((c < 10) || (c < 30 && c > 20)) {
          setColor(0, 0, 0);
        } else {
          setColor(0, 255, 0);
        }
        c++;
      } else {
        setColor(255, 0, 0);
        joystick.setRumble(RumbleType.kBothRumble, 0);
        c = 0;
      }
    }
  }

public void autoAlignPeriodic() {
    if (!isAutoAligning) return;
  
    if (vision.hasTarget()) {
        double yaw = vision.getTargetYaw();      // horizontal offset
        double pitch = vision.getTargetPitch();  // vertical offset
        System.out.println("yaw = " + yaw);
        System.out.println("pitch = " + pitch);

        // PID outputs (clamped to -1..1)
        double turnOutput = MathUtil.clamp(turnPID.calculate(yaw, 0.0), -1.0, 1.0);
        double forwardOutput = MathUtil.clamp(forwardPID.calculate(pitch, desiredPitch), -1.0, 1.0);
        double strafeOut = MathUtil.clamp(strafePID.calculate(yaw, 0.0), -1.0, 1.0);

        System.out.println("turn out = " + turnOutput);
        System.out.println("forward out = " + forwardOutput);

        // Scale to robot max speeds
        xs = swerve.getCos() * maxN * -forwardOutput;          // no forward/back motion
        ys = swerve.getSin() * maxN * -forwardOutput;
        xs += swerve.getSin() * maxN * strafeOut;
        ys += -swerve.getCos() * maxN * strafeOut;
        xs /= 2;
        ys /= 2;
        rs = -turnOutput; // rotation
    } else {
        // Stop if target lost
        xs = 0.0;
        ys = 0.0;
        rs = 0.0;
        isAutoAligning = false;
    }
  
    // Stop when PID reaches setpoint
    if (vision.hasTarget() && turnPID.atSetpoint() && forwardPID.atSetpoint()) {
        xs = 0.0;
        ys = 0.0;
        rs = 0.0;
        isAutoAligning = false;
    }
  }
  public int ranI() {
    return (int) (Math.random() * 255);
  }

  public void rainbow() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        // Cycle hue smoothly across LEDs
        int hue = (int) (c + (i * 360 / m_ledBuffer.getLength())) % 360;

        // Convert HSV to RGB, dim by scaling brightness
        int rgb = java.awt.Color.HSBtoRGB(hue / 360f, 1.0f, 0.3f); // 0.3f = dimmed brightness

        int r = (rgb >> 16) & 0xFF;
        int g = (rgb >> 8) & 0xFF;
        int b = rgb & 0xFF;

        m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
    c += 5;
}

  public void fox() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if ((int) (((i + c) / 5) % 3) == 0) {
        m_ledBuffer.setRGB(i, 255, 40, 0); // Orange 
      } else if ((int) (((i + c) / 5) % 3) == 1) {
        m_ledBuffer.setRGB(i, 255, 255, 255); // White 
      } else {
        m_ledBuffer.setRGB(i, 2, 10, 10); // Gray 

      }
    }
    m_led.setData(m_ledBuffer);
    c += 0.5;
  }

  public void fox2() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if ((int) (((i + c) / 5) % 3) == 0) {
        m_ledBuffer.setRGB(i, 200, 0, 200); // Pink
      } else if ((int) (((i + c) / 5) % 3) == 1) {
        m_ledBuffer.setRGB(i, 255, 255, 255); // White
      } else {
        m_ledBuffer.setRGB(i, 2, 10, 10); // Gray

      }
    }
    m_led.setData(m_ledBuffer);
    c += 0.5;
  }

  public void fun() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (c % 20 > 10) {
        m_ledBuffer.setRGB(i, 255, 0, 0); // red
      } else {
        m_ledBuffer.setRGB(i, 0, 0, 255); // blue
      }
    }
    m_led.setData(m_ledBuffer);
    c += 1;
  }
  public void test() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i <= (m_ledBuffer.getLength() * tesController.getRightTriggerAxis())) {
        m_ledBuffer.setRGB(i, 255, 255, 255); // White for odd indices
      } else {
        m_ledBuffer.setRGB(i, 2, 10, 10); // White for odd indices
      }
    }
    System.out.println(m_ledBuffer.getLength() * tesController.getRightTriggerAxis());
  }



}

