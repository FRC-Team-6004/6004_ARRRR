package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.AprilTag.VisionConstants;

public class constants {
   public static final boolean tuningMode = false;
    public static final Mode currentMode = Robot.isReal() ? Mode.REAL : Mode.SIM;
    public static final boolean visonSimEnabled = true;

    public static final class driveConstants {
        public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    }
    public static enum Mode {
        SIM,
        REAL,
        REPLAY
    }

    public static class OIConstants {
        public static final CommandXboxController driverController = new CommandXboxController(0);
        public static final CommandXboxController operatorController = new CommandXboxController(1);
        public static final double KAxisDeadband = 0.1;  
        public static final double OperatorLAxisDeadband = 0.3;
        public static boolean isScoringLeft = true;
        public static boolean aligned = false; 
        public static boolean inScoringDistance = false;
        public static boolean isReefTagOnly = true;
        public static int autoScoreMode = 4;
    }

    public static enum Gamepiece {
        ALGAE,
        CORAL,
        NONE,
        SIM,
    }

    public static final class FieldConstants {
        public static final double fieldLength = VisionConstants.aprilTagFieldLayout.getFieldLength();
        public static final double fieldWidth = VisionConstants.aprilTagFieldLayout.getFieldWidth();
    }

    public static final class ElevatorConstants {
        public static final int LIFT_MAIN = 62;             //PWM
        public static final int LIFT_FOLLOW = 61;             //PWM
        public static final int LIFT_CUR_LMT = 50;   //const

        public static final double LIFT_MOTOR_VOLTAGE_COMP = 10;
        public static final double LIFT_SPEED_DOWN = 0.5;
        public static final double LIFT_SPEED_UP = -0.5;
        public static final double LIFT_HOLD_DOWN = .1;
        public static final double LIFT_HOLD_UP = .1;

        public static final double LIFT_HEIGHT_1 = 0.0;   //set point 1 is Stowed
        public static final double LIFT_HEIGHT_2 = .122;   //set point 2 is L2
        public static final double LIFT_HEIGHT_3 = .27;   //set point 3 is L3
        public static final double LIFT_HEIGHT_4 = 0.6;   //set point 4 is L4
        public static final double LIFT_HEIGHT_5 = .2;   //L3/4 Algae
        public static final double LIFT_HEIGHT_6 = .45;   //L2/3 Algae

        public static final double   kElevatorGearing         = 12.0;
        public static final double   kElevatorSproketTeeth    = 22;
        public static final double   kElevatorPitch           = 0.25 * 0.0254; // Convert inches to meters
        public static final double   kElevatorDrumRadius      = (kElevatorSproketTeeth * kElevatorPitch) / (2 * Math.PI);// radius = Circumference / (2 pi)
        // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
        public static final double   kMinElevatorHeightMeters = 0;//min height / 10
        public static final double   kMaxElevatorHeightMeters = 30 * 0.0254;

      }

      public final class TalonFXConstants {
        public static final int COUNTS_PER_REV = 2048;
        public static final double COUNTS_PER_DEG = COUNTS_PER_REV / 360.0;
    
        public static final boolean TALON_FUTURE_PROOF = true; 
        
    
        
    }
    
        public final class IntakeConstants {
      public static final int INTAKE_Pivot_ID=30;
      public static final int INTAKE_Grab_ID= 16;
      
      
      public static final double Pivot_SPEED= .15;
      public static final double Pivot_SPEED_HOLD= -.017;

      public static final double INTAKE_SPEED= .30;
      public static final double INTAKE_SPEED_HOLD= .0;
  
      public static final InvertedValue INTAKE_INVERSION = InvertedValue.Clockwise_Positive;
      public static final NeutralModeValue INTAKE_NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final double INTAKE_POSITION_STATUS_FRAME = 0.05;
      public static final double INTAKE_VELOCITY_STATUS_FRAME = 0.01; 
      
      public static final double PIVOT_POS_0 = 0;
      public static final double PIVOT_POS_1 = 1.5;
      public static final double PIVOT_POS_2 = 4;  
      public static final double PIVOT_POS_3 = 15;  
      
  
  }

  public final class CoverConstants {
    public static final int MotorID=30;

    public static final double SPEED= .15;
    public static final double HOLD= .017;
    public static final InvertedValue INTAKE_INVERSION = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue INTAKE_NEUTRAL_MODE = NeutralModeValue.Brake;

  }

  public static final class ClimbConstants {
    public static final int LIFT_MAIN = 60;             //PWM
    public static final int LIFT_FOLLOW = 10;             //PWM
    public static final int LIFT_CUR_LMT = 50;   //const

    public static final double LIFT_MOTOR_VOLTAGE_COMP = 10;
    public static final double LIFT_SPEED_DOWN = 0.2;
    public static final double LIFT_SPEED_UP = -0.2;
    public static final double LIFT_HOLD_DOWN = 0;
    public static final double LIFT_HOLD_UP = 0;

    public static final double LIFT_HEIGHT_1 = 0.0;   //set point 1 is Stowed
    public static final double LIFT_HEIGHT_2 = 6.0;   //set point 2 is L2

    public static final double   kElevatorGearing         = 12.0;
    public static final double   kElevatorSproketTeeth    = 22;
    public static final double   kElevatorPitch           = 0.25 * 0.0254; // Convert inches to meters
    public static final double   kElevatorDrumRadius      = (kElevatorSproketTeeth * kElevatorPitch) / (2 * Math.PI);// radius = Circumference / (2 pi)
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double   kMinElevatorHeightMeters = 0;//min height / 10
    public static final double   kMaxElevatorHeightMeters = 30 * 0.0254;
  }

}
