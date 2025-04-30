package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private final SparkMax elevatorMotor;
    private final SparkMax elevatorMotorFollow;
    
    private final RelativeEncoder encoder;  
    private final PIDController pid;  

    private final double COUNTS_PER_INCH = 42.0; // Needs to be measured, not publically available
    private final double GRAVITY_COMPENSATION = (0.5 * 0.05); // edit if needed, should be good
    private double targetHeight = 0.0; //target height for elevator
    private double position = 0.0; // current position of the elevator

    /**
     * This subsytem that controls the arm.
     */
    public Elevator () {

        // Set up the arm motor as a brushed motor
        elevatorMotor = new SparkMax(ElevatorConstants.LIFT_MAIN, MotorType.kBrushless);
        elevatorMotorFollow = new SparkMax(ElevatorConstants.LIFT_FOLLOW, MotorType.kBrushless);

        // Set can timeout. Because this project only sets parameters once on
        // construction, the timeout can be long without blocking robot operation. Code
        // which sets or gets parameters during operation may need a shorter timeout.
        elevatorMotor.setCANTimeout(250);
        elevatorMotorFollow.setCANTimeout(250);

        // Create and apply configuration for arm motor. Voltage compensation helps
        // the arm behave the same as the battery
        // voltage dips. The current limit helps prevent breaker trips or burning out
        // the motor in the event the arm stalls.
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.voltageCompensation(10);
        elevatorConfig.smartCurrentLimit(ElevatorConstants.LIFT_CUR_LMT);
        elevatorConfig.idleMode(IdleMode.kBrake);
        elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotorFollow.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // PID values need tuning for your specific elevator
        encoder = elevatorMotor.getEncoder();
        pid = new PIDController(2.5, 0, 0);
    }

    @Override
    public void periodic() {
        setPosition(targetHeight);
    }
    /** 
     * This is a method that makes the arm move at your desired speed
     *  Positive values make it spin forward and negative values spin it in reverse
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void moveElevator(double speed){
        elevatorMotor.set(speed);
        elevatorMotorFollow.set(-speed);
    }
    /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in Meters
   * @param tolerance Tolerance in meters.
   * @return {@link Trigger}
   */
    public Trigger atHeight(double height, double tolerance)    {
        return new Trigger(() -> MathUtil.isNear(height,
                                                getHeightMeters(),
                                                tolerance));
    }

    // Returns elevator height in inches
    public double getHeight() {
        return encoder.getPosition() / COUNTS_PER_INCH;
    }

    public double getHeightMeters(){
        // m = (e / g) * (2*pi*r)
        // m/(2*pi*r) = e / g
        // m/(2*pi*r)*g = e
        return (encoder.getPosition() / ElevatorConstants.kElevatorGearing) *
            (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius);
    }

    public boolean atGoal(double goal) {
        return MathUtil.isNear(goal, getHeight(), 0.02);
    }
    
    public void setPosition(double targetHeight) {
        double pidOutput = pid.calculate(getHeight(), targetHeight);
        
        // Add gravity compensation
        // The sign is positive because we need to work against gravity
        // You might need to flip the sign depending on your motor polarity
        double motorOutput = pidOutput + GRAVITY_COMPENSATION;
        
        // Clamp the output to valid range
        motorOutput = Math.min(Math.max(motorOutput, -1.0), 1.0);
        
        elevatorMotor.set(motorOutput);  
        elevatorMotorFollow.set(-motorOutput);  
    }

    public void setGoal(double th) {
        targetHeight = th;
    }

}