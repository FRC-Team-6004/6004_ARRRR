
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
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ElevatorConstants;

public class Climb extends SubsystemBase {

    private final SparkMax climbMotor;
    private final SparkMax climbMotorFollow;

    private final RelativeEncoder encoder;  
    private final PIDController pid;  

    private final double COUNTS_PER_INCH = 42.0; // Needs to be measured, not publically available
    private final double GRAVITY_COMPENSATION = 0; // edit if needed, should be good

    private static double targetpos = 0;

    /**
     * This subsytem that controls the arm.
     */
    public Climb() {

        // Set up the arm motor as a brushed motor
        climbMotor = new SparkMax(ClimbConstants.LIFT_MAIN, MotorType.kBrushless);
        climbMotorFollow = new SparkMax(ClimbConstants.LIFT_FOLLOW, MotorType.kBrushless);

        // Set can timeout. Because this project only sets parameters once on
        // construction, the timeout can be long without blocking robot operation. Code
        // which sets or gets parameters during operation may need a shorter timeout.
        climbMotor.setCANTimeout(250);
        climbMotorFollow.setCANTimeout(250);

        // Create and apply configuration for arm motor. Voltage compensation helps
        // the arm behave the same as the battery
        // voltage dips. The current limit helps prevent breaker trips or burning out
        // the motor in the event the arm stalls.
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.voltageCompensation(10);
        elevatorConfig.smartCurrentLimit(ClimbConstants.LIFT_CUR_LMT);
        elevatorConfig.idleMode(IdleMode.kBrake);
        climbMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbMotorFollow.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // PID values need tuning for your specific elevator
        encoder = climbMotor.getEncoder();
        pid = new PIDController(3, 0, 0);
    }

    @Override
    public void periodic() {
        //setPosition(targetpos);
    }
    /** 
     * This is a method that makes the arm move at your desired speed
     *  Positive values make it spin forward and negative values spin it in reverse
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void moveClimb(double speed){
        climbMotor.set(speed);
        climbMotorFollow.set(-speed);
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
    public double getPos() {
        return encoder.getPosition() / COUNTS_PER_INCH;
    }

    public double getHeightMeters(){
        // m = (e / g) * (2*pi*r)
        // m/(2*pi*r) = e / g
        // m/(2*pi*r)*g = e
        return (encoder.getPosition() / ClimbConstants.kElevatorGearing) *
            (2 * Math.PI * ClimbConstants.kElevatorDrumRadius);
    }

    public void setPosition(double targetPos) {
        double pidOutput = pid.calculate(getPos(), targetPos);
        
        // Add gravity compensation
        // The sign is positive because we need to work against gravity
        // You might need to flip the sign depending on your motor polarity
        double motorOutput = pidOutput + GRAVITY_COMPENSATION;
        
        // Clamp the output to valid range
        motorOutput = Math.min(Math.max(motorOutput, -1.0), 1.0);
        
        climbMotor.set(motorOutput);  
        climbMotorFollow.set(-motorOutput); 
    }

    public void setGoal(double t) {
        targetpos = t;
    }

}