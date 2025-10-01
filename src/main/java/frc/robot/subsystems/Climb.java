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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // === Load calculation constants (tune for your setup) ===
    private static final double KT = 2.6 / 105.0;   // N·m/A, NEO empirical torque constant
    private static final double FREE_CURRENT = 1.8; // A, NEO no-load current
    private static final double GEAR_RATIO = ClimbConstants.kElevatorGearing; // use your gearbox ratio
    private static final double EFFICIENCY = 0.90;  // assume 90% drivetrain efficiency
    private static final double DRUM_RADIUS = ClimbConstants.kElevatorDrumRadius; // meters
    private static final double G = 9.81;           // gravity (m/s^2)

    /**
     * This subsystem controls the climb winch.
     */
    public Climb() {
        climbMotor = new SparkMax(ClimbConstants.LIFT_MAIN, MotorType.kBrushless);
        climbMotorFollow = new SparkMax(ClimbConstants.LIFT_FOLLOW, MotorType.kBrushless);

        climbMotor.setCANTimeout(250);
        climbMotorFollow.setCANTimeout(250);

        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.voltageCompensation(10);
        elevatorConfig.smartCurrentLimit(ClimbConstants.LIFT_CUR_LMT);
        elevatorConfig.idleMode(IdleMode.kBrake);

        climbMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbMotorFollow.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = climbMotor.getEncoder();
        pid = new PIDController(3, 0, 0);
    }

    @Override
    public void periodic() {
        // Report live load estimate
        //System.out.println("Climb/LoadKg " + getEstimatedLoadKg());
    }

    public void moveClimb(double speed){
        climbMotor.set(speed);
        climbMotorFollow.set(-speed);
    }

    public Trigger atHeight(double height, double tolerance) {
        return new Trigger(() -> MathUtil.isNear(height,
                                                getHeightMeters(),
                                                tolerance));
    }

    public double getPos() {
        return encoder.getPosition() / COUNTS_PER_INCH;
    }

    public double getHeightMeters(){
        return (encoder.getPosition() / ClimbConstants.kElevatorGearing) *
            (2 * Math.PI * ClimbConstants.kElevatorDrumRadius);
    }

    public void setPosition(double targetPos) {
        double pidOutput = pid.calculate(getPos(), targetPos);
        double motorOutput = pidOutput + GRAVITY_COMPENSATION;
        motorOutput = Math.min(Math.max(motorOutput, -1.0), 1.0);
        
        climbMotor.set(motorOutput);  
        climbMotorFollow.set(-motorOutput); 
    }

    public void setGoal(double t) {
        targetpos = t;
    }

    /**
     * Estimate the suspended load (in kilograms) based on motor currents.
     */
    public double getEstimatedLoadKg() {
        double i1 = Math.max(0.0, climbMotor.getOutputCurrent() - FREE_CURRENT);
        double i2 = Math.max(0.0, climbMotorFollow.getOutputCurrent() - FREE_CURRENT);

        double tauMotor1 = KT * i1;
        double tauMotor2 = KT * i2;

        double tauOutTotal = (tauMotor1 + tauMotor2) * GEAR_RATIO * EFFICIENCY;

        double force = tauOutTotal / DRUM_RADIUS; // Newtons
        return force / G; // kilograms
    }
}
