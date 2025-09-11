package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.CoverConstants;


public class Cover extends SubsystemBase {
    TalonFX motor;

    /**
     * This subsytem that controls the arm.
     */
    public Cover() {
        motor = new TalonFX(CoverConstants.MotorID);
        
        var motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 55;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Apply the motor configuration
        motor.getConfigurator().apply(motorConfig);
    } 

    @Override
    public void periodic() {
        double matchTime = edu.wpi.first.wpilibj.DriverStation.getMatchTime();
        if (matchTime < 15) {
            if (matchTime > 14) {
                motor.set(CoverConstants.SPEED);
            } else {
                motor.set(0);
            }
        }
    }
    /** 
     * This is a method that makes the arm move at your desired speed
     *  Positive values make it spin forward and negative values spin it in reverse
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void moveCover(double speed){
        motor.set(speed);

    }
}