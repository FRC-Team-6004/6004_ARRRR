package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfigurator;
//custom
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoverConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.TalonFXConstants;

import com.ctre.phoenix6.hardware.CANrange;


import edu.wpi.first.wpilibj.DriverStation;

public class Cover extends SubsystemBase {
    TalonFX motor;
    double CANrangeDistance;


    /**
     * This subsytem that controls the arm.
     */
    public Cover () {
        motor = new TalonFX(CoverConstants.MotorID);
        
        var motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 55;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    } 

    @Override
    public void periodic() {
        /* 
        if (DriverStation.getMatchTime() < 20) {
            if (DriverStation.getMatchTime() > 10) {
                motor.set(CoverConstants.SPEED);
            } else {
                motor.set(CoverConstants.HOLD);
            }
        }
        */
    }
    /** 
     * This is a method that makes the arm move at your desired speed
     *  Positive values make it spin forward and negative values spin it in reverse
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void moveGrab(double speed){
        motor.set(speed);

    }
}