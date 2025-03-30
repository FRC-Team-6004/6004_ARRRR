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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.TalonFXConstants;

import com.ctre.phoenix6.hardware.CANrange;

public class GrabSub extends SubsystemBase {
    TalonFX intakeGrab;
    double CANrangeDistance;


    /**
     * This subsytem that controls the arm.
     */
    public GrabSub () {
        intakeGrab = new TalonFX(IntakeConstants.INTAKE_Grab_ID);
        
        var intakeGrabConfiguration = new TalonFXConfiguration();

        intakeGrabConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeGrabConfiguration.CurrentLimits.SupplyCurrentLimit = 55;
        intakeGrabConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        //set up CANrange with an Id of 1 and no canbus specifications
        CANrange CANrange = new CANrange(1);
        CANrangeConfiguration CANrangeConfigs = new CANrangeConfiguration();
        CANrangeDistance = CANrange.getDistance().getValueAsDouble();
    } 

    @Override
    public void periodic() {
     /*   int cantime = 0;
       CANrange CANrange = new CANrange(1);
       CANrangeConfiguration CANrangeConfigs = new CANrangeConfiguration();
       CANrangeDistance = CANrange.getDistance().getValueAsDouble();
       boolean CANdetects = CANrange.getIsDetected().getValue();
       if (CANdetects && CANrangeDistance <= .04 && CANrangeDistance >= .023) {
        intakeGrab.set(-0.05);
       } else {
        intakeGrab.set(0);
        cantime = 0;
        System.out.println(CANrangeDistance);
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
        intakeGrab.set(speed);

    }
}