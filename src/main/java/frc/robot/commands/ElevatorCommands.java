package frc.robot.commands;

import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.Commands;

public class ElevatorCommands {
    
    public static Command setElevatorToPosition(Elevator elevator, int position) {
        double targetHeight;

        switch (position) {
            case 1:
                targetHeight = ElevatorConstants.LIFT_HEIGHT_1;
                break;
            case 2:
                targetHeight = ElevatorConstants.LIFT_HEIGHT_2;
                break;
            case 3:
                targetHeight = ElevatorConstants.LIFT_HEIGHT_3;
                break;
            case 4:
                targetHeight = ElevatorConstants.LIFT_HEIGHT_4;
                break;
            case 5:
                targetHeight = ElevatorConstants.LIFT_HEIGHT_5;
                break;
            case 6:
                targetHeight = ElevatorConstants.LIFT_HEIGHT_6;
                break;
            default:
                throw new IllegalArgumentException("Invalid elevator position: " + position);
        }

        return Commands.runOnce(() -> elevator.setGoal(targetHeight), elevator)
        .andThen(Commands.waitUntil(() -> elevator.atGoal(targetHeight)))
        .withTimeout(1); // 1 sec max
    
    }
}
