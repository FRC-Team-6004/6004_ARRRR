package frc.robot.commands;

import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotSub;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.Commands;
/* 
public class PivotCommands {
    public static Command setPivotToPosition(PivotSub pivot, int position) {
        double targetHeight;

        switch (position) {
            case 0:
                targetHeight = IntakeConstants.PIVOT_POS_0;
                break;
            case 1:
                targetHeight = IntakeConstants.PIVOT_POS_1;
                break;
            case 2:
                targetHeight = IntakeConstants.PIVOT_POS_2;
                break;
            case 3:
                targetHeight = IntakeConstants.PIVOT_POS_3;
                break;
            default:
                throw new IllegalArgumentException("Invalid elevator position: " + position);
        }

        //return Commands.runOnce(() -> pivot.setGoal(targetHeight), pivot)
        //.andThen(Commands.waitUntil(() -> pivot.atGoal(targetHeight)))
        //.withTimeout(1); // 1 sec max
    
    }
}
*/