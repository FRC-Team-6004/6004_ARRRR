package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GenericRequirement;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.PivotSub;

public class RunL1 {
    private final static PivotSub m_intake = new PivotSub();
    private final static Elevator m_elevator = new Elevator();

    public static Command createRunL1Command() {
        Command command = Commands.sequence(
            Commands.runOnce(() -> m_intake.setGoal(IntakeConstants.PIVOT_POS_1)),
            Commands.runOnce(() -> m_elevator.setGoal(ElevatorConstants.LIFT_HEIGHT_1)),
            Commands.waitUntil(() -> m_elevator.atGoal(ElevatorConstants.LIFT_HEIGHT_1)))
                .andThen(Commands.runOnce(() -> m_intake.setGoal(IntakeConstants.PIVOT_POS_0)));

        command.addRequirements(m_elevator, m_intake);

        return command;
    }

}
