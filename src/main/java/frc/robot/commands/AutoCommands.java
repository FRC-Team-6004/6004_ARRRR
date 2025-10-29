package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.*;

public class AutoCommands {

    public static Command l4Command(PivotSub pivotSubsystem, Elevator elevatorSubsystem) {
        return new PivotPos1(pivotSubsystem)
            .andThen(ElevatorCommands.setElevatorToPosition(elevatorSubsystem, 4))
            .andThen(new PivotPos2(pivotSubsystem));
    }

    public static Command l3Command(PivotSub pivotSubsystem, Elevator elevatorSubsystem) {
        return new PivotPos1(pivotSubsystem)
            .andThen(ElevatorCommands.setElevatorToPosition(elevatorSubsystem, 3));
    }

    public static Command l2Command(PivotSub pivotSubsystem, Elevator elevatorSubsystem) {
        return new PivotPos1(pivotSubsystem)
            .andThen(ElevatorCommands.setElevatorToPosition(elevatorSubsystem, 2));
    }

    public static Command l1Command(PivotSub pivotSubsystem, Elevator elevatorSubsystem) {
        return new PivotPos1(pivotSubsystem)
            .andThen(ElevatorCommands.setElevatorToPosition(elevatorSubsystem, 1))
            .andThen(new PivotPos0(pivotSubsystem));
    }

    public static Command grabInAuto(GrabSub grab) {
        return new Command() {
            Timer t = new Timer();
            { t.start(); }
            @Override public void initialize() { t.reset(); }
            @Override public void execute() { grab.moveGrab(IntakeConstants.INTAKE_SPEED); }
            @Override public void end(boolean i) { grab.moveGrab(IntakeConstants.INTAKE_SPEED_HOLD); }
            @Override public boolean isFinished() { return t.hasElapsed(0.4); }
        };
    }
    public static Command grabOutAuto(GrabSub grab) {
        return new Command() {
            Timer t = new Timer();
            { t.start(); }
            @Override public void initialize() { t.reset(); }
            @Override public void execute() { grab.moveGrab(-IntakeConstants.INTAKE_SPEED); }
            @Override public void end(boolean i) { grab.moveGrab(IntakeConstants.INTAKE_SPEED_HOLD); }
            @Override public boolean isFinished() { return t.hasElapsed(0.4); }
        };
    }
}