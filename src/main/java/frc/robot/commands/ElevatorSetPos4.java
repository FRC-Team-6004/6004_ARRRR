// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An liftUpCommand that uses an lift subsystem. */
public class ElevatorSetPos4 extends Command {
  private final Elevator m_elevator;

  /**
   * Powers the lift up, when finished passively holds the lift up.
   * 
   * We recommend that you use this to only move the lift into the hardstop
   * and let the passive portion hold the lift up.
   *
   * @param lift The subsystem used by this command.
   */
  public ElevatorSetPos4(Elevator lift) {
    m_elevator = lift;
    addRequirements(lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setGoal(ElevatorConstants.LIFT_HEIGHT_4);
    //System.out.print("run pid elev");
  }

  // Called once the command ends or is interrupted.
  // Here we run a command that will hold the lift up after to ensure the lift does
  // not drop due to gravity.
  @Override
  public void end(boolean interrupted) {
    m_elevator.moveElevator(ElevatorConstants.LIFT_HOLD_UP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}