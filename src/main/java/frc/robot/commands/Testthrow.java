// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.GrabSub;
import frc.robot.subsystems.PivotSub;
import edu.wpi.first.wpilibj2.command.Command;


/** An liftUpCommand that uses an lift subsystem. */
public class Testthrow extends Command {
  private final PivotSub m_intake;
  private final GrabSub m_grab;

  Timer m_timer;
  double m_duration;
  /**
   * Powers the lift up, when finished passively holds the lift up.
   * 
   * We recommend that you use this to only move the lift into the hardstop
   * and let the passive portion hold the lift up.
   *
   * @param lift The subsystem used by this command.
   */
  public Testthrow(GrabSub input, PivotSub input2) {
    m_grab = input;
    m_intake = input2;
    addRequirements(input, input2);
    m_timer = new Timer();
    m_timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_duration = .5;
    // Reset the clock
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() > .1) {
      m_intake.setGoal(IntakeConstants.PIVOT_POS_1);
      if (m_timer.get() > .25) {
          m_grab.moveGrab(-IntakeConstants.INTAKE_SPEED);
      } else {
          m_grab.moveGrab(IntakeConstants.INTAKE_SPEED);
      }
  } else {
    m_intake.setGoal(IntakeConstants.PIVOT_POS_3);
    m_grab.moveGrab(IntakeConstants.INTAKE_SPEED);
  }

  }

  // Called once the command ends or is interrupted.
  // Here we run arm down at low speed to ensure it stays down
  // When the next command is caled it will override this command
  @Override
  public void end(boolean interrupted) {
    m_grab.moveGrab(IntakeConstants.INTAKE_SPEED_HOLD);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration);
  }
}