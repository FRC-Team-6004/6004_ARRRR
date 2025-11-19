package frc.robot.commands;

import frc.robot.constants.CoverConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Cover;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DAP extends Command{
    private final Cover cover;
    Timer m_timer;
    double m_duration;

    public DAP(Cover c) {
        cover = c;
        addRequirements(c);
        m_timer = new Timer();
        m_timer.start();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {    
        m_duration = 1;
        m_timer.reset();
    }  

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        cover.moveCover(CoverConstants.SPEED);
    }

    // Called once the command ends or is interrupted.
    // Here we run a command that will hold the lift up after to ensure the lift does
    // not drop due to gravity.
    @Override
    public void end(boolean interrupted) {
        cover.moveCover(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration);
    }
}
