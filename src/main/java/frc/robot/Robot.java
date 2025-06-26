// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LocalADStarAK;

public class Robot extends LoggedRobot {
  Orchestra m_orchestra = new Orchestra();
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() throws IOException, ParseException {
    Logger.recordMetadata("ProjectName", "2025-Reefscape");
    switch (constants.currentMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start(); 
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.periodic();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void robotInit(){
    Pathfinding.setPathfinder(new LocalADStarAK());

    m_orchestra.addInstrument(new com.ctre.phoenix6.hardware.TalonFX(9));
    m_orchestra.addInstrument(new com.ctre.phoenix6.hardware.TalonFX(10));
    m_orchestra.addInstrument(new com.ctre.phoenix6.hardware.TalonFX(11));
    m_orchestra.addInstrument(new com.ctre.phoenix6.hardware.TalonFX(12));
    m_orchestra.addInstrument(new com.ctre.phoenix6.hardware.TalonFX(13));
    m_orchestra.addInstrument(new com.ctre.phoenix6.hardware.TalonFX(14));  
    m_orchestra.addInstrument(new com.ctre.phoenix6.hardware.TalonFX(15));  
    m_orchestra.addInstrument(new com.ctre.phoenix6.hardware.TalonFX(16));
    m_orchestra.addInstrument(new com.ctre.phoenix6.hardware.TalonFX(18));
    m_orchestra.addInstrument(new com.ctre.phoenix6.hardware.TalonFX(30));
    m_orchestra.addInstrument(new com.ctre.phoenix6.hardware.TalonFX(42));


    m_orchestra.loadMusic("sb.chrp");
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    m_orchestra.play();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {    
  if (!m_orchestra.isPlaying()) {
    m_orchestra.play();
  }
}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void testExit() {}
}
