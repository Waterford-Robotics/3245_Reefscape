// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Updated;

import frc.robot.subsystems.CANRangeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// Runs Intake
public class RunIntakeForSecsAutoCommand extends Command {

  // Uses Intake
  IntakeSubsystem m_intakeSubsystem;
  CANRangeSubsystem m_canRangeSubsystem;
  double m_seconds;
  Timer m_timer = new Timer();

  // Constructor
  public RunIntakeForSecsAutoCommand(IntakeSubsystem intakeSubsystem, CANRangeSubsystem canRangeSubsystem, double seconds) {
        
    // Definitions and setting parameters are equal to members!
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
    m_canRangeSubsystem = canRangeSubsystem;
    addRequirements(canRangeSubsystem);
    // Time
    m_seconds = seconds;
  }

  // Reset timer when the command starts executing
  public void initialize() {
    m_timer.start();
    m_timer.reset();
  }
  
  // Actual command
  public void execute() {

    if(m_timer.get() < m_seconds) {
      m_intakeSubsystem.intake();
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopShooter();
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return m_timer.get() > m_seconds || m_canRangeSubsystem.getIsDetected();
  }
}
