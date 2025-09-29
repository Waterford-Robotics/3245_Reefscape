// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CANrangeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// Raises Elevator
public class RunShootCommand extends Command {

  // Uses Elevator and Subsystems
  IntakeSubsystem m_intakeSubsystem;
  CANrangeSubsystem m_CANrangesubsystem;
  boolean m_positioned;

  // Constructor
  public RunShootCommand(IntakeSubsystem intakeSubsystem, CANrangeSubsystem CANrangesubsystem, boolean positioned) {
        
    // Definitions and setting parameters are equal to members!
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
    m_CANrangesubsystem = CANrangesubsystem;

    // Position Status
    m_positioned = positioned;
  }

  // Reset timer when the command starts executing
  public void initialize() {
  }
  
  // Actual command
  public void execute() {
    if(m_CANrangesubsystem.getIsDetected()) {
      m_intakeSubsystem.shoot();
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopShooter();
  }

  // Checks if the command is done
  public boolean isFinished() {
    // Am I done?  Am I done? Am I finally done?
    return !m_CANrangesubsystem.getIsDetected();
  }
}