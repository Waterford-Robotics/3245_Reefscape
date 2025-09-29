// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CANrangeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// Runs Intake
public class RunIntakeCommand extends Command {

  // Uses Intake
  IntakeSubsystem m_intakeSubsystem;
  CANrangeSubsystem m_CANrangeSubsystem;

  // Constructor
  public RunIntakeCommand(IntakeSubsystem intakeSubsystem, CANrangeSubsystem CANrangeSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
    m_CANrangeSubsystem = CANrangeSubsystem;
    addRequirements(CANrangeSubsystem);
  }

  // Reset timer when the command starts executing
  public void initialize() {
  }
  
  // Actual command
  public void execute() {
    if(!m_CANrangeSubsystem.getIsDetected()) {
      m_intakeSubsystem.intake();
    }

  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopShooter();
  }

  // Checks if the command is done
  public boolean isFinished() {
    return m_CANrangeSubsystem.getIsDetected();
  }
}
