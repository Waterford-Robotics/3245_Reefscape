// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

// Raises Wrist
public class SetWristCommand extends Command {

  // Uses Wrist and Subsystems
  WristSubsystem m_WristSubsystem;
  String m_level;

  // Constructor
  public SetWristCommand(WristSubsystem wristSubsystem, String level) {
        
    // Definitions and setting parameters are equal to members!
    m_WristSubsystem = wristSubsystem;
    addRequirements(wristSubsystem);

    // Level
    m_level = level;
  }

  // Reset timer when the command starts executing
  public void initialize() {
  }
  
  // Actual command
  public void execute() {

    // Raise to Intake Position
    if (m_level == "intake") {
      m_WristSubsystem.setPosition(WristConstants.k_wristIntakeAngle);
    }

    // Lower to Scoring Position
    if(m_level == "score") {
      m_WristSubsystem.setPosition(WristConstants.k_wristScoreAngle);
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return true;
  }
}
