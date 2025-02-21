// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

// Raises Wrist
public class ZeroWristCommand extends Command {

  // Uses Wrist and Subsystems
  WristSubsystem m_WristSubsystem;
  boolean m_finished;

  // Constructor
  public ZeroWristCommand(WristSubsystem WristSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_WristSubsystem = WristSubsystem;
    addRequirements(WristSubsystem);
  }

  // Reset timer when the command starts executing
  public void initialize() {
    m_finished = false;
  }
  
  // Actual command
  public void execute() {
    if(m_WristSubsystem.getCurrentPosition() < 2 && m_WristSubsystem.getCurrentVelocity() == 0) {
      m_finished = true;
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {

    // Neutral Motors and Reset Encoder Values
    m_WristSubsystem.setNeutral();
    m_WristSubsystem.resetSensorPosition(WristConstants.k_wristScoreAngle);
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return m_finished;
  }
}
