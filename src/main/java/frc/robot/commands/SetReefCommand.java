// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj2.command.Command;

// Raises Elevator
public class SetReefCommand extends Command {

  // Side of the Reef
  String m_reef;

  // Constructor
  public SetReefCommand(String reef) {
    
    // Level
    m_reef = reef;
  }

  // Reset timer when the command starts executing
  public void initialize() {
  }
  
  // Actual command
  public void execute() {

    // Set to Left Reef
    if (m_reef.equals("left")) {
      VisionConstants.k_isRightReef = false;
    }

    // Set to Right Reef
    if(m_reef.equals("right")) {
      VisionConstants.k_isRightReef  = true;
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
