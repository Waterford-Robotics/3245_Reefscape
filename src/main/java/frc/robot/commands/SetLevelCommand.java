// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Command;

// Raises Elevator
public class SetLevelCommand extends Command {

  // Side of the Reef
  String m_level;

  // Constructor
  public SetLevelCommand(String level) {
    
    // Level
    m_level = level;
  }

  // Reset timer when the command starts executing
  public void initialize() {
  }
  
  // Actual command
  public void execute() {

    // Set to L1
    if (m_level.equals("L1")) {
      ElevatorConstants.k_elevatorSetting = "L1";
    }

    // Set to L2
    if (m_level.equals("L2")) {
      ElevatorConstants.k_elevatorSetting = "L2";
    }

    // Set to L3
    if (m_level.equals("L3")) {
      ElevatorConstants.k_elevatorSetting = "L3";
    }

    // Set to L4
    if (m_level.equals("L4")) {
      ElevatorConstants.k_elevatorSetting = "L4";
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
