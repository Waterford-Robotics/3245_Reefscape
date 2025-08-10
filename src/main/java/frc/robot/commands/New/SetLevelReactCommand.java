// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.New;

import frc.robot.Constants.ReactConstants;
import edu.wpi.first.wpilibj2.command.Command;

// Sets level manually for autos and as a backup for op controller
public class SetLevelReactCommand extends Command {

  // Level of the Reef
  String m_level;

  // Constructor
  public SetLevelReactCommand(String level) {
    
    // Level
    m_level = level;
  }

  // Reset timer when the command starts executing
  public void initialize() {
  }
  
  // Actual command
  public void execute() {

    // Set to L1
    if (m_level.equals("1")) {
      ReactConstants._levelSelection = "1";
    }

    // Set to L2
    if (m_level.equals("2")) {
      ReactConstants._levelSelection = "2";
    }

    // Set to L3
    if (m_level.equals("3")) {
      ReactConstants._levelSelection = "3";
    }

    // Set to L4
    if (m_level.equals("4")) {
      ReactConstants._levelSelection = "4";
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {
    ReactConstants._scoreCoral = false;
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return true;
  }
}
