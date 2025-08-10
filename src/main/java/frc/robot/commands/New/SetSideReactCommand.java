// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.New;

import frc.robot.Constants.ReactConstants;
import edu.wpi.first.wpilibj2.command.Command;

// Sets side of reef for autos and for op controller just in case
public class SetSideReactCommand extends Command {

  // Side of the Reef
  String m_side;

  // Constructor
  public SetSideReactCommand(String side) {
    
    // Level
    m_side = side;
  }

  // Reset timer when the command starts executing
  public void initialize() {}
  
  // Actual command
  public void execute() {

    // Set to L1
    if (m_side.equals("L")) {
      ReactConstants._sideSelection = "L";
    }

    // Set to L2
    if (m_side.equals("R")) {
      ReactConstants._sideSelection = "R";
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {}

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return true;
  }
}
