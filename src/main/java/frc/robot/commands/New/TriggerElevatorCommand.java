// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.New;

import frc.robot.Constants.ReactConstants;
import edu.wpi.first.wpilibj2.command.Command;

// Trigger Elevator Raise
public class TriggerElevatorCommand extends Command {

  // It can be "RESET" or "SCORE"
  private String m_status;

  // Constructor
  public TriggerElevatorCommand(String status) {
    m_status = status;
  }

  // Init
  public void initialize() {}
  
  // Actual command
  public void execute() {

    // Trigger commands if the time is right
    if (m_status.equals("SCORE")) ReactConstants._triggerElevatorScore = true;
    if (m_status.equals("RESET")) ReactConstants._triggerElevatorReset = true;
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {}

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return true;
  }
}
