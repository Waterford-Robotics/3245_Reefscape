// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Updated;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ReactConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

// Raises Elevator (the old way smh)
public class SetElevatorCommand extends Command {

  // Uses Elevator and Subsystems
  ElevatorSubsystem m_elevatorSubsystem;
  String m_level;

  // Constructor
  public SetElevatorCommand(ElevatorSubsystem elevatorSubsystem, String level) {
        
    // Definitions and setting parameters are equal to members!
    m_elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);

    // Level
    m_level = level;
  }

  // Reset timer when the command starts executing
  public void initialize() {
  }
  
  // Actual command
  public void execute() {

    // Zero
    if (m_level.equals("zero")) {
      m_elevatorSubsystem.setPositionCANcoder(ElevatorConstants.k_zeroHeight);
    }

    // Raise to L1
    if(m_level.equals("L1")) {
      m_elevatorSubsystem.setPositionCANcoder(ElevatorConstants.k_coralL1Height);
    }

    // Raise to L2
    if(m_level.equals("L2")) {
      m_elevatorSubsystem.setPositionCANcoder(ElevatorConstants.k_coralL2Height);
    }

    // Raise to L3
    if(m_level.equals("L3")) {
      m_elevatorSubsystem.setPositionCANcoder(ElevatorConstants.k_coralL3Height);
    }

    // Raise to L4
    if(m_level.equals("L4")) {
      m_elevatorSubsystem.setPositionCANcoder(ElevatorConstants.k_coralL4Height);
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {
    ReactConstants._triggerElevatorScore = false;
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return true;
  }
}
