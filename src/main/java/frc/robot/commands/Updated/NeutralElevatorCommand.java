// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Updated;

import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

// Neutrals Elevator
public class NeutralElevatorCommand extends Command {

  // Uses Elevator and Subsystems
  ElevatorSubsystem m_elevatorSubsystem;

  // Constructor
  public NeutralElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  // Reset timer when the command starts executing
  public void initialize() {
  }
  
  // Actual command
  public void execute() {
    m_elevatorSubsystem.setNeutral();
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {

    // Neutral Motors again for safety
    m_elevatorSubsystem.setNeutral();
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return true;
  }
}
