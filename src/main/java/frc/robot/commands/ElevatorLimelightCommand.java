// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

// Raises Elevator
public class ElevatorLimelightCommand extends Command {

  // Uses Elevator and Subsystems
  ElevatorSubsystem m_elevatorSubsystem;
  boolean m_tiv;

  // Constructor
  public ElevatorLimelightCommand(ElevatorSubsystem elevatorSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  // Reset timer when the command starts executing
  public void initialize() {
  }
  
  // Actual command
  public void execute() {

    VisionConstants.k_botPoseTargetSpace = NetworkTableInstance
      .getDefault()
      .getTable(VisionConstants.k_limelightName)
      .getEntry("botpose_targetspace")
      .getDoubleArray(new double[6]
    );

    m_tiv = (LimelightHelpers.getTV(VisionConstants.k_limelightName) 
      && VisionConstants.k_botPoseTargetSpace[2] > VisionConstants.k_tzValidRangeElevator 
      && Math.abs(VisionConstants.k_botPoseTargetSpace[4]) < VisionConstants.k_yawValidRangeElevator
    );

    // Raise to L2
    if(m_tiv && ElevatorConstants.k_elevatorSetting.equals("L2")) {
      m_elevatorSubsystem.setPosition(ElevatorConstants.k_coralL2Height);
    }

    // Raise to L3
    if(m_tiv && ElevatorConstants.k_elevatorSetting.equals("L3")) {
      m_elevatorSubsystem.setPosition(ElevatorConstants.k_coralL3Height);
    }

    // Raise to L4
    if(m_tiv && ElevatorConstants.k_elevatorSetting.equals("L4")) {
      m_elevatorSubsystem.setPosition(ElevatorConstants.k_coralL4Height);
    }

    // Zero
    if (!m_tiv && m_elevatorSubsystem.getCurrentPosition() > 2) {
      m_elevatorSubsystem.setPosition(ElevatorConstants.k_zeroHeight);
    }

    if(m_elevatorSubsystem.getCurrentPosition() < 2 && m_elevatorSubsystem.getCurrentVelocity() == 0) {

      // Neutral Motors and Reset Encoder Values
      m_elevatorSubsystem.setNeutral();
      m_elevatorSubsystem.resetSensorPosition(ElevatorConstants.k_zeroHeight);
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
