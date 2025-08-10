package frc.robot.subsystems.Limelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ReactConstants;
import frc.robot.Constants.VisionConstants;

// Handle basic Limelight stuff hehe
// Simplest subsystem ever
public class LimelightSubsystem extends SubsystemBase {

  // Easiest Constructor Ever
  public LimelightSubsystem() {}

  // This periodic is kinda power hungry?
  public void periodic() {
    
    // Get botpose_targetspace from Limelight through NT
    ReactConstants._botPoseTargetSpace = NetworkTableInstance
      .getDefault()
      .getTable(VisionConstants.k_limelightName)
      .getEntry("botpose_targetspace")
      .getDoubleArray(new double[6]
    );

    // Update TIV conditions based on the latest reading
    ReactConstants._tiv = (LimelightHelpers.getTV(VisionConstants.k_limelightName) 
      && ReactConstants._botPoseTargetSpace[2] > VisionConstants.k_tzValidRange 
      && Math.abs(ReactConstants._botPoseTargetSpace[4]) < VisionConstants.k_yawValidRange
    );
  }
}
