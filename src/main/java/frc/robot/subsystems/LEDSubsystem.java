package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;

  public class LEDSubsystem extends SubsystemBase {

    private Spark m_blinkinLeft;
    private Spark m_blinkinRight;

    private boolean m_colorNotSet;

    public LEDSubsystem(){
      m_blinkinLeft = new Spark(LEDConstants.blinkinPortLeft);
      m_blinkinRight = new Spark(LEDConstants.blinkinPortRight);

      setAllianceColor();

      m_colorNotSet = true;
    }
    // Bot Pose Target Space Relative [TX, TY, TZ, Pitch, Yaw, Roll]
    private double[] botPoseTargetSpace = new double[6];
    
    private boolean tiv;
    public void periodic() {
      var alliance = DriverStation.getAlliance();
      if (m_colorNotSet && alliance.isPresent()) {
        setAllianceColor();
        m_colorNotSet = false;
      }
      botPoseTargetSpace = NetworkTableInstance.getDefault().getTable(VisionConstants.k_limelightName).getEntry("botpose_targetspace").getDoubleArray(new double[6]);;
      tiv = (LimelightHelpers.getTV(VisionConstants.k_limelightName) 
      && botPoseTargetSpace[2] > VisionConstants.k_tzValidRange 
      && Math.abs(botPoseTargetSpace[4]) < VisionConstants.k_yawValidRange);
      if (tiv){
        setColorWavesForestLimelight();
      }
      else {
        setAllianceColor();
      }
    }

    public void setAllianceColor() {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()){
        if (alliance.get() == DriverStation.Alliance.Red) {
          setLavaPaletteRedAlliance();
        }
        else if (alliance.get() == DriverStation.Alliance.Blue){
          setOceanPaletteBlueAlliance();
        }
      }
    }

    public void setRainbowParty() {
      m_blinkinLeft.set(-0.97);
      m_blinkinRight.set(-0.97);
    }

    public void setOceanPaletteBlueAlliance() {
      m_blinkinLeft.set(-0.95);
      m_blinkinRight.set(-0.95);
    }

    public void setLavaPaletteRedAlliance() {
      m_blinkinLeft.set(-0.93);
      m_blinkinRight.set(-0.93);
    }

    public void setColorWavesForestLimelight() {
      m_blinkinLeft.set(-0.37);
      m_blinkinRight.set(-0.37);
    }

    public void turnOff() {
      m_blinkinLeft.set(0);
      m_blinkinRight.set(0);
    }
}
