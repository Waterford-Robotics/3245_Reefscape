package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

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

    public void periodic() {
      var alliance = DriverStation.getAlliance();
      if (m_colorNotSet && alliance.isPresent()) {
        setAllianceColor();
        m_colorNotSet = false;
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
