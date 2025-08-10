package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReactDashSubsystem extends SubsystemBase {

  public static final NetworkTable ReactDash = NetworkTableInstance.getDefault().getTable("ReactDash");

  public static final String AUTO_TAB_NAME = "Autonomous";
  public static final String TELEOP_TAB_NAME = "Teleop";

  private StringPublisher m_goTotabPub;
  private StringPublisher m_alliancePub;
  private IntegerPublisher m_locationPub;
  private IntegerPublisher m_matchTimePub;
  
  private BooleanPublisher m_joystick0Pub;
  private BooleanPublisher m_joystick1Pub;

  public ReactDashSubsystem() {

    var autoTable = ReactDashSubsystem.ReactDash.getSubTable("Main");

    m_goTotabPub = autoTable.getStringTopic("rpub/goTotab").publish();
    m_alliancePub = autoTable.getStringTopic("rpub/alliance").publish();
    m_locationPub = autoTable.getIntegerTopic("rpub/driverStation").publish();
    m_matchTimePub = autoTable.getIntegerTopic("rpub/matchTime").publish();

    m_joystick0Pub = autoTable.getBooleanTopic("rpub/joystick0").publish();
    m_joystick1Pub = autoTable.getBooleanTopic("rpub/joystick1").publish();
  }

  public void periodic() {
    m_locationPub.set(DriverStation.getLocation().orElse(0));
    m_alliancePub.set(DriverStation.getAlliance().get().toString());
    m_matchTimePub.set((int) DriverStation.getMatchTime());
    m_joystick0Pub.set(DriverStation.getJoystickIsXbox(0));
    m_joystick1Pub.set(DriverStation.getJoystickIsXbox(1));
  }

  public void SwitchTab(String tab) {
    m_goTotabPub.set(tab);
  }
}
