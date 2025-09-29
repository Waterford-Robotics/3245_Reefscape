package frc.robot.subsystems.Dashboard;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ReactConstants;

public class AutoDashboardSubsystem extends SubsystemBase {
  
  private StringSubscriber m_selectedAutoSub;

  private StringPublisher m_selectedAutoPub;
  private StringArrayPublisher m_autoSelectorPub;

  public AutoDashboardSubsystem(){

    var autoTable = ReactDashSubsystem.ReactDash.getSubTable("Main");

    m_selectedAutoSub = autoTable.getStringTopic("dpub/selectedAuto").subscribe("NONE");

    m_selectedAutoPub = autoTable.getStringTopic("rpub/selectedAutoFromRobot").getEntry(ReactConstants._selectedAuto);
    m_autoSelectorPub = autoTable.getStringArrayTopic("rpub/autoSelector").getEntry(ReactConstants._autoSelector);
  }

  public void periodic() {
    
    // Get Auto
    ReactConstants._selectedAuto = m_selectedAutoSub.get();

    // Update Publishers
    m_selectedAutoPub.set(ReactConstants._selectedAuto);
    m_autoSelectorPub.set(ReactConstants._autoSelector);
  }
}
