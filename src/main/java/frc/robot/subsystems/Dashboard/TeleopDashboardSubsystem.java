package frc.robot.subsystems.Dashboard;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ReactConstants;

public class TeleopDashboardSubsystem extends SubsystemBase {
  
  private BooleanSubscriber m_raiseElevatorSub;
  private StringSubscriber m_intakeSub;
  private StringSubscriber m_sourceLaneSub;
  private StringSubscriber m_ledSelectionSub;
  private StringSubscriber m_levelSelectionSub;
  private StringSubscriber m_sideSelectionSub;
  private BooleanSubscriber m_scoreCoralSub;
  private BooleanSubscriber m_resetGyroSub;
  private BooleanSubscriber m_neutralElevatorSub;
  private BooleanSubscriber m_zeroElevatorSub;
  private BooleanSubscriber m_playOrchestraSub;

  private StringPublisher m_levelSelectionPub;
  private StringPublisher m_sideSelectionPub;
  private BooleanPublisher m_tivPub;

  public TeleopDashboardSubsystem(){

    var teleopTable = ReactDashSubsystem.ReactDash.getSubTable("Main");

    m_raiseElevatorSub = teleopTable.getBooleanTopic("dpub/raiseElevator").subscribe(false);
    m_intakeSub = teleopTable.getStringTopic("dpub/intakeType").subscribe("manual");
    m_sourceLaneSub = teleopTable.getStringTopic("dpub/coralStationLane").subscribe("center");
    m_ledSelectionSub = teleopTable.getStringTopic("dpub/ledSelection").subscribe("NONE");
    m_levelSelectionSub = teleopTable.getStringTopic("dpub/reefLevel").subscribe("4");
    m_sideSelectionSub = teleopTable.getStringTopic("dpub/reefSide").subscribe("R");
    m_scoreCoralSub = teleopTable.getBooleanTopic("dpub/scoreCoral").subscribe(false);
    m_resetGyroSub = teleopTable.getBooleanTopic("dpub/zeroGyro").subscribe(false);
    m_neutralElevatorSub = teleopTable.getBooleanTopic("dpub/neutralElevator").subscribe(false);
    m_zeroElevatorSub = teleopTable.getBooleanTopic("dpub/zeroElevator").subscribe(false);
    m_playOrchestraSub = teleopTable.getBooleanTopic("dpub/playOrchestra").subscribe(false);

    m_levelSelectionPub = teleopTable.getStringTopic("rpub/levelSelection").getEntry("4");
    m_sideSelectionPub = teleopTable.getStringTopic("rpub/sideSelection").getEntry("R");
    m_tivPub = teleopTable.getBooleanTopic("rpub/tiv").getEntry(false);
  }

  public void periodic() {
    // Update Values
    ReactConstants._raiseElevator = m_raiseElevatorSub.get();
    ReactConstants._intakeType = m_intakeSub.get();
    ReactConstants._coralStationLane = m_sourceLaneSub.get();
    ReactConstants._ledSelection = m_ledSelectionSub.get();

    ReactConstants._levelSelection = m_levelSelectionSub.get();
    ReactConstants._sideSelection = m_sideSelectionSub.get();

    ReactConstants._scoreCoral = m_scoreCoralSub.get();
    ReactConstants._resetGyro = m_resetGyroSub.get();
    ReactConstants._neutralElevator = m_neutralElevatorSub.get();
    ReactConstants._zeroElevator = m_zeroElevatorSub.get();
    ReactConstants._playOrchestra = m_playOrchestraSub.get();

    // Update Publishers
    m_levelSelectionPub.set(ReactConstants._levelSelection);
    m_sideSelectionPub.set(ReactConstants._sideSelection);
    m_tivPub.set(ReactConstants._tiv);

    // Update Triggers
    ReactConstants._triggerLEDScoreSignal = ReactConstants._ledSelection.equals("SCORE_SIGNAL");
    ReactConstants._triggerLEDManualSignal = ReactConstants._ledSelection.equals("MANUAL_SIGNAL");
    ReactConstants._triggerLEDParkSignal = ReactConstants._ledSelection.equals("PARK_SIGNAL");
    ReactConstants._triggerLEDAllianceSignal = !ReactConstants._tiv && ReactConstants._ledSelection.equals("NONE");
    ReactConstants._triggerLEDLimelightSignal = ReactConstants._tiv && ReactConstants._ledSelection.equals("NONE");
  }
}
