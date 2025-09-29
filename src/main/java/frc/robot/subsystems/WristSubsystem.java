package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.WristConfigs;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIDConstants;

public class WristSubsystem extends SubsystemBase{

  private TalonFX m_wrist;

  public WristSubsystem() {
    
    m_wrist = new TalonFX(MotorIDConstants.k_wristKrakenID, "Elevator/Coral");

    m_wrist.getConfigurator().apply(WristConfigs.WRIST_TALON_FX_CONFIGURATION, 0.05);

    MotorConstants.k_orchestra.addInstrument(m_wrist); 
  }

  public Angle getWristPosition(){
    return Units.Rotations.of(m_wrist.get());
  }

  public void setPosition(Angle angle){
    m_wrist.setControl(new PositionVoltage(angle.in(Units.Rotations)).withEnableFOC(true));
  }

  public void setNeutral() {
    m_wrist.setControl(new NeutralOut());
  }

  public void resetSensorPosition(Angle setpoint) {
    m_wrist.setPosition(setpoint.in(Units.Rotations));
  }

  public double getCurrentPosition() {
    return m_wrist.getPosition().getValueAsDouble();
  }

  public double getCurrentVelocity() {
    return m_wrist.getVelocity().getValueAsDouble();
  }

  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist/Pos", Units.Rotations.of(m_wrist.getPosition().getValueAsDouble()).magnitude());
    
    /*
    SmartDashboard.putString("Wrist/Units", m_wrist.getPosition().getUnits());
    SmartDashboard.putNumber("Wrist/CLO", m_wrist.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Wrist/Output", m_wrist.get());
    SmartDashboard.putNumber("Wrist/Inverted", m_wrist.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Wrist/Current", m_wrist.getSupplyCurrent().getValueAsDouble());
    */
  }
}
