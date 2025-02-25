package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorPIDConstants;

public class WristSubsystem extends SubsystemBase{

  private TalonFX m_wrist;
  private TalonFX m_shooter;

  private TalonFXConfiguration wristConfig;
  private TalonFXConfiguration shooterConfig;

  private Angle lastDesiredPosition;

  public WristSubsystem() {
    
    m_wrist = new TalonFX(MotorIDConstants.k_wristKrakenID, "Elevator/Coral");
    m_shooter = new TalonFX(MotorIDConstants.k_shooterKrakenID, "Elevator/Coral");

    lastDesiredPosition = Units.Rotations.of(0);

    wristConfig = new TalonFXConfiguration();
    wristConfig.Slot0.kP = MotorPIDConstants.k_wristP;
    wristConfig.Slot0.kI = MotorPIDConstants.k_wristI;
    wristConfig.Slot0.kD = MotorPIDConstants.k_wristD;
    wristConfig.Slot0.kS = MotorPIDConstants.k_wristS;
    wristConfig.Slot0.kV = MotorPIDConstants.k_wristV;
    wristConfig.Slot0.kG = MotorPIDConstants.k_wristG;

    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.CurrentLimits.SupplyCurrentLimit = WristConstants.k_supplyCurrentLimit;
    wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; 
    wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Rotations.of(5).in(Units.Rotations);
    wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Rotations.of(0).in(Units.Rotations); // Starting position
    wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    wristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 

    shooterConfig = new TalonFXConfiguration();

    // PID Stuff (Shooter)
    shooterConfig.Slot0.kP = MotorPIDConstants.k_shooterkP;
    shooterConfig.Slot0.kI = MotorPIDConstants.k_shooterkI;
    shooterConfig.Slot0.kD = MotorPIDConstants.k_shooterkD;
    shooterConfig.Slot0.kS = MotorPIDConstants.k_shooterkS;
    shooterConfig.Slot0.kV = MotorPIDConstants.k_shooterkV;

    // Kraken Configs
    shooterConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = WristConstants.k_shooterRampRate;
    shooterConfig.MotorOutput.PeakForwardDutyCycle = WristConstants.k_shooterClosedMaxSpeed;
    shooterConfig.MotorOutput.PeakReverseDutyCycle = -WristConstants.k_shooterClosedMaxSpeed;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = WristConstants.k_supplyCurrentLimit;

    m_wrist.getConfigurator().apply(wristConfig, 0.05);
    m_shooter.getConfigurator().apply(shooterConfig, 0.05);
  }

  public Angle getWristPosition(){
    return Units.Rotations.of(m_wrist.get());
  }

  public void setPosition(Angle angle){
    m_wrist.setControl(new PositionVoltage(angle.in(Units.Rotations)));
    lastDesiredPosition = angle;
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

  public void shoot(){
    m_shooter.set(WristConstants.k_shootSpeed);
  }

  public void stopShooter(){
    m_shooter.set(0);
  }

  public void intake(){
    m_shooter.set(-WristConstants.k_intakeSpeed);
  }

  public void periodic() {
    // This method will be called once per scheduler run
    /*
    SmartDashboard.putNumber("Wrist/Pos", Units.Rotations.of(m_wrist.getPosition().getValueAsDouble()).magnitude());
    SmartDashboard.putString("Wrist/Units", m_wrist.getPosition().getUnits());
    SmartDashboard.putNumber("Wrist/CLO", m_wrist.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Wrist/Output", m_wrist.get());
    SmartDashboard.putNumber("Wrist/Inverted", m_wrist.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Wrist/Current", m_wrist.getSupplyCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Wrist/Last Desired Position", lastDesiredPosition.magnitude());
    */
  }
}
