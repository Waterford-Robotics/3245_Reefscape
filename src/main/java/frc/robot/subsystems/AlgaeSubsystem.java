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
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorPIDConstants;

public class AlgaeSubsystem extends SubsystemBase{

  private TalonFX m_algaeWrist;
  private TalonFX m_algaeIntake;

  private TalonFXConfiguration algaeWristConfig;
  private TalonFXConfiguration algaeIntakeConfig;

  private Angle lastDesiredPosition;

  public AlgaeSubsystem() {
    
    m_algaeWrist = new TalonFX(MotorIDConstants.k_algaeWristID, "Elevator/Coral");
    m_algaeIntake = new TalonFX(MotorIDConstants.k_algaeIntakeID, "Elevator/Coral");

    lastDesiredPosition = Units.Rotations.of(0);

    algaeWristConfig = new TalonFXConfiguration();
    algaeWristConfig.Slot0.kP = MotorPIDConstants.k_algaeWristkP;
    algaeWristConfig.Slot0.kI = MotorPIDConstants.k_algaeWristkI;
    algaeWristConfig.Slot0.kD = MotorPIDConstants.k_algaeWristkD;
    algaeWristConfig.Slot0.kS = MotorPIDConstants.k_algaeWristkS;
    algaeWristConfig.Slot0.kV = MotorPIDConstants.k_algaeWristkV;
    algaeWristConfig.Slot0.kG = MotorPIDConstants.k_algaeWristkG;

    algaeWristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    algaeWristConfig.CurrentLimits.SupplyCurrentLimit = AlgaeConstants.k_algaeSupplyCurrentLimit;
    algaeWristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; 
    algaeWristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Rotations.of(0.7).in(Units.Rotations); // TODO: Check me
    algaeWristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    algaeWristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Rotations.of(2).in(Units.Rotations); // Starting position
    algaeWristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    algaeWristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // algaeWristConfig.Feedback.SensorToMechanismRatio = 0.4545;

    algaeIntakeConfig = new TalonFXConfiguration();

    // PID Stuff (Shooter)
    algaeIntakeConfig.Slot0.kP = MotorPIDConstants.k_algaeIntakekP;
    algaeIntakeConfig.Slot0.kI = MotorPIDConstants.k_algaeIntakekI;
    algaeIntakeConfig.Slot0.kD = MotorPIDConstants.k_algaeIntakekD;
    algaeIntakeConfig.Slot0.kS = MotorPIDConstants.k_algaeIntakekS;
    algaeIntakeConfig.Slot0.kV = MotorPIDConstants.k_algaeIntakekV;

    // Kraken Configs
    algaeIntakeConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = AlgaeConstants.k_algaeRampRate;
    algaeIntakeConfig.MotorOutput.PeakForwardDutyCycle = AlgaeConstants.k_algaeClosedMaxSpeed;
    algaeIntakeConfig.MotorOutput.PeakReverseDutyCycle = -AlgaeConstants.k_algaeClosedMaxSpeed;
    algaeIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    algaeIntakeConfig.CurrentLimits.SupplyCurrentLimit = AlgaeConstants.k_algaeSupplyCurrentLimit;

    m_algaeWrist.getConfigurator().apply(algaeWristConfig, 0.05);
    m_algaeIntake.getConfigurator().apply(algaeIntakeConfig, 0.05);
  }

    public Angle getWristPosition(){
      return Units.Rotations.of(m_algaeWrist.get());
    }

    public void setPosition(Angle angle){
      m_algaeWrist.setControl(new PositionVoltage(angle.in(Units.Rotations)));
      lastDesiredPosition = angle;
    }

    public void setNeutral() {
      m_algaeWrist.setControl(new NeutralOut());
    }

    public void resetSensorPosition(Angle setpoint) {
      m_algaeWrist.setPosition(setpoint.in(Units.Rotations));
    }

    public void shoot(){
      m_algaeIntake.set(0.1);
    }

    public void stopShooter(){
      m_algaeIntake.set(0);
    }

    public void intake(){
      m_algaeIntake.set(-0.1);
    }

    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("Wrist/Pos", m_algaeWrist.getPosition().getValueAsDouble());
      SmartDashboard.putString("Wrist/Units", m_algaeWrist.getPosition().getUnits());
      SmartDashboard.putNumber("Wrist/CLO", m_algaeWrist.getClosedLoopOutput().getValueAsDouble());
      SmartDashboard.putNumber("Wrist/Output", m_algaeWrist.get());
      SmartDashboard.putNumber("Wrist/Inverted", m_algaeWrist.getAppliedRotorPolarity().getValueAsDouble());
      SmartDashboard.putNumber("Wrist/Current", m_algaeWrist.getSupplyCurrent().getValueAsDouble());
  }
}
