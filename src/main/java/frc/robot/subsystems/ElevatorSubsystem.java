// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorPIDConstants;

// Elevator Subsystem yay yippee
public class ElevatorSubsystem extends SubsystemBase {
  // Init stuff
  private TalonFX m_elevatorKrakenLeft;
  private TalonFX m_elevatorKrakenRight;
  private TalonFXConfiguration krakenConfig;
  // Distance currentLeftPosition = Units.Inches.of(0);
  // Distance currentRightPosition = Units.Inches.of(0);
  private Distance lastDesiredPosition;

  // Creates new elevator
  public ElevatorSubsystem() {

    // Last position is home position
    lastDesiredPosition = Units.Inches.of(0);

    // Krakens
    m_elevatorKrakenLeft = new TalonFX(MotorIDConstants.k_elevatorKrakenLeftID, "Elevator/Coral");
    m_elevatorKrakenRight = new TalonFX(MotorIDConstants.k_elevatorKrakenRightID, "Elevator/Coral");

    // Init krakenConfig
    krakenConfig = new TalonFXConfiguration();

    // PID Stuff
    krakenConfig.Slot0.kP = MotorPIDConstants.k_elevatorkP;
    krakenConfig.Slot0.kI = MotorPIDConstants.k_elevatorkI;
    krakenConfig.Slot0.kD = MotorPIDConstants.k_elevatorkD;
    krakenConfig.Slot0.kS = MotorPIDConstants.k_elevatorkS;
    krakenConfig.Slot0.kV = MotorPIDConstants.k_elevatorkV;
    krakenConfig.Slot0.kG = MotorPIDConstants.k_elevatorkG;
    krakenConfig.Slot0.kA = MotorPIDConstants.k_elevatorkA;

    // Kraken Configs
    krakenConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    krakenConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.k_elevatorSupplyCurrentLimit;
    krakenConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Motor Limitations
    krakenConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; // No breaking elevator
    krakenConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Inches.of(49).in(Units.Inches);
    krakenConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    krakenConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Inches.of(0).in(Units.Inches); // Starting position

    // Elevator Mode
    krakenConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    
    // Elevator motors will provide feedback in INCHES the carriage has moved
    krakenConfig.Feedback.SensorToMechanismRatio = 0.4545;

    // Apply Configs, Inversion, Control requests
    m_elevatorKrakenLeft.getConfigurator().apply(krakenConfig, 0.5);
    m_elevatorKrakenRight.getConfigurator().apply(krakenConfig, 0.5);

    // (Deprecated)
    // m_elevatorKrakenLeft.setInverted(false); 
    // m_elevatorKrakenRight.setInverted(false);
  }

  // Gets current position of elevator
  public Distance getElevatorPosition() {
    return Units.Inches.of(m_elevatorKrakenRight.getPosition().getValueAsDouble());
  }

  // Moves the elevator to position
  public void setPosition(Distance height) {

    // Right motor is leader
    m_elevatorKrakenRight.setControl(new PositionVoltage(height.in(Units.Inches)));

    // Left motor is follower
    m_elevatorKrakenLeft.setControl(new Follower(m_elevatorKrakenRight.getDeviceID(), false));

    // Updates the last desired position
    lastDesiredPosition = height;
  }

  // Sets motors to neutral mode
  public void setNeutral() {
    m_elevatorKrakenRight.setControl(new NeutralOut());
    m_elevatorKrakenLeft.setControl(new NeutralOut());
  }

  // Resets the sensor position to the value provided
  public void resetSensorPosition(Distance setpoint) {
    m_elevatorKrakenRight.setPosition(setpoint.in(Units.Inches));
    m_elevatorKrakenLeft.setPosition(setpoint.in(Units.Inches));
  }

  public double getCurrentPosition() {
    return m_elevatorKrakenRight.getPosition().getValueAsDouble();
  }

  public double getCurrentVelocity() {
    return m_elevatorKrakenRight.getVelocity().getValueAsDouble();
  }

  public void periodic() {
    /*
    // This method will be called once per scheduler run
    currentLeftPosition = Units.Inches.of(m_elevatorKrakenLeft.getPosition().getValueAsDouble());
    currentRightPosition = Units.Inches.of(m_elevatorKrakenRight.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Pos", m_elevatorKrakenLeft.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/CLO", m_elevatorKrakenLeft.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Output", m_elevatorKrakenLeft.get());
    SmartDashboard.putNumber("Elevator/Left/Inverted", m_elevatorKrakenLeft.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Current", m_elevatorKrakenLeft.getSupplyCurrent().getValueAsDouble());

    // What did Shuffleboard say to the Programmer?
    // I'm drowning in numbers!
    */

    SmartDashboard.putNumber("Elevator/Right/Pos", m_elevatorKrakenRight.getPosition().getValueAsDouble());
    /*
    SmartDashboard.putNumber("Elevator/Right/CLO", m_elevatorKrakenRight.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Output", m_elevatorKrakenRight.get());
    SmartDashboard.putNumber("Elevator/Right/Inverted", m_elevatorKrakenRight.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Current", m_elevatorKrakenRight.getSupplyCurrent().getValueAsDouble());
    */

    SmartDashboard.putNumber("Elevator/Last Desired Position", lastDesiredPosition.magnitude()); 
  }
}
