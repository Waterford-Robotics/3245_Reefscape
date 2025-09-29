// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorPIDConstants;
import frc.robot.Constants.SensorIDConstants;

// Elevator Subsystem yay yippee
public class ElevatorSubsystem extends SubsystemBase {
  // Init stuff
  private TalonFX m_elevatorKrakenLeft;
  private TalonFX m_elevatorKrakenRight;
  private TalonFXConfiguration krakenConfig;
  private CANcoder m_CANcoder;
  private int rotations = 0;
  private double CANcoderLastAngle = 0.5; // set bottom position as 180

  // Creates new elevator
  public ElevatorSubsystem() {

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

    // Init CANCoder
    m_CANcoder = new CANcoder(SensorIDConstants.k_CANcoderID);

    MotorConstants.k_orchestra.addInstrument(m_elevatorKrakenLeft); 
    MotorConstants.k_orchestra.addInstrument(m_elevatorKrakenRight); 
  }

  // Gets current position of elevator
  public Distance getElevatorPosition() {
    return Units.Inches.of(m_elevatorKrakenRight.getPosition().getValueAsDouble());
  }

  // Gets current position of CANcoder
  public double getCANcoderPosition(){
    return m_CANcoder.getAbsolutePosition().getValue().abs(Units.Rotation);
  }

  // Get elevator position in inches
  public Distance getElevatorPositionCANcoder() {
    return Units.Inches.of((rotations + getCANcoderPosition() - 0.5) * SensorIDConstants.k_CANcoderRotationHeight);
  }

  // Moves the elevator to position
  public void setPosition(Distance height) {

    // Right motor is leader
    m_elevatorKrakenRight.setControl(new PositionVoltage(height.in(Units.Inches)).withEnableFOC(true));

    // Left motor is follower
    m_elevatorKrakenLeft.setControl(new Follower(m_elevatorKrakenRight.getDeviceID(), false));
  }

  // Moves elevator to position with CANcoder
  public void setPositionCANcoder(Distance height) {
    double kp = 1;
    double kd = 0;
    double ki = 0;
    ProfiledPIDController elevatorPidController = new ProfiledPIDController(kp, ki, kd, null);
    m_elevatorKrakenRight.set(elevatorPidController.calculate(getElevatorPositionCANcoder().abs(Units.Inches),height.abs(Units.Inches)));
    m_elevatorKrakenLeft.setControl(new Follower(m_elevatorKrakenRight.getDeviceID(), false));
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
  
  // Resets CANcoder position to the value provided
  public void resetCANcoderPosition(Distance setpoint){
    double totalRotations = setpoint.in(Units.Inches)/SensorIDConstants.k_CANcoderRotationHeight + 0.5;
    rotations = (int) totalRotations;
    m_CANcoder.setPosition(totalRotations%1);
  }

  public double getCurrentPosition() {
    return m_elevatorKrakenRight.getPosition().getValueAsDouble();
  }

  public double getCurrentVelocity() {
    return m_elevatorKrakenRight.getVelocity().getValueAsDouble();
  }

  public void periodic() {
    // This method will be called once per scheduler run
    /*
    SmartDashboard.putNumber("Elevator/Left/Pos", m_elevatorKrakenLeft.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/CLO", m_elevatorKrakenLeft.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Output", m_elevatorKrakenLeft.get());
    SmartDashboard.putNumber("Elevator/Left/Inverted", m_elevatorKrakenLeft.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Current", m_elevatorKrakenLeft.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/CLO", m_elevatorKrakenRight.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Output", m_elevatorKrakenRight.get());
    SmartDashboard.putNumber("Elevator/Right/Inverted", m_elevatorKrakenRight.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Current", m_elevatorKrakenRight.getSupplyCurrent().getValueAsDouble());
    */

    // What did Shuffleboard say to the Programmer?
    // I'm drowning in numbers!
    SmartDashboard.putNumber("Elevator/Right/Pos", m_elevatorKrakenRight.getPosition().getValueAsDouble());
    // SmartDashboard.putString("Elevator/Setting", ElevatorConstants.k_elevatorSetting);
    if (getCANcoderPosition() - CANcoderLastAngle >= 0.5){
      rotations += 1;
    }
    if (getCANcoderPosition() - CANcoderLastAngle <= -0.5){
      rotations -= 1;
    }
    CANcoderLastAngle = getCANcoderPosition();
    SmartDashboard.putNumber("elevator position cancoder", getElevatorPositionCANcoder().in(Inches));
  }
}
