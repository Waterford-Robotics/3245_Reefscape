// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SetElevatorCommand;
import frc.robot.commands.SetLevelCommand;
import frc.robot.commands.SetReefCommand;
import frc.robot.commands.SetWristCommand;
import frc.robot.commands.ZeroElevatorCommand;
import frc.robot.commands.ZeroWristCommand;
import frc.robot.commands.RunIntakeForSecsCommand;
import frc.robot.commands.RunShootForSecsCommand;
import frc.robot.commands.RunShootForSecsSpeedCommand;
import frc.robot.commands.AimNRangeAlgaeRemovalCommand;
import frc.robot.commands.AimNRangeAutoCommand;
import frc.robot.commands.AimNRangeAutoCoralStationCommand;
import frc.robot.commands.AimNRangeCommand;
import frc.robot.commands.LEDColorChangeCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// This class is where the bulk of the robot should be declared.  Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the Robot
// periodic methods (other than the scheduler calls).  Instead, the structure of the robot
// (including subsystems, commands, and button mappings) should be declared here.
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final WristSubsystem m_wristSubsystem = new WristSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  // private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  // Create New Choosing Option in SmartDashboard for Autos
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.k_driverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(ControllerConstants.k_operatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    // Makes the drive command the default command (good!)
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
    m_elevatorSubsystem.setDefaultCommand(elevatorLimelightCommand);

    // Named Command Configuration
    NamedCommands.registerCommand("Score L4", AimNRangescoreL4Command);
    NamedCommands.registerCommand("Score Left Reef", AimNRangescoreAllAutoLeftCommand);
    NamedCommands.registerCommand("Score Right Reef", AimNRangescoreAllAutoRightCommand);
    NamedCommands.registerCommand("Finish Scoring", AimNRangeFinishCommand);
    NamedCommands.registerCommand("Lower Elevator", new SetLevelCommand("L1"));
    NamedCommands.registerCommand("Raise Wrist", RaiseWristCommand);
    NamedCommands.registerCommand("Intake", IntakeAutoCommand);
    NamedCommands.registerCommand("Prepare to Score", Akatsuki);
    NamedCommands.registerCommand("Score L4 Left", AimNRangeLeftNoElevatorCommand);
    NamedCommands.registerCommand("Score L4 Right", AimNRangeRightNoElevatorCommand);
    NamedCommands.registerCommand("Zero Gyro", new InstantCommand(() -> m_swerveSubsystem.zeroGyro(), m_swerveSubsystem));
    NamedCommands.registerCommand("Position Coral Station Left", positionNIntakeAutoCoralStationLeft);
    NamedCommands.registerCommand("Position Coral Station Right", positionNIntakeAutoCoralStationRight);

    /* 
     * KEY:
     * U - UNCONFIRMED
     * P - Preferred
    */

    // Autos!

    // COMP AUTOS
    // m_chooser.addOption("3C: BL-R, FL-R, FL-L REVISED", m_swerveSubsystem.getAutonomousCommand("3C BL-R, FL-R, FL-L REVISED")); // U (P)
    // m_chooser.addOption("3C BR-L, FR-L, FR-R *", m_swerveSubsystem.getAutonomousCommand("3C BR-L, FR-L, FR-R")); // U (P)

    m_chooser.addOption("2.5C: BL-R, FL-R *", m_swerveSubsystem.getAutonomousCommand("2.5C BL-R, FL-R REVISED"));
    m_chooser.addOption("2.5C: BR-L, FR-L *", m_swerveSubsystem.getAutonomousCommand("2.5C BR-L, FR-L REVISED")); 

    m_chooser.addOption("1C: BC-L", m_swerveSubsystem.getAutonomousCommand("1C BC-L")); 
    m_chooser.addOption("1C: BC-R", m_swerveSubsystem.getAutonomousCommand("1C BC-R")); 

    m_chooser.addOption("0C: Center Leave", m_swerveSubsystem.getAutonomousCommand("CenterLeave"));
    m_chooser.addOption("0C: Left Leave", m_swerveSubsystem.getAutonomousCommand("LeftLeave")); 
    m_chooser.addOption("0C: Right Leave", m_swerveSubsystem.getAutonomousCommand("RightLeave")); // U

    // Puts a chooser on the SmartDashboard!
    SmartDashboard.putData("AutoMode", m_chooser);
  }

  // Trigger & Button Bindings!
  private void configureBindings() {
    
    /*
     * DRIVER CONTROLLER
     * A - Set L1
     * B - Set L2
     * X - Set L3
     * Y - Set L4
     * Right Bump - Raise Wrist
     * Left Bump - Lower Wrist
     * Right Trig - Intake and Set X
     * Left Trig - Align with Coral Station
     * Start - Reset Gyro
     * Back - Reset Elevator
     * POV UP - Position and Score (REQUIRES [A, B, X, Y] ELEVATOR SETTING!)
     * POV DOWN - Shoot Slowly (For L1)
     * POV LEFT - Set Destination to Left Branch
     * POV RIGHT - Set Destination to Right Branch
     */
    
    // Automatic Reset Elevator - "A" Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_A)
      .onTrue(
        new SetLevelCommand("L1")
      );
    
    // Automated Coral L2 - "B" Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_B)
      .onTrue(
        new SetLevelCommand("L2")
      );

    // Automated Coral L3 - "X" Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_X)
      .onTrue(
        new SetLevelCommand("L3")
      );

    // Automated Coral L4 - "Y" Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_Y)
      .onTrue(
        new SetLevelCommand("L4")
      );
    
    // Raise Wrist - Right Bump
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_rightbump)
      .onTrue(
        RaiseWristCommand
      );

    // Lower Wrist - Left Bump
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_leftbump)
      .onTrue(
        LowerWristCommand
      );

    // Intake and Set X - Right Trig
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
      .whileTrue(
        new InstantCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem).alongWith(
        new RunCommand(() -> m_swerveSubsystem.setx()))
      )
      .onFalse(
        new InstantCommand(() -> m_intakeSubsystem.stopShooter(), m_intakeSubsystem).alongWith(
        new InstantCommand(() -> m_swerveSubsystem.driveCommandLimelight(0, 0, 0), m_swerveSubsystem))
      );

    // Align with Coral Station - Left Trig
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_lefttrig) > 0.05)
      .onTrue(
        new AimNRangeAutoCoralStationCommand(m_swerveSubsystem, true)
      );

    // Reset Gyro - Start Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_start)
      .onTrue(
        new InstantCommand(() -> m_swerveSubsystem.zeroGyro(), m_swerveSubsystem)
      );
    
    // Reset Elevator - Back Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_back)
    .onTrue(
      ResetElevatorCommand
    );

    // Set AimNRange Destination to "Right"
    new POVButton(m_driverController.getHID(), ControllerConstants.k_dpadRight)
      .onTrue(
        new SetReefCommand("right")
      );

    // Set AimNRange Destination to "Left"
    new POVButton(m_driverController.getHID(), ControllerConstants.k_dpadLeft)
      .onTrue(
        new SetReefCommand("left")
      );

    // All in One!
    new POVButton(m_driverController.getHID(), ControllerConstants.k_dpadup)
      .onTrue(
        AimNRangescoreAllCommand
      );

    // L1 Eject TODO: Test L1 Eject (I'm bad rawr)
    new POVButton(m_driverController.getHID(), ControllerConstants.k_dpadDown)
      .whileTrue(
        new RunShootForSecsSpeedCommand(m_intakeSubsystem, 1, true, 0.1)
      );

    /*
     * OPERATOR CONTROLLER
     * A - Lower Elevator
     * B - Raise L2
     * X - Raise L3
     * Y - Raise L4
     * Right Bump - Raise Wrist
     * Left Bump - Lower Wrist
     * Right Trig - Intake Coral
     * Left Trig - Shoot Coral
     * Start - Neutral Elevator Manual
     * Back - Zero Elevator Manual
     * POV UP - 
     * POV DOWN - 
     * POV LEFT - Set Destination to Left Branch
     * POV RIGHT - Set Destination to Right Branch
     */
    
    // Lower Elevator - "A" Button
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_A)
      .onTrue(
        new SetLevelCommand("L1")
      );
    
    // Raise Elevator to L2 - "B" Button TODO: Test manual elevator setting
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_B)
      .onTrue(
        new SetLevelCommand("L2")
      );

    // Raise Elevator to L3 - "X" Button
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_X)
      .onTrue(
        new SetLevelCommand("L3")
      );

    // Raise Elevator to L4 - "Y" Button
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_Y)
      .onTrue(
        new SetLevelCommand("L4")
      );
    
    // Raise Wrist - Right Bump
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_rightbump)
      .onTrue(
        RaiseWristCommand
      );

    // Lower Wrist - Left Bump
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_leftbump)
      .onTrue(
        LowerWristCommand
      );

    // Intake - Right Trig
    new Trigger(() -> m_operatorController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
      .whileTrue(
        new InstantCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem)
      )
      .onFalse(
        new InstantCommand(() -> m_intakeSubsystem.stopShooter(), m_intakeSubsystem)
      );

    // Eject Algae - Left Trig
    new Trigger(() -> m_operatorController.getRawAxis(ControllerConstants.k_lefttrig) > 0.05)
      .whileTrue(
        new InstantCommand(() -> m_intakeSubsystem.shoot(), m_intakeSubsystem)
      )
      .onFalse(
        new InstantCommand(() -> m_intakeSubsystem.stopShooter(), m_intakeSubsystem)
      );

    // Neutral Elevator - Start Button
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_start)
      .onTrue(
        new InstantCommand(() -> m_elevatorSubsystem.setNeutral(), m_elevatorSubsystem)
      );
    
    // Reset Elevator Manually - Back Button
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_back)
    .onTrue(
      new InstantCommand(() -> m_elevatorSubsystem.resetSensorPosition(ElevatorConstants.k_zeroHeight), m_elevatorSubsystem)
    );

    // Set AimNRange Destination to "Right"
    new POVButton(m_operatorController.getHID(), ControllerConstants.k_dpadRight)
      .onTrue(
        new SetReefCommand("right")
      );

    // Set AimNRange Destination to "Left"
    new POVButton(m_operatorController.getHID(), ControllerConstants.k_dpadLeft)
      .onTrue(
        new SetReefCommand("left")
      );

    /* 
    // Algae Intake
    new POVButton(m_driverController.getHID(), ControllerConstants.k_dpadup)
    .whileTrue(
      new InstantCommand(() -> m_algaeSubsystem.intake(), m_algaeSubsystem)
    )
    .onFalse(
      new InstantCommand(() -> m_algaeSubsystem.stopShooter(), m_algaeSubsystem)
    );

    // Algae Eject
    new POVButton(m_driverController.getHID(), ControllerConstants.k_dpadDown)
      .whileTrue(
        new InstantCommand(() -> m_algaeSubsystem.shoot(), m_algaeSubsystem)
      )
      .onFalse(
        new InstantCommand(() -> m_algaeSubsystem.stopShooter(), m_algaeSubsystem)
      );
      */
  }
  
  // Commands!
  // Command that takes Xbox Controller Inputs and allows robot to drive
  // NOTE: getLeftY and getLeftX are opposite for a reason!!! It is correct!!
  public Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveCommand(
      () -> MathUtil.applyDeadband(m_driverController.getLeftY() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
      () -> MathUtil.applyDeadband(m_driverController.getLeftX() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
      () -> m_driverController.getRightX() * DriveConstants.k_turnRate);

  // Use this to pass the autonomous command to Robot.java
  // Returns the command to run in autonomous
  public Command getAutonomousCommand() {

    // The selected auto on SmartDashboard will be run in autonomous
    return m_chooser.getSelected(); 
  }

  public Command elevatorLimelightCommand = m_elevatorSubsystem.automaticRaiseCommand();

  // Command Chain for Raising Wrist
  SequentialCommandGroup RaiseWristCommand = new SequentialCommandGroup(
    new ZeroWristCommand(m_wristSubsystem),
    new SetWristCommand(m_wristSubsystem, "intake")
  );

  // Command Chain for Lowering Wrist
  SequentialCommandGroup LowerWristCommand = new SequentialCommandGroup(
    new SetWristCommand(m_wristSubsystem, "score"),
    new ZeroWristCommand(m_wristSubsystem)
  );

  // Command Chain for Manual Reset
  SequentialCommandGroup ResetElevatorCommand = new SequentialCommandGroup(
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ZeroElevatorCommand(m_elevatorSubsystem)
  );
  
  // Command Chain for Intake Auto
  SequentialCommandGroup IntakeAutoCommand = new SequentialCommandGroup(
    new SetWristCommand(m_wristSubsystem, "intake"), 
    new RunIntakeForSecsCommand(m_intakeSubsystem, 1.0)
  );

  // Command Chain for Shoot Auto
  SequentialCommandGroup ShootAutoCommand = new SequentialCommandGroup(
    new RunShootForSecsCommand(m_intakeSubsystem, 0.5, true)
  );

  // Command Chain for Removing Algae
  SequentialCommandGroup RemoveAlgaeCommand = new SequentialCommandGroup(
    new SetWristCommand(m_wristSubsystem, "score"),
    new ZeroWristCommand(m_wristSubsystem),
    new AimNRangeAlgaeRemovalCommand(m_swerveSubsystem, "position"),
    new ParallelCommandGroup(
      new SetElevatorCommand(m_elevatorSubsystem, "algae"),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 1)
    ),
    new ZeroWristCommand(m_wristSubsystem),
    new SetWristCommand(m_wristSubsystem, "intake"),
    new AimNRangeAlgaeRemovalCommand(m_swerveSubsystem, "remove"),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ZeroElevatorCommand(m_elevatorSubsystem)
  );

  // Command Chain for Finishing AimNRange
  SequentialCommandGroup AimNRangeFinishCommand = new SequentialCommandGroup(
    new ZeroWristCommand(m_wristSubsystem),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ParallelCommandGroup(
      new ZeroElevatorCommand(m_elevatorSubsystem),
      new SetWristCommand(m_wristSubsystem, "intake")
    ),
    new LEDColorChangeCommand(m_ledSubsystem, "Alliance")
  );

  // Command Chain for Raising to L2
  SequentialCommandGroup RaiseL2Command = new SequentialCommandGroup(
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new SetElevatorCommand(m_elevatorSubsystem, "L2"),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        ) 
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    )
  );

  // Command Chain for Raising to L3
  SequentialCommandGroup RaiseL3Command = new SequentialCommandGroup(
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new SetElevatorCommand(m_elevatorSubsystem, "L3"),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        ) 
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    )
  );

  // Command Chain for Raising to L4
  SequentialCommandGroup RaiseL4Command = new SequentialCommandGroup(
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new SetElevatorCommand(m_elevatorSubsystem, "L4"),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        ) 
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    )
  );

  // Command Chain for Completely Automated L2
  SequentialCommandGroup AimNRangescoreL2Command = new SequentialCommandGroup(
    new LEDColorChangeCommand(m_ledSubsystem, "Scoring"),
    new ZeroElevatorCommand(m_elevatorSubsystem),
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new AimNRangeCommand(m_swerveSubsystem),
        new SetElevatorCommand(m_elevatorSubsystem, "L2"),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        ) 
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    ),
    new RunShootForSecsCommand(m_intakeSubsystem, 0.7, VisionConstants.k_positioned),
    new ZeroWristCommand(m_wristSubsystem),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ParallelCommandGroup(
      new ZeroElevatorCommand(m_elevatorSubsystem),
      new SetWristCommand(m_wristSubsystem, "intake")
    ),
    new LEDColorChangeCommand(m_ledSubsystem, "Alliance")
  );

  // Command Chain for Completely Automated L3
  SequentialCommandGroup AimNRangescoreL3Command = new SequentialCommandGroup(
    new LEDColorChangeCommand(m_ledSubsystem, "Scoring"),
    new ZeroElevatorCommand(m_elevatorSubsystem),
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new AimNRangeCommand(m_swerveSubsystem),
        new SetElevatorCommand(m_elevatorSubsystem, "L3"),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        ) 
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    ),
    new RunShootForSecsCommand(m_intakeSubsystem, 0.7, VisionConstants.k_positioned),
    new ZeroWristCommand(m_wristSubsystem),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ParallelCommandGroup(
      new ZeroElevatorCommand(m_elevatorSubsystem),
      new SetWristCommand(m_wristSubsystem, "intake")
    ),
    new LEDColorChangeCommand(m_ledSubsystem, "Alliance")
  );

  // Command Chain for Completely Automated L4
  SequentialCommandGroup AimNRangescoreL4Command = new SequentialCommandGroup(
    new LEDColorChangeCommand(m_ledSubsystem, "Scoring"),
    new ZeroElevatorCommand(m_elevatorSubsystem),
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new AimNRangeCommand(m_swerveSubsystem),
        new SetElevatorCommand(m_elevatorSubsystem, "L4"),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        ) 
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    ),
    new RunShootForSecsCommand(m_intakeSubsystem, 0.7, VisionConstants.k_positioned),
    new ZeroWristCommand(m_wristSubsystem),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ParallelCommandGroup(
      new ZeroElevatorCommand(m_elevatorSubsystem),
      new SetWristCommand(m_wristSubsystem, "intake")
    ),
    new LEDColorChangeCommand(m_ledSubsystem, "Alliance")
  );

  // Command Chain for Early Elevator Raise in Auto
  SequentialCommandGroup Akatsuki = new SequentialCommandGroup(
    new LEDColorChangeCommand(m_ledSubsystem, "Scoring"),
    new ZeroElevatorCommand(m_elevatorSubsystem),
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new SetElevatorCommand(m_elevatorSubsystem, "L4"),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        ) 
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    )
  );

  // Command Chain for Scoring Preparation no Elevator
  SequentialCommandGroup PrepareToScore = new SequentialCommandGroup(
    new LEDColorChangeCommand(m_ledSubsystem, "Scoring"),
    new ZeroElevatorCommand(m_elevatorSubsystem),
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new SetElevatorCommand(m_elevatorSubsystem, "L4"),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        ) 
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    )
  );

  // Command Chain for Elevator Preset L4 - RIGHT REEF
  SequentialCommandGroup AimNRangeRightNoElevatorCommand = new SequentialCommandGroup(
    new AimNRangeAutoCommand(m_swerveSubsystem, true), 
    new RunShootForSecsCommand(m_intakeSubsystem, 0.5, VisionConstants.k_positioned)
  );

  // Command Chain for Elevator Preset L4 - LEFT REEF
  SequentialCommandGroup AimNRangeLeftNoElevatorCommand = new SequentialCommandGroup(
    new AimNRangeAutoCommand(m_swerveSubsystem, false), 
    new RunShootForSecsCommand(m_intakeSubsystem, 0.5, VisionConstants.k_positioned)
  );

  // Command Chain for Completely Automated L4 in Auto - RIGHT REEF
  SequentialCommandGroup AimNRangeScoreAutoRightCommand = new SequentialCommandGroup(
    new LEDColorChangeCommand(m_ledSubsystem, "Scoring"),
    new ZeroElevatorCommand(m_elevatorSubsystem),
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new AimNRangeAutoCommand(m_swerveSubsystem, true),
        new SetElevatorCommand(m_elevatorSubsystem, "L4"),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        ) 
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    ),
    new RunShootForSecsCommand(m_intakeSubsystem, 0.5, VisionConstants.k_positioned)
    /*
    new ZeroWristCommand(m_wristSubsystem),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ZeroElevatorCommand(m_elevatorSubsystem),
    new InstantCommand(() -> m_ledSubsystem.setAllianceColor(), m_ledSubsystem)
    */
  );

  // Command Chain for Completely Automated L4 in Auto - LEFT REEF
  SequentialCommandGroup AimNRangeScoreAutoLeftCommand = new SequentialCommandGroup(
    new LEDColorChangeCommand(m_ledSubsystem, "Scoring"),
    new ZeroElevatorCommand(m_elevatorSubsystem),
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new AimNRangeAutoCommand(m_swerveSubsystem, false),
        new SetElevatorCommand(m_elevatorSubsystem, "L4"),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        ) 
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    ),
    new RunShootForSecsCommand(m_intakeSubsystem, 0.5, VisionConstants.k_positioned)
    /*
    new ZeroWristCommand(m_wristSubsystem),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ZeroElevatorCommand(m_elevatorSubsystem),
    new InstantCommand(() -> m_ledSubsystem.setAllianceColor(), m_ledSubsystem)
    */
  );

  // Command Chain for Completely Automated Scoring with Automated Elevator
  SequentialCommandGroup AimNRangescoreAllCommand = new SequentialCommandGroup(
    new LEDColorChangeCommand(m_ledSubsystem, "Scoring"),
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new AimNRangeCommand(m_swerveSubsystem),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        )
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    ),
    new RunShootForSecsCommand(m_intakeSubsystem, 0.5, VisionConstants.k_positioned),
    new ZeroWristCommand(m_wristSubsystem),
    new SetWristCommand(m_wristSubsystem, "intake"),
    new LEDColorChangeCommand(m_ledSubsystem, "Alliance"), 
    new SetLevelCommand("L1")
  );

  // Command Chain for Completely Automated Scoring with Automated Elevator
  SequentialCommandGroup AimNRangescoreAllAutoRightCommand = new SequentialCommandGroup(
    new SetLevelCommand("L4"),
    new LEDColorChangeCommand(m_ledSubsystem, "Scoring"),
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new AimNRangeAutoCommand(m_swerveSubsystem, true),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        )
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    ),
    new RunShootForSecsCommand(m_intakeSubsystem, 0.5, VisionConstants.k_positioned),
    new ZeroWristCommand(m_wristSubsystem),
    new SetWristCommand(m_wristSubsystem, "intake"),
    new LEDColorChangeCommand(m_ledSubsystem, "Alliance"), 
    new SetLevelCommand("L1")
  );

  // Command Chain for Completely Automated Scoring with Automated Elevator
  SequentialCommandGroup AimNRangescoreAllAutoLeftCommand = new SequentialCommandGroup(
    new SetLevelCommand("L4"),
    new LEDColorChangeCommand(m_ledSubsystem, "Scoring"),
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new AimNRangeAutoCommand(m_swerveSubsystem, false),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        )
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    ),
    new RunShootForSecsCommand(m_intakeSubsystem, 0.7, VisionConstants.k_positioned),
    new ZeroWristCommand(m_wristSubsystem),
    new SetWristCommand(m_wristSubsystem, "intake"),
    new LEDColorChangeCommand(m_ledSubsystem, "Alliance"), 
    new SetLevelCommand("L1")
  );

  // Command Chain for Positioning and Loading at the Coral Station in Auto
  ParallelDeadlineGroup positionNIntakeAutoCoralStationLeft = new ParallelDeadlineGroup(
    new AimNRangeAutoCoralStationCommand(m_swerveSubsystem, false),
    new RunIntakeForSecsCommand(m_intakeSubsystem, 3)
  );

  // Command Chain for Positioning and Loading at the Coral Station in Auto
  ParallelDeadlineGroup positionNIntakeAutoCoralStationRight = new ParallelDeadlineGroup(
    new AimNRangeAutoCoralStationCommand(m_swerveSubsystem, true),
    new RunIntakeForSecsCommand(m_intakeSubsystem, 3)
  );
}
