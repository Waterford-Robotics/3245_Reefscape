// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SetElevatorCommand;
import frc.robot.commands.SetReefCommand;
import frc.robot.commands.SetWristCommand;
import frc.robot.commands.ZeroElevatorCommand;
import frc.robot.commands.ZeroWristCommand;
import frc.robot.commands.RunIntakeForSecsCommand;
import frc.robot.commands.RunShootForSecsCommand;
import frc.robot.commands.AimNRangeAlgaeRemovalCommand;
import frc.robot.commands.AimNRangeAutoCommand;
import frc.robot.commands.AimNRangeCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

  // Create New Choosing Option in SmartDashboard for Autos
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    // Makes the drive command the default command (good!)
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

    // Named Command Configuration
    NamedCommands.registerCommand("Score L4", AimNRangescoreL4Command);
    NamedCommands.registerCommand("Score Left Reef", AimNRangeScoreAutoLeftCommand);
    NamedCommands.registerCommand("Score Right Reef", AimNRangeScoreAutoRightCommand);
    NamedCommands.registerCommand("Raise Wrist", RaiseWristCommand);
    NamedCommands.registerCommand("Intake", IntakeAutoCommand);

    /* 
     * KEY:
     * U - UNCONFIRMED
     * P - Preferred
    */

    // Autos!

    // 0C - Leave
    m_chooser.addOption("0C: Center Leave", m_swerveSubsystem.getAutonomousCommand("CenterLeave")); // U
    m_chooser.addOption("0C: Left Leave", m_swerveSubsystem.getAutonomousCommand("LeftLeave")); // U
    m_chooser.addOption("0C: Right Leave", m_swerveSubsystem.getAutonomousCommand("RightLeave")); // U

    // 1C - Back Center
    m_chooser.addOption("1C: BC-L", m_swerveSubsystem.getAutonomousCommand("1C BC-L")); // U
    m_chooser.addOption("1C: BC-R", m_swerveSubsystem.getAutonomousCommand("1C BC-R")); // U

    // 1C - Back Left
    m_chooser.addOption("1C: BL-L", m_swerveSubsystem.getAutonomousCommand("1C BL-L")); // U
    m_chooser.addOption("1C: BL-R", m_swerveSubsystem.getAutonomousCommand("1C BL-R")); // U

    // 1C - Back Right
    m_chooser.addOption("1C: BR-L", m_swerveSubsystem.getAutonomousCommand("1C BR-L")); // U
    m_chooser.addOption("1C: BR-R", m_swerveSubsystem.getAutonomousCommand("1C BR-R")); // U

    // 1C - Front Left
    m_chooser.addOption("1C: FL-L", m_swerveSubsystem.getAutonomousCommand("1C FL-L")); // U
    m_chooser.addOption("1C: FL-R", m_swerveSubsystem.getAutonomousCommand("1C FL-R")); // U

    // 1C - Front Right
    m_chooser.addOption("1C: FR-L", m_swerveSubsystem.getAutonomousCommand("1C FR-L")); // U
    m_chooser.addOption("1C: FR-R", m_swerveSubsystem.getAutonomousCommand("1C FR-R")); // U

    // 1.5C - Back Left
    m_chooser.addOption("1.5C: BL-L", m_swerveSubsystem.getAutonomousCommand("1.5C BL-L")); // U
    m_chooser.addOption("1.5C: BL-R", m_swerveSubsystem.getAutonomousCommand("1.5C BL-R")); // U

    // 1.5C - Back Right
    m_chooser.addOption("1.5C: BR-L", m_swerveSubsystem.getAutonomousCommand("1.5C BR-L")); // U
    m_chooser.addOption("1.5C: BR-R", m_swerveSubsystem.getAutonomousCommand("1.5C BR-R")); // U

    // 2C - Back Left -> Front Left
    m_chooser.addOption("2C: BL-L, FL-L", m_swerveSubsystem.getAutonomousCommand("2C BL-L, FL-L")); // U
    m_chooser.addOption("2C: BL-L, FL-R", m_swerveSubsystem.getAutonomousCommand("2C BL-L, FL-R")); // U

    m_chooser.addOption("2C: BL-R, FL-L", m_swerveSubsystem.getAutonomousCommand("2C BL-R, FL-L")); // U
    m_chooser.addOption("2C: BL-R, FL-R *", m_swerveSubsystem.getAutonomousCommand("2C BL-R, FL-R")); // U (P)

    // 2C - Back Right -> Front Right
    m_chooser.addOption("2C: BR-L, FR-L *", m_swerveSubsystem.getAutonomousCommand("2C BR-L, FR-L")); // U (P)
    m_chooser.addOption("2C: BR-L, FR-R", m_swerveSubsystem.getAutonomousCommand("2C BR-L, FR-R")); // U

    m_chooser.addOption("2C: BR-R, FR-L", m_swerveSubsystem.getAutonomousCommand("2C BR-R, FR-L")); // U
    m_chooser.addOption("2C: BR-R, FR-R", m_swerveSubsystem.getAutonomousCommand("2C BR-R, FR-R")); // U 

    // 2.5C - Back Left -> Front Left
    m_chooser.addOption("2.5C: BL-R, FL-R *", m_swerveSubsystem.getAutonomousCommand("2.5C BL-R, FL-R")); // U (P)

    // 2.5C - Back Right -> Front Right
    m_chooser.addOption("2.5C: BR-L, FR-L *", m_swerveSubsystem.getAutonomousCommand("2.5C BR-L, FR-L")); // U (P)

    // Puts a chooser on the SmartDashboard!
    SmartDashboard.putData("AutoMode", m_chooser);
  }

  // Trigger & Button Bindings!
  private void configureBindings() {
    
    // Lower Elevator Manually - "A" Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_A)
      .onTrue(
        ResetElevatorCommand
      );
    
    // Score Coral on L2 and Zero Elevator - "B" Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_B)
      .onTrue(
        AimNRangescoreL2Command
      );

    // Score Coral on L3 and Zero Elevator - "X" Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_X)
      .onTrue(
        AimNRangescoreL3Command
      );

    // Score Coral on L4 and Zero Elevator - "Y" Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_Y)
      .onTrue(
        AimNRangescoreL4Command
      );
    
    // Strafe Right - Right Bump
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_rightbump)
      .whileTrue(
        new RunCommand(() -> m_swerveSubsystem.driveCommandLimelight(0, 0.15, 0))
      )
      .onFalse(
        new InstantCommand(() -> m_swerveSubsystem.driveCommandLimelight(0, 0, 0))
      );

    // Strafe Left - Left Bump
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_leftbump)
      .onTrue(
        LowerWristCommand
      );

    // Intake Manually - Right Trig
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
      .whileTrue(
        new InstantCommand(() -> m_intakeSubsystem.intake(), m_wristSubsystem)
      )
      .whileFalse(
        new InstantCommand(() -> m_intakeSubsystem.stopShooter(), m_wristSubsystem)
      );

    // Raise Wrist - Left Trig
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_lefttrig) > 0.05)
      .onTrue(
        RaiseWristCommand
      );

      /*
      .whileTrue(
        new InstantCommand(() -> m_wristSubsystem.shoot(), m_wristSubsystem)
      )
      .whileFalse(
        new InstantCommand(() -> m_wristSubsystem.stopShooter(), m_wristSubsystem)
      );
      */

    // Reset Gyro
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_start)
      .onTrue(
        new InstantCommand(() -> m_swerveSubsystem.zeroGyro(), m_swerveSubsystem)
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

    // Remove Algae
    new POVButton(m_driverController.getHID(), ControllerConstants.k_dpadup)
      .onTrue(
        RemoveAlgaeCommand
      );

    // Alg Intake
    /*
    new POVButton(m_driverController.getHID(), ControllerConstants.k_dpadDown)
      .whileTrue(
        new InstantCommand(() -> m_algaeSubsystem.intake())
      )
      .onFalse(
        new InstantCommand(() -> m_algaeSubsystem.stopShooter())
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
    new ParallelDeadlineGroup(
      new RunIntakeForSecsCommand(m_intakeSubsystem, 1.0),
      new RunCommand(() -> m_swerveSubsystem.driveCommandLimelight(0, 0.15, 0))
    )
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

  // Command Chain for Scoring on L2
  SequentialCommandGroup ScoreL2Command = new SequentialCommandGroup(
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new SetElevatorCommand(m_elevatorSubsystem, "L2"),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        ) 
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    ),
    new RunShootForSecsCommand(m_intakeSubsystem, 1.0, true),
    new ZeroWristCommand(m_wristSubsystem),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ZeroElevatorCommand(m_elevatorSubsystem)
  );

  // Command Chain for Scoring on L3
  SequentialCommandGroup ScoreL3Command = new SequentialCommandGroup(
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new SetElevatorCommand(m_elevatorSubsystem, "L3"),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        ) 
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    ),
    new RunShootForSecsCommand(m_intakeSubsystem, 1.0, true),
    new ZeroWristCommand(m_wristSubsystem),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ZeroElevatorCommand(m_elevatorSubsystem)
  );

  // Command Chain for Scoring on L4
  SequentialCommandGroup ScoreL4Command = new SequentialCommandGroup(
    new ParallelDeadlineGroup(
      new ParallelCommandGroup(
        new SetElevatorCommand(m_elevatorSubsystem, "L4"),
        new SequentialCommandGroup(
          new SetWristCommand(m_wristSubsystem, "score"),
          new ZeroWristCommand(m_wristSubsystem)
        ) 
      ),
      new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
    ),
    new RunShootForSecsCommand(m_intakeSubsystem, 1.0, true),
    new ZeroWristCommand(m_wristSubsystem),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ZeroElevatorCommand(m_elevatorSubsystem)
  );

  // Command Chain for Completely Automated L2
  SequentialCommandGroup AimNRangescoreL2Command = new SequentialCommandGroup(
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
    new ZeroElevatorCommand(m_elevatorSubsystem)
  );

  // Command Chain for Completely Automated L3
  SequentialCommandGroup AimNRangescoreL3Command = new SequentialCommandGroup(
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
    new ZeroElevatorCommand(m_elevatorSubsystem)
  );

  // Command Chain for Completely Automated L4
  SequentialCommandGroup AimNRangescoreL4Command = new SequentialCommandGroup(
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
    new ZeroElevatorCommand(m_elevatorSubsystem)
  );

  // Command Chain for Completely Automated L4 in Auto - RIGHT REEF
  SequentialCommandGroup AimNRangeScoreAutoRightCommand = new SequentialCommandGroup(
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
    new RunShootForSecsCommand(m_intakeSubsystem, 0.7, VisionConstants.k_positioned),
    new ZeroWristCommand(m_wristSubsystem),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ZeroElevatorCommand(m_elevatorSubsystem)
  );

  // Command Chain for Completely Automated L4 in Auto - LEFT REEF
  SequentialCommandGroup AimNRangeScoreAutoLeftCommand = new SequentialCommandGroup(
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
    new RunShootForSecsCommand(m_intakeSubsystem, 0.7, VisionConstants.k_positioned),
    new ZeroWristCommand(m_wristSubsystem),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ZeroElevatorCommand(m_elevatorSubsystem)
  );
}
