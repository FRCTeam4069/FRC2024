// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FeedIntakeCommand;
import frc.robot.commands.FieldCentricDrive;
import frc.robot.commands.ShooterCommand;
import frc.robot.constants.CameraConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight.CameraController;

import java.io.File;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public AprilTagFieldLayout aprilTagFieldLayout;
  //public CameraController fCam = new CameraController(CameraConstants.fCamName);
  //public CameraController bCam = new CameraController(CameraConstants.bCamName);

  //public static final ShooterController shooter = new ShooterController();
  //public static final IndexerController indexer = new IndexerController();
  //public static final IntakeController intake = new IntakeController();

  public static final ShooterController shooter = null;
  public static final IndexerController indexer = null;
  public static final IntakeController intake = null;
  
  public SwerveSubsystem drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  //private final SwerveSubsystem drive = SwerveSubsystem.getInstance();
  
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController Controller1 =
      new CommandXboxController(0);
  private final CommandXboxController Controller2 = 
      new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    drive.setDefaultCommand(new FieldCentricDrive(
      drive, 
      () -> Controller1.getLeftY(),
      () -> Controller1.getLeftX(),
      () -> Controller1.getRightX()));
    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(Controller2.y()).whileTrue(new ShooterCommand(fCam.getDistanceToTarget(), fCam.getYaw()));
    //new Trigger(Controller2.pov(0)).onTrue(new);

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //Controller1.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    //new Trigger(Controller2.rightBumper()).whileTrue(new FeedIntakeCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
