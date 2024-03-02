// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BackIntakeCommand;
import frc.robot.commands.BringIntakeUpCommand;
import frc.robot.commands.DefualtIndexerCommand;
import frc.robot.commands.FeedIntakeCommand;
import frc.robot.commands.SetShooterCommand;
import frc.robot.commands.SetShooterRotation;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterPositions;
import frc.robot.commands.ShooterRotationCommand;
import frc.robot.commands.defaultArtCommand;
import frc.robot.commands.unIndexCOmmand;
import frc.robot.constants.CameraConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.IntakeController.positions;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.Limelight.CameraController;
import frc.robot.subsystems.Limelight.CameraIsAsCameraDoes;
import frc.robot.subsystems.ShooterRotationController.shooterAngles;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.constants.CameraConstants;
import frc.robot.subsystems.Limelight.CameraHelper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

  //t public static CameraController cam = new CameraController("frontCamera","http://10.40.69.11:5800", "photonvision");
  public static final ShooterController shooter = new ShooterController();
  
  // public static final CameraHelper frontCamera = new CameraHelper(CameraConstants.fCamName, CameraConstants.aprilTagFieldLayout, CameraConstants.robotToFrontCam);
  public static final CameraIsAsCameraDoes FrontCamera = new CameraIsAsCameraDoes("limelight-front");

  public final PowerDistribution powerDistributionHub = new PowerDistribution();

  public static final IndexerController indexer = new IndexerController();
  public static final IntakeController intake = new IntakeController();

  public static final ClimberSubsystem climber = new ClimberSubsystem();

  public static final ShooterRotationController artShooter = new ShooterRotationController();
  
  //public SwerveSubsystem drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  public SwerveDrivetrain drive = new SwerveDrivetrain();

  public final SendableChooser<Command> autoChooser;
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController Controller1 =
      new CommandXboxController(0);
  private final CommandXboxController Controller2 = 
      new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(drive.driveCommand(
      () -> Controller1.getLeftY(), 
      () -> Controller1.getLeftX(), 
      () -> Controller1.getRightX()));
    
    //drive.setDefaultCommand(drive.angleModulesCommand(() -> Controller1.getLeftY(), () -> Controller1.getLeftX()));
    Controller1.a().onTrue(new InstantCommand(() -> drive.resetGyro()));
    
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    //Controller1.a().onTrue(drive.sysIdSteerTest());
    //Controller1.b().onTrue(drive.sysIdDriveTestDynamic());

    intake.setDefaultCommand(new BringIntakeUpCommand(intake));
    artShooter.setDefaultCommand(new ShooterRotationCommand(artShooter));
    //artShooter.setDefaultCommand(new ShooterRotationCommand(artShooter));
    //intake.setDefaultCommand(new defaultArtCommand());
    
    // Configure the trigger bindings

    //  indexer.setDefaultCommand(new DefualtIndexerCommand(
    //    () -> Controller2.leftBumper().getAsBoolean()
    //  ));

    //configureBindings();
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
    Controller2.y().whileTrue(new SetShooterRotation(artShooter, Math.hypot(FrontCamera.getXDistanceToApriltag(new int[]{8, 7}), FrontCamera.getYDistanceToApriltag(7)), shooter));
    Controller2.x().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.WALL_AREA));
    Controller2.a().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.WALL_AREA));
    Controller2.b().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.SAFE_ZONE));
    Controller2.leftStick().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.AMP_AREA));

    //new Trigger(Controller2.rightBumper()).whileTrue(new FeedIntakeCommand());
    //new Trigger(Controller2.leftBumper()).whileTrue(new BackIntakeCommand(intake));
    Controller2.leftBumper().whileTrue(new unIndexCOmmand(indexer));
    
    Controller2.rightBumper().whileTrue(new DefualtIndexerCommand(() -> shooter.isShooting()));                                                       
   
    //new Trigger(Controller2.rightBumper()).whileTrue(intake.setPosition(positions.LOWER)).onFalse(intake.setPosition(positions.UPPER));
    
    Controller2.rightBumper().whileTrue(intake.setPosition(positions.LOWER)).whileFalse(intake.setPosition(positions.UPPER));
    Controller2.leftBumper().whileTrue(new BackIntakeCommand(intake));
        
  }
    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(Controller2.y()).whileTrue(new ShooterCommand(fCam.getDistanceToTarget(), fCam.getYaw()));
    //new Trigger(Controller2.pov(0)).onTrue(new);

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //Controller1.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    //new Trigger(Controller2.rightBumper()).whileTrue(new FeedIntakeCommand());


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return autoChooser.getSelected();
    //return new PathPlannerAuto("Example Auto");
  }
}
