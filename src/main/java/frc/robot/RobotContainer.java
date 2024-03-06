// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BackIntakeCommand;

import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DefualtIndexerCommand;
import frc.robot.commands.DefualtShooter;
import frc.robot.commands.FeedIntakeCommand;
import frc.robot.commands.FieldCentricDrive;
import frc.robot.commands.REverseIndexerCommand;
import frc.robot.commands.SetShooterCommand;
import frc.robot.commands.SetShooterRotation;

import frc.robot.commands.ShooterPositions;
import frc.robot.commands.ShooterRotationCommand;
import frc.robot.commands.defaultArtCommand;
import frc.robot.commands.unIndexCOmmand;
import frc.robot.commands.drivebase.Rotate;
import frc.robot.commands.drivebase.testAuto;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.LEDController;
//import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.IntakeController.positions;
import frc.robot.subsystems.LEDController.Colours;
//import frc.robot.subsystems.LEDController.Colours;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.Limelight.CameraIsAsCameraDoes;
import frc.robot.subsystems.Limelight.PoseEstimatorSubsystem;

import frc.robot.subsystems.swerve.SwerveDrivetrain;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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

  //public static final LEDController led = new LEDController();
  
  // public static final CameraHelper frontCamera = new CameraHelper(CameraConstants.fCamName, CameraConstants.aprilTagFieldLayout, CameraConstants.robotToFrontCam);
  public static final CameraIsAsCameraDoes  FrontCamera = new CameraIsAsCameraDoes("limelight-front");
  // public static final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(null, null);

  public static final ShuffleboardTab autoTab = Shuffleboard.getTab("auto");


  public static LEDController led = new LEDController();

  public final PowerDistribution powerDistributionHub = new PowerDistribution();

  public static final IndexerController indexer = new IndexerController();
  public static final IntakeController intake = new IntakeController();

  //public static final ClimberSubsystem climber = new ClimberSubsystem();

  public static final ShooterRotationController artShooter = new ShooterRotationController();
  
  //public SwerveSubsystem drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  public static final SwerveDrivetrain drive = new SwerveDrivetrain();

  // public static final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(drive.getGyroscropeRotation(), drive.getModulePositions());
  public static final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(drive::getGyroscropeRotation, drive::getModulePositions);

  //public final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(drive::getRotation2d, drive::getModulePositions);

  public final SendableChooser<Command> autoChooser;
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController Controller1 =
      new CommandXboxController(0);
  public static final CommandXboxController Controller2 = 
      new CommandXboxController(1);
  private final CommandXboxController Controller3 = 
      new CommandXboxController(2);

  private ClimberSubsystem climber = new ClimberSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // var camX = FrontCamera.getXDistanceToApriltag(7, 4);
    // var camY = FrontCamera.getYDistanceToApriltag(4, 7);
    // var angle = Math.atan2(camY, camX);
    drive.setDefaultCommand(new FieldCentricDrive(
      drive,
      () -> Controller1.getLeftY(), 
      () -> Controller1.getLeftX(), 
      () -> Controller1.getRightX(),
      () -> Controller1.getHID().getRightBumper(),
      () -> Controller1.getHID().getYButton(),
      () -> FrontCamera.getTX(7, 4)));
    
    //drive.setDefaultCommand(drive.angleModulesCommand(() -> Controller1.getLeftY(), () -> Controller1.getLeftX()));
    Controller1.a().onTrue(new InstantCommand(() -> drive.resetGyro()));
    Controller1.b().onTrue(new InstantCommand(() -> drive.resetPose()));
    Controller1.x().whileTrue(new Rotate(drive, Units.degreesToRadians(-8.0)));
    
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    //Controller1.a().onTrue(drive.sysIdSteerTest());
    //Controller1.b().onTrue(drive.sysIdDriveTestDynamic());

    //intake.setDefaultCommand(new BringIntakeUpCommand(intake));
    artShooter.setDefaultCommand(new ShooterRotationCommand(artShooter));
    intake.setDefaultCommand(new defaultArtCommand());
    climber.setDefaultCommand(new ClimberCommand(climber, () -> Controller2.getLeftY()));
    
    //led.setDefaultCommand(led.HoldSetColour());
    led.setDefaultCommand(led.setColour(Colours.STARTUPPATTERN));

    // Configure the trigger bindings

    //  indexer.setDefaultCommand(new DefualtIndexerCommand(
    //    () -> Controller2.leftBumper().getAsBoolean()
    //  ));
  
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
    Controller2.y().whileTrue(new SetShooterRotation(artShooter, Math.hypot(FrontCamera.getXDistanceToApriltag(7, 4), 
                                                                            FrontCamera.getYDistanceToApriltag(4, 7)), 
                                                                            shooter)).onTrue(intake.setPosition(positions.UPPER));

    Controller2.x().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.SAFE_ZONE)).onTrue(intake.setPosition(positions.UPPER));
    Controller2.a().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.WALL_AREA)).onTrue(intake.setPosition(positions.UPPER));
    Controller2.b().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.SAFE_ZONE)).onTrue(intake.setPosition(positions.UPPER));
    Controller2.leftStick().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.WHITE_LINE)).onTrue(intake.setPosition(positions.UPPER));

    // new Trigger(Controller2.rightBumper()).whileTrue(new FeedIntakeCommand());
    // new Trigger(Controller2.leftBumper()).whileTrue(new BackIntakeCommand(intake));
    Controller2.leftBumper().whileTrue(new unIndexCOmmand(indexer));
    Controller2.rightBumper().whileTrue(new DefualtIndexerCommand(() -> shooter.isShooting(), () -> Controller2.getRightTriggerAxis(), () -> Controller2.getRightTriggerAxis()));                                                       
   
    //new Trigger(Controller2.rightBumper()).whileTrue(intake.setPosition(positions.LOWER)).onFalse(intake.setPosition(positions.UPPER));
    
    //Controller2.rightBumper().onTrue(intake.setPosition(positions.LOWER));
    //Controller2.rightBumper().whileTrue(new RunCommand(() -> intake.driveFeed())).whileFalse(new InstantCommand(() -> intake.stopFeed()));
    Controller2.leftBumper().whileTrue(new BackIntakeCommand(intake));
    //Controller2.start().whileTrue(new ClimberCommand(climber, () -> Controller2.getLeftY(), artShooter));
    Controller2.start().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.CLIMB)).onTrue(artShooter.changeClimbStatus());
    Controller2.start().onTrue(led.setColour(Colours.RED));
    // new Trigger(Controller2.pov(0).onTrue(new InstantCommand( () -> intake.setPosition(positions.UPPER))));
    // new Trigger(Controller2.pov(180).onTrue(new InstantCommand(() -> intake.setPosition(positions.LOWER))));
    
    new Trigger(Controller2.pov(0).onTrue(intake.setPosition(positions.UPPER)));
    new Trigger(Controller2.pov(180).onTrue(intake.setPosition(positions.LOWER)));
    
    Controller2.rightBumper().whileTrue(new FeedIntakeCommand());

    new Trigger(Controller2.rightTrigger(0.5)).whileTrue(new DefualtShooter(indexer, () -> shooter.isShooting(), Controller2.rightTrigger()));

    Controller2.leftTrigger(0.5).whileTrue(new REverseIndexerCommand(indexer, () -> indexer.pastSensor(), () -> indexer.getPhotoReading()));
    new Trigger(() -> indexer.getPhotoReading()).whileTrue(led.setColour(Colours.GREEN));
    new Trigger(() -> shooter.atSpeed()).whileTrue(led.setColour(Colours.ERROR_YELLOw));

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
    //return autoChooser.getSelected();
    return new testAuto(drive, intake);
    //return new PathPlannerAuto("Example Auto");
  }
}
