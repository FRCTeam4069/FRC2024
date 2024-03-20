// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.SerialPort.Parity;
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
import frc.robot.commands.IndexShooter;
import frc.robot.commands.REverseIndexerCommand;
import frc.robot.commands.SetShooterCommand;
import frc.robot.commands.SetShooterRotation;

import frc.robot.commands.ShooterPositions;
import frc.robot.commands.ShooterRotationCommand;
import frc.robot.commands.defaultArtCommand;
import frc.robot.commands.unIndexCOmmand;
import frc.robot.commands.drivebase.Auto2056;
import frc.robot.commands.drivebase.Four;
import frc.robot.commands.drivebase.FrontAuto;
import frc.robot.commands.drivebase.OneNote;
import frc.robot.commands.drivebase.Rotate;
import frc.robot.commands.drivebase.SideAuto;
import frc.robot.commands.drivebase.StrafeUntilCam;
import frc.robot.commands.drivebase.Toggle;
import frc.robot.commands.drivebase.TwoNote;
import frc.robot.commands.drivebase.TwoNoteNew;
import frc.robot.commands.drivebase.TwoNoteNewBlue;
import frc.robot.commands.drivebase.testAuto;
import frc.robot.commands.drivebase.test.DriveTest;
import frc.robot.commands.drivebase.test.PoseTest;
import frc.robot.commands.drivebase.test.straightLineTest;
import frc.robot.commands.drivebase.test.testAutov2;
import frc.robot.commands.drivebase.test.testAutov3;
import frc.robot.commands.drivebase.test.testAutov4;
import frc.robot.commands.drivebase.test.testAutov5;
import frc.robot.constants.CameraConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.RevBlinkinPatterns;
//import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.IntakeController.positions;

//import frc.robot.subsystems.LEDController.Colours;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.Limelight.CameraIsAsCameraDoes;
import frc.robot.subsystems.Limelight.PhotonRunnable;
import frc.robot.subsystems.Limelight.PoseEstimatorSubsystem;

import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.traputil.MakePathOnTheFly;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public static final LEDController led = new LEDController();
  
  // public static final CameraHelper frontCamera = new CameraHelper(CameraConstants.fCamName, CameraConstants.aprilTagFieldLayout, CameraConstants.robotToFrontCam);
  public static final CameraIsAsCameraDoes  FrontCamera = new CameraIsAsCameraDoes("limelight-front");
  // public static final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(null, null);

  public final ShuffleboardTab autoTab = Shuffleboard.getTab("auto");

  public final PowerDistribution powerDistributionHub = new PowerDistribution();

  public static final IndexerController indexer = new IndexerController();
  public static final IntakeController intake = new IntakeController();

  //public static final ClimberSubsystem climber = new ClimberSubsystem();

  public static final ShooterRotationController artShooter = new ShooterRotationController();
  
  //public SwerveSubsystem drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  public static final SwerveDrivetrain drive = new SwerveDrivetrain();

  public final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(drive::getRotation2d, drive::getModulePositions, drive::addVisionMeasurement);

  public final SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController Controller1 =
      new CommandXboxController(0);
  public static final CommandXboxController Controller2 = 
      new CommandXboxController(1);
  private final CommandXboxController Controller3 = 
      new CommandXboxController(2);

  private ClimberSubsystem climber = new ClimberSubsystem();

  private Toggle toggle;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // var camX = FrontCamera.getXDistanceToApriltag(7, 4);
    // var camY = FrontCamera.getYDistanceToApriltag(4, 7);
    // var angle = Math.atan2(camY, camX);

    toggle = new Toggle(() -> Controller1.getHID().getStartButton(), () -> Controller1.getHID().getBackButton());
    drive.setDefaultCommand(new FieldCentricDrive(
      drive,
      () -> -Controller1.getLeftY(), 
      () -> -Controller1.getLeftX(), 
      () -> Controller1.getRightX(),
      () -> Controller1.getHID().getRightBumper(),
      () -> Controller1.getHID().getAButton(),
      () -> poseEstimator.getSpeakerTransform(),
      () -> Controller1.getHID().getLeftBumper(),
      () -> (Controller1.getHID().getPOV() == 90),
      () -> poseEstimator.getRotation2d()
    ));

    //drive.setDefaultCommand(drive.angleModulesCommand(() -> Controller1.getLeftY(), () -> Controller1.getLeftX()));
    Controller1.y().onTrue(new InstantCommand(() -> drive.resetGyro()));
    //Controller1.povUp().onTrue(new InstantCommand(() -> drive.resetPose()));
    Controller1.back().onTrue(new InstantCommand(() -> drive.setPose(new Pose2d(1.30, 5.55, Rotation2d.fromDegrees(0.0)))));
    // Controller1.povUp().onTrue(new InstantCommand(() -> drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)))));

    new Trigger(() -> poseEstimator.isAligned())
      .whileTrue(new InstantCommand(() -> Controller1.getHID().setRumble(RumbleType.kBothRumble, 0.1)))
      .onFalse(new InstantCommand(() -> Controller1.getHID().setRumble(RumbleType.kBothRumble, 0)));

    Controller1.x().whileTrue(new Rotate(drive, Units.degreesToRadians(-15.0)));
    Controller1.b().whileTrue(new Rotate(drive, Units.degreesToRadians(15.0)));
    // Controller1.rightBumper().whileTrue(new StrafeUntilCam(drive, () -> FrontCamera.getTX(7, 4), 1.0, () -> FrontCamera.hasTarget(7, 4)));
    // Controller1.leftBumper().whileTrue(new StrafeUntilCam(drive, () -> FrontCamera.getTX(7, 4), -1.0, () -> FrontCamera.hasTarget(7, 4)));
    //autoChooser = AutoBuilder.buildAutoChooser();

    // drive.setDefaultCommand(drive.angleModulesCommand(() -> Controller1.getLeftY(), () -> Controller1.getLeftX()));

    
    autoChooser.setDefaultOption("one", new testAuto(drive, intake, indexer, shooter, artShooter));
    autoChooser.addOption("onev2", new testAutov2(drive, intake, indexer, shooter, artShooter));
    //autoChooser.addOption("onev3", new testAutov3(drive, intake, indexer, shooter, artShooter));
    //autoChooser.addOption("onev4", new testAutov4(drive, intake, indexer, shooter, artShooter));
    //autoChooser.addOption("side auto", new SideAuto(drive, intake, indexer, shooter, artShooter));
    //autoChooser.addOption("front auto", new FrontAuto(drive, intake, indexer, shooter, artShooter));
    autoChooser.addOption("no move auto", new OneNote(drive, intake, indexer, shooter, artShooter));
    autoChooser.addOption("two ring on angle", new TwoNote(drive, intake, indexer, shooter, artShooter));
    //autoChooser.addOption("new pid auto", new testAutov5(drive, intake, indexer, shooter, artShooter));
    //autoChooser.addOption("new two ring on the side auto", new TwoNoteNew(drive, intake, indexer, shooter, artShooter));
    //autoChooser.addOption("BLUE new two ring on the side auto BLUE", new TwoNoteNewBlue(drive, intake, indexer, shooter, artShooter));
    autoChooser.addOption("test", new straightLineTest(drive, intake, indexer, shooter, artShooter));
    autoChooser.addOption("2056 auto", new Auto2056(drive, intake, indexer, shooter, artShooter));
    autoChooser.addOption("pose test", new PoseTest(drive, intake, indexer, shooter, artShooter));
    autoChooser.addOption("four ring", new Four(drive, intake, indexer, shooter, artShooter));
    autoChooser.addOption("drive test", new DriveTest(drive));
    

    SmartDashboard.putData("Auto Chooser", autoChooser);

    //drive.setDefaultCommand(drive.zeroModules());
    //Controller1.a().onTrue(drive.sysIdDriveTestQuasistatic());
    //Controller1.b().onTrue(drive.sysIdDriveTestDynamic());



    artShooter.setDefaultCommand(new ShooterRotationCommand(artShooter));
    intake.setDefaultCommand(new defaultArtCommand());
    //climber.setDefaultCommand(new Climb erCommand(climber, () -> Controller2.getLeftY()));

    //led.setDefaultCommand(led.setPattern(RevBlinkinPatterns.WHITE));
    //led.setDefaultCommand(led.HoldSetColour());

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
    Controller2.y().whileTrue(new SetShooterRotation(artShooter, () -> poseEstimator.getSpeakerTransform().getX(), shooter)).onTrue(intake.setPosition(positions.UPPER));
    //git hub trial//
    Controller2.x().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.AMP_AREA));
    //Controller2.x().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.SAFE_ZONE)).onTrue(intake.setPosition(positions.UPPER));
    Controller2.a().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.WALL_AREA))/*.onTrue(intake.setPosition(positions.UPPER))* */;
    Controller2.b().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.SAFE_ZONE)).onTrue(intake.setPosition(positions.UPPER));
    //Controller2.x().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.WHITE_LINE)).onTrue(intake.setPosition(positions.UPPER));
    Controller1.rightTrigger(0.2).whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.AUTO_FEED));
    Controller1.rightTrigger(0.2).whileTrue(new DefualtIndexerCommand(() -> shooter.isShooting(), () -> Controller2.getRightTriggerAxis(), () -> Controller1.getRightTriggerAxis()));
    Controller1.rightTrigger(0.2).whileTrue(new FeedIntakeCommand());
    Controller1.rightTrigger(0.2).whileTrue(led.setPattern(RevBlinkinPatterns.SHOT_BLUE));
    Controller1.rightTrigger(0.2).onFalse(led.setPattern(RevBlinkinPatterns.SHOT_RED));
    Controller1.rightTrigger(0.2).onTrue(intake.setPosition(positions.LOWER));
    // Controller2.x().whileTrue(new SetShooterCommand(shooter, artShooter, ShooterPositions.WHITE_LINE)).onTrue(intake.setPosition(positions.UPPER));

    //Controller3.b().whileTrue(new MakePathOnTheFly(0, false, new PathConstraints(2, 0.5, 1, 0.5), poseEstimator, DriverStation.getAlliance().get()));

    // new Trigger(Controller2.rightBumper()).whileTrue(new FeedIntakeCommand());
    // new Trigger(Controller2.leftBumper()).whileTrue(new BackIntakeCommand(intake));
    Controller2.leftBumper().whileTrue(new unIndexCOmmand(indexer));
    Controller2.rightBumper().whileTrue(new DefualtIndexerCommand(() -> shooter.isShooting(), () -> Controller2.getRightTriggerAxis(), () -> Controller2.getRightTriggerAxis()));                                                       
   
    //new Trigger(Controller2.rightBumper()).whileTrue(intake.setPosition(positions.LOWER)).onFalse(intake.setPosition(positions.UPPER));
    
    //Controller2.rightBumper().onTrue(intake.setPosition(positions.LOWER));

    //Controller2.rightBumper().whileTrue(new RunCommand(() -> intake.driveFeed())).whileFalse(new InstantCommand(() -> intake.stopFeed()));
    Controller2.leftBumper().whileTrue(new BackIntakeCommand(intake));
    //Controller2.start().whileTrue(new ClimberCommand(climber, () -> Controller2.getLeftY(), artShooter));
    Controller2.start().onTrue(artShooter.changeClimbStatus()).onTrue(climber.setPower(Direction.kForward));
    Controller2.back().onTrue(climber.setPower(Direction.kReverse)).onTrue(artShooter.setDown())//.onTrue(()-> artShooter.setCustomAngle(15));
;
    Controller2.start().onTrue(led.setPattern(RevBlinkinPatterns.VIOLET));

    //Controller2.start().onTrue(led.setColour(Colours.RED));

    // new Trigger(Controller2.pov(0).onTrue(new InstantCommand( () -> intake.setPosition(positions.UPPER))));
    // new Trigger(Controller2.pov(180).onTrue(new InstantCommand(() -> intake.setPosition(positions.LOWER))));

    
    new Trigger(Controller2.pov(0).onTrue(intake.setPosition(positions.UPPER)));
    new Trigger(Controller2.pov(180).onTrue(intake.setPosition(positions.LOWER)));
    
    Controller2.rightBumper().whileTrue(new FeedIntakeCommand());

    Controller2.rightTrigger(0.2).whileTrue(new DefualtShooter(indexer, () -> shooter.isShooting(), Controller2.rightTrigger(0.2)));

    Controller2.leftTrigger(0.2).onTrue(new REverseIndexerCommand(indexer, () -> indexer.pastSensor(), () -> indexer.getPhotoReading()).withTimeout(1));

    new Trigger(() -> indexer.getPhotoReading()).onFalse(led.setPattern(RevBlinkinPatterns.WHITE)).onTrue(led.setPattern(RevBlinkinPatterns.ORANGE));
    new Trigger(() -> shooter.atSpeed()).onTrue(led.setPattern(RevBlinkinPatterns.GREEN)).onFalse(led.setPattern(RevBlinkinPatterns.WHITE));
    new Trigger(() -> shooter.atSpeed()).whileTrue(new InstantCommand(() -> Controller2.getHID().setRumble(RumbleType.kBothRumble, 0.1))).onFalse(new InstantCommand(() -> Controller2.getHID().setRumble(RumbleType.kBothRumble, 0)));
    
    
      
    

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
    //return new testAuto(drive, intake, indexer, shooter, artShooter);
    //return new PathPlannerAuto("Example Auto");
  }
}
