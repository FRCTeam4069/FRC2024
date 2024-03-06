// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

//import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.ShooterTest;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // private PhotonCamera cam;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_robotContainer.powerDistributionHub.clearStickyFaults();

    DriverStation.silenceJoystickConnectionWarning(true);

    // cam = new PhotonCamera("frontCamera");

    // m_robotContainer.intake.setBrakeState(1);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //SmartDashboard.putNumber("Distance", m_robotContainer.FrontCamera.getXDistanceToApriltag(4, 7));
    m_robotContainer.FrontCamera.printNumbers();

    // var res = cam.getLatestResult();
    // SmartDashboard.putBoolean("has targets", cam.getLatestResult().hasTargets());
    // if (res.hasTargets()) {
    //   for (PhotonTrackedTarget target : res.targets) {
    //     SmartDashboard.putNumber(Integer.toString(target.getFiducialId()), target.getSkew());
    //   }
    //   SmartDashboard.putNumber("cam x", res.getMultiTagResult().estimatedPose.best.getX());
    //   SmartDashboard.putNumber("cam y", res.getMultiTagResult().estimatedPose.best.getY());
    //   SmartDashboard.putNumber("cam yes", res.getBestTarget().getYaw());
    // }
    //m_robotContainer.FrontCamera.printNumbers();
    //RobotContainer.poseEstimator.addDashboardWidgets(RobotContainer.autoTab);
    //RobotContainer.led.setColour(Colours.BLUE);
    //SmartDashboard.putNumber("Sensor", m_robotContainer.indexer.getPhotoReading());
    //SmartDashboard.putNumber("intake encoder", m_robotContainer.intake.getEncoder());
  //   SmartDashboard.putNumber("Angle", m_robotContainer.artShooter.getEncoder());
  //  SmartDashboard.putNumber("Sensor", m_robotContainer.indexer.getPhotoReading());
   //m_robotContainer.cam.printerNumbers();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.intake.setBrakeState(1);

  }

  @Override
  public void disabledPeriodic() {
    //m_robotContainer.led.setColour(Colours.COOL_PATTERN);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    m_robotContainer.intake.setBrakeState(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.intake.setBrakeState(0);

    

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //double values[] = m_robotContainer.drive.getEncoderValues();
    //SmartDashboard.putNumberArray("modules", values);
    //SmartDashboard.putNumber("1", values[1]);
    //SmartDashboard.putNumber("2", values[2]);
    //SmartDashboard.putNumber("3", values[3]);
    
  }
  //private ShooterTest t;
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    //c = new ShooterRotationController();

    SmartDashboard.clearPersistent("Field");

    //t = new ShooterTest();
   
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //SmartDashboard.putNumber("SHOOTER", c.getEncoder());
    SmartDashboard.putNumber("intake", m_robotContainer.intake.getEncoder());
  }
  

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

}