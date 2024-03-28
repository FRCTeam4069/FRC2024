package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Limelight.PoseEstimatorSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class SystemCheck {
    //PoseEstimatorSubsystem limelight;
    ShooterController shooter;
    ShooterRotationController shooterRotation;
    IntakeController intake;
    IndexerController indexer;
   // SwerveDrivetrain sdb;
    

    public SystemCheck(PoseEstimatorSubsystem l, ShooterController s, ShooterRotationController r, IntakeController i, IndexerController ix, SwerveDrivetrain sw){
        //limelight = l;
        shooter = s;
        shooterRotation = r;
        intake = i;
        indexer = ix;
        //sdb = sw;

        
    }

    public void checkSystem(){
        if(shooterRotation.getLeftErrors() + shooterRotation.getRightErrors() + indexer.getErrors() + intake.getArtErrors() + intake.getFeedErrors() == 0){
            SmartDashboard.putBoolean("System Clear", true);
        }
        else{
            SmartDashboard.putBoolean("System Clear", false);

            SmartDashboard.putBoolean("Left Shooter Motor Fault", shooterRotation.getLeftMotorError());
            SmartDashboard.putBoolean("Left Shooter Sensor Fault", shooterRotation.getLeftSensorError());
            SmartDashboard.putBoolean("Right Shooter Motor Fault", shooterRotation.getRightMotorError());
            SmartDashboard.putBoolean("Right Shooter Sensor Fault", shooterRotation.getRightSensorError());

            SmartDashboard.putBoolean("Intake Feed Motor Fault", intake.getFeedMotorError());
            SmartDashboard.putBoolean("Intake Feed Sensor Fault", intake.getFeedSensorError());
            SmartDashboard.putBoolean("Intake Articulate Motor Fault", intake.getArtMotorError());
            SmartDashboard.putBoolean("Intake Articulate Motor Fault", intake.getArtSensorError());

            SmartDashboard.putBoolean("Indexer Motor Fault", indexer.getMotorError());
            SmartDashboard.putBoolean("Indexer Sensor Fault", indexer.getSensorError());
        }
    }

    
}
