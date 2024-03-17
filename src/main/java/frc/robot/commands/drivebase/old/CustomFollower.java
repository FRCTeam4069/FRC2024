// package frc.robot.commands.drivebase.old;

// import com.pathplanner.lib.path.PathPlannerTrajectory;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;

// public class CustomFollower {
//   private final PIDController xController;
//   private final PIDController yController;
//   private final PIDController rotationController;

//   private Translation2d translationError = new Translation2d();
//   private Rotation2d rotationError = new Rotation2d();
//   private Pose2d tolerance = new Pose2d();
//   private boolean isEnabled = true;

//   public CustomFollower(
//       PIDController xController, PIDController yController, PIDController rotationController) {
//     this.xController = xController;
//     this.yController = yController;
//     this.rotationController = rotationController;

//     this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
//   }

//   public boolean atReference() {
//     Translation2d translationTolerance = this.tolerance.getTranslation();
//     Rotation2d rotationTolerance = this.tolerance.getRotation();

//     return Math.abs(this.translationError.getX()) < translationTolerance.getX()
//         && Math.abs(this.translationError.getY()) < translationTolerance.getY()
//         && Math.abs(this.rotationError.getRadians()) < rotationTolerance.getRadians();
//   }

//   public void setTolerance(Pose2d tolerance) {
//     this.tolerance = tolerance;
//   }

//   /**
//    * Enables and disables the controller for troubleshooting. When calculate() is called on a
//    * disabled controller, only feedforward values are returned.
//    *
//    * @param enabled If the controller is enabled or not
//    */
//   public void setEnabled(boolean enabled) {
//     this.isEnabled = enabled;
//   }

//   /**
//    * Calculates the next output of the holonomic drive controller
//    *
//    * @param currentPose The current pose
//    * @param referenceState The desired trajectory state
//    * @return The next output of the holonomic drive controller
//    */
//   public ChassisSpeeds calculate(Pose2d currentPose, PathPlannerTrajectory.State referenceState) {
//     double xFF =
//         referenceState.velocityMps * referenceState.positionMeters.getRotation().getCos();
//     double yFF =
//         referenceState.velocityMps * referenceState.positionMeters.getRotation().getSin();
//     double rotationFF = -referenceState.getTargetHolonomicPose().getRotation().getRadians();

//     this.translationError = referenceState.positionMeters.relativeTo(currentPose).getTranslation();
//     this.rotationError = referenceState.targetHolonomicRotation.minus(currentPose.getRotation());

//     if (!this.isEnabled) {
//       return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, rotationFF, currentPose.getRotation());
//     }

//     double xFeedback =
//         this.xController.calculate(currentPose.getX(), referenceState.poseMeters.getX());
//     double yFeedback =
//         this.yController.calculate(currentPose.getY(), referenceState.poseMeters.getY());
//     double rotationFeedback =
//         -this.rotationController.calculate(
//             currentPose.getRotation().getRadians(), referenceState.holonomicRotation.getRadians());

//     return ChassisSpeeds.fromFieldRelativeSpeeds(
//         xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, currentPose.getRotation());
//   }
// }
