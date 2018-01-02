package missdaisy.loops.controllers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;
import missdaisy.Vision;
import missdaisy.loops.SynchronousPID;
import missdaisy.subsystems.Turret;
import missdaisy.tracking.ReferenceFrame;
import missdaisy.tracking.RobotPose;
import missdaisy.tracking.Track;
import missdaisy.tracking.TrackManager;
import missdaisy.utilities.DaisyMath;
import missdaisy.utilities.InterpolatingDouble;
import missdaisy.utilities.MovingAverageFilter;

/**
 * A controller that sets the RPM according the value supplied by the vision processing, ie by how
 * far away from the target the robot is.
 */

public class AutoAimTurretController implements Controller {

  private static AutoAimTurretController autoAimTurretControllerInstance = null;
  private Turret mTurret;
  private Vision mVision;
  private TrackManager mTrackManager = null;
  private MovingAverageFilter movingAverageFilter;
  private SynchronousPID controller;
  private double desiredTurretYaw = 0.0;

  /**
   * Gets the instance of the auto-aim shooter speed controller. Used in order to never have more
   * than one auto-aim shooter speed controller object, ever.
   *
   * @return The one and only instance of the auto-aim shooter speed controller
   */
  public static AutoAimTurretController getInstance() {
    if (autoAimTurretControllerInstance == null) {
      autoAimTurretControllerInstance = new AutoAimTurretController();
    }
    return autoAimTurretControllerInstance;
  }

  private AutoAimTurretController() {
    mTurret = Turret.getInstance();
    mTrackManager = TrackManager.getInstance();
    mVision = Vision.getInstance();
    movingAverageFilter = new MovingAverageFilter(5);
    controller = new SynchronousPID();
    controller.setPID(0.018, 0.0, 0.06);
    controller.setOutputRange(-0.2, 0.2);
    controller.setInputRange(-180, 180);
    controller.setSetpoint(0.0);
    SmartDashboard.putNumber("AutoAimTurret P", 0.018);
    SmartDashboard.putNumber("AutoAimTurret I", 0.0);
    SmartDashboard.putNumber("AutoAimTurret D", 0.06);
    
    loadProperties();
  }

  public synchronized void setGoal() {

  }
  
  public synchronized void tunePID(boolean tune) {

    if (tune) {
      double p = SmartDashboard.getNumber("AutoAimTurret P", 0.0);
      double i = SmartDashboard.getNumber("AutoAimTurret I", 0.0);
      double d = SmartDashboard.getNumber("AutoAimTurret D", 0.0);
      controller.setPID(p, i, d);
    }
  }

  /**
   * If the robot sees the target, the shooter will ramp up to the specified RPM based on distance
   * from the target. Otherwise, it will ramp up to the default RPM
   */
  @Override
  public synchronized void run() {
    tunePID(false);
    double output = controller.calculate(mVision.getAzimuth());
    SmartDashboard.putNumber("AutoAimTurretAngle Command", output);
    if (onTarget()) {
      mTurret.setSpeed(0.0);
    } else {
      mTurret.setSpeed(output);
    }
    /*
    // Get the best boiler track 
    Track bestTrk = mTrackManager.getBestBoilerTrack();
    if (Constants.DEBUG_MODE) {
      SmartDashboard.putBoolean("AutoAimTurret_BestTrkFound", bestTrk != null);
    }
    // Get the current robot pose
    RobotPose.Pose rPose = RobotPose.getInterpolated(new InterpolatingDouble(Timer.getFPGATimestamp()));

    if (rPose != null){
      
      
      // Convert the camera measurements into an xy position relative to
      // the camera
      ReferenceFrame targetPosInCameraFrame =
          new ReferenceFrame(mVision.getRange() * Math.cos(mVision.getAzimuth()), mVision.getRange() * Math.sin(mVision.getAzimuth()), 0.0);

      // Rotate target point from camera frame to turret frame
      ReferenceFrame targetPosInTurretFrame =
          rPose.cameraFrame.rotateFrom(targetPosInCameraFrame);
      
      // The angle to the goal in turret frame is the effective error, adjust the current turret angle by this amount
      double targetYawInTurretFrame = Math.toDegrees(DaisyMath.boundAngleNegPiToPiRadians(Math.atan2(targetPosInTurretFrame.getY(), targetPosInTurretFrame.getX())-targetPosInTurretFrame.getYaw()));
      double newYaw = DaisyMath.boundAngleNeg180to180Degrees(mTurret.getTurretAngle() + targetYawInTurretFrame);
      
      SmartDashboard.putNumber("AutoAim Target Yaw in Turret Frame", targetYawInTurretFrame);
      
      movingAverageFilter.setInput(newYaw);      
      movingAverageFilter.run();
      desiredTurretYaw = movingAverageFilter.getAverage();
      
      SmartDashboard.putNumber("AutoAimTurretAngle Command", desiredTurretYaw);
      
      mTurret.setAngle(desiredTurretYaw + mTurret.getTurretAngle());
      
      /*
      // Get the current robot pose
      RobotPose.Pose rPose = RobotPose.getInterpolated(new InterpolatingDouble(Timer.getFPGATimestamp()));
      
      // Convert the target into the robot frame
      ReferenceFrame targetInRobotFrame = rPose.driveFrame.rotateTo(bestTrk.getPosition());
      
      // Shift the target location relative to robot frame to the turret location
      ReferenceFrame turretFrame = new ReferenceFrame(Constants.Physical.TURRET_X_FROM_ROBOT_CENTER, Constants.Physical.TURRET_Y_FROM_ROBOT_CENTER, 0.0);
      ReferenceFrame targetInTurretFrame = turretFrame.rotateTo(targetInRobotFrame);
      
      SmartDashboard.putNumber("Target In Turret Frame Command", Math.toDegrees(targetInTurretFrame.getYaw()));

      double targetYawInTurretFrame = Math.toDegrees(Math.atan2(targetInTurretFrame.getY(), targetInTurretFrame.getX()));
      
      movingAverageFilter.setInput(targetYawInTurretFrame);
      movingAverageFilter.run();
      desiredTurretYaw = movingAverageFilter.getAverage();
      
      SmartDashboard.putNumber("AutoAimTurretAngle Command", targetYawInTurretFrame);
      mTurret.setAngle(targetYawInTurretFrame);
      */
      
      
      /*
      double trkYawInWF = Math.atan2(bestTrk.getPosition().getY(), bestTrk.getPosition().getX());
      
      double trkYawInRobotFrame = DaisyMath.boundAngleNegPiToPiRadians(trkYawInWF - rPose.driveFrame.getYaw());
      double trkYawInTurretFrame = DaisyMath.boundAngleNegPiToPiRadians(trkYawInRobotFrame - rPose.turretFrame.getYaw());
      
      SmartDashboard.putNumber("RobotPose_Yaw", Math.toDegrees(rPose.driveFrame.getYaw()));
      SmartDashboard.putNumber("RobotPose_turretYaw", Math.toDegrees(rPose.turretFrame.getYaw()));
      
      // Get the boilers range to the shooter (i.e. turret)
      ReferenceFrame turretPosInWF = rPose.turretFrame.mapTo(rPose.driveFrame);
      SmartDashboard.putNumber("Turret_YawInWF", Math.toDegrees(turretPosInWF.getYaw()));
      
      double angleToBoilerInWF = turretPosInWF.FindAngle(bestTrk.getPosition());
      //double turretError = DaisyMath.boundAngleNegPiToPiRadians(angleToBoilerInWF - turretPosInWF.getYaw());
      double turretError = DaisyMath.boundAngleNegPiToPiRadians(trkYawInTurretFrame - rPose.driveFrame.getYaw());
      
      SmartDashboard.putNumber("Angle to boiler in WF", angleToBoilerInWF);
      SmartDashboard.putNumber("Turret Error", turretError);
      // Tell the turret to turn to the 
      //mTurret.setAngle(DaisyMath.boundAngleNegPiToPiRadians(turretError + mTurret.getTurretAngle()));
      double desired = Math.toDegrees(trkYawInTurretFrame);//Math.toDegrees(angleToBoilerInWF);
      SmartDashboard.putNumber("AutoAimTurretAngle Command", desired);
      //mTurret.setAngle(desired);
      mTurret.setSpeed(1.5*turretError);
     
    
    }
    */
  }

  /**
   * Sets the shooter motor off
   */
  @Override
  public synchronized void reset() {
  }

  /**
   * Returns true if the shooter RPM is within an acceptable range of the goal RPM
   */
  @Override
  public synchronized boolean onTarget() {
    return controller.onTarget(.25);
    //return Math.abs(DaisyMath.boundAngleNeg180to180Degrees(desiredTurretYaw - mTurret.getTurretAngle())) < Constants.Properties.TURRET_ON_TARGET_TOLERANCE;
  }

  @Override
  public void loadProperties() {
  }

  @Override
  public String toString() {
    return "AutoAimTurretController";
  }
}
