package missdaisy.loops.controllers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Vision;
import missdaisy.loops.Navigation;
import missdaisy.utilities.DaisyMath;

/**
 * Since our new shooter is fairly wobbly, the camera moves left and right as our turn controller
 * jerks the robot towards its desired heading. This makes our desired azimuth change, resulting in a 
 * changing goal input into the turn controller. To combat this, we decide to trust the vision system and
 * take one sample of the azimuth, turn to it, and when the controller says we should be where we want to be,
 * we take another sample of the azimuth. If we are within azumuthal tolerance, we'll stay where we are, or we'll tell the 
 * turn controller to take control again and set a new goal.
 * 
 */
public class AutoAimDriveController implements Controller {
  private static AutoAimDriveController autoAimDriveControllerInstance = null;
  private Vision mVision;
  private Navigation mNavigation;
  private DriveTurnController2 mDrive;
  private boolean mGoalSet;
  private double mAzimuthTolerance = 1.5;

  /**
   * Gets the instance of the auto-aim drive turn controller. Used in order to never have more than
   * one auto aim-drive turn controller object, ever.
   * 
   * @return The one and only instance of the auto-aim drive turn controller
   */
  public static AutoAimDriveController getInstance() {
    if (autoAimDriveControllerInstance == null)
      autoAimDriveControllerInstance = new AutoAimDriveController();
    return autoAimDriveControllerInstance;
  }

  private AutoAimDriveController() {
    mVision = Vision.getInstance();
    mNavigation = Navigation.getInstance();
    mDrive = DriveTurnController2.getInstance();
    mGoalSet = false;
  }

  /**
   * Tries to turn the robot to the target if it identifies the target. If it cannot find the
   * target, the robot attempts to turn towards the last known goal location
   */
  @Override
  public synchronized void run() {
    // if we can see the target, then we set the goal to the
    // angle that we want to turn to
    
    if (mVision.seesTarget()) {
      if (!mGoalSet) {
        mDrive.setGoal(mVision.getAzimuth());
        mGoalSet = true;
      }
      
      double azimuthError = Math.abs(DaisyMath.boundAngle0to360Degrees(mNavigation.getHeadingInDegrees() - mVision.getAzimuth()));
      if (mGoalSet && mDrive.onTarget() && (azimuthError > mAzimuthTolerance)) {
        mGoalSet = false;
      }
    } else {
      mDrive.setGoal(mNavigation.getApproxTargetAngle());
      mGoalSet = false;
    }
    
    mDrive.run();
  }

  /**
   * Resets the controller's goal to be the current angle of the robot
   */
  @Override
  public synchronized void reset() {
    mDrive.reset();
    mGoalSet = false;
  }

  /**
   * Returns true if the robot is facing the target correctly Returns false otherwise
   */
  @Override
  public synchronized boolean onTarget() {
    return mDrive.onTarget();
  }

  @Override
  public void loadProperties() {
    // No properties to load
  }

  @Override
  public String toString() {
    return "AutoAimDriveController";
  }
}