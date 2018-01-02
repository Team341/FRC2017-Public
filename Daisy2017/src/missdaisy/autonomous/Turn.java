package missdaisy.autonomous;

import missdaisy.loops.Navigation;
import missdaisy.loops.controllers.DriveTurnController;
import missdaisy.subsystems.Drive;
import missdaisy.utilities.DaisyMath;

/**
 * Turns the robot to an angle relative to the robot's current angular position.
 *
 * @author Joshua Sizer
 */
public class Turn extends State {

  private Drive mDrive;
  private Navigation mNavigation;
  private DriveTurnController mDriveController;
  private double mAngle;

  /**
   * @param angle The angle to turn to.
   */
  public Turn(double angle) {
    super("Turn");
    mDrive = Drive.getInstance();
    mNavigation = Navigation.getInstance();
    mDriveController = DriveTurnController.getInstance();
    mAngle = angle;
  }

  @Override
  public void enter() {
    mDriveController
        .setGoal(DaisyMath.boundAngle0to360Degrees(mNavigation.getHeadingInDegrees() + mAngle));
    mDrive.setCurrentController(mDriveController);
  }

  /**
   * The drive turn controller runs in it's own thread
   */
  @Override
  public void running() {}

  /**
   * Puts the robot back into a known state.
   */
  @Override
  public void exit() {
    mDrive.setOpenLoop();
  }


  @Override
  public boolean isDone() {
    return mDriveController.onTarget();
  }

}
