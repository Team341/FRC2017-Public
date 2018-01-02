package missdaisy.autonomous;

import missdaisy.loops.Navigation;

/**
 * Resets the robot heading
 *
 * @author Joshua Sizer
 */
public class ResetHeading extends State {

  private Navigation mNavigation;
  private double mAngle;

  /**
   * @param angle The yaw angle to reset the robot to.
   */
  public ResetHeading(double angle) {
    super("ResetHeading");
    mNavigation = Navigation.getInstance();
    mAngle = angle;
  }

  /**
   * Sets the robot's field position to be (0.0, 0.0),
   */
  @Override
  public void enter() {
    mNavigation.resetRobotPosition(0, 0, mAngle);;
  }

  @Override
  public void running() {}

  /**
   * Only needs to be run once.
   */
  @Override
  public boolean isDone() {
    return true;
  }

}
