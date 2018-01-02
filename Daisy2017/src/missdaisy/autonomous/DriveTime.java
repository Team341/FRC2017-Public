package missdaisy.autonomous;

import missdaisy.loops.Navigation;
import missdaisy.subsystems.Drive;
import missdaisy.utilities.DaisyMath;

/**
 * A very rudimentary class that simply drives the drive base for some specified amount of time. It
 * also uses a proportional gain on the gyroscopes heading reading in order to drive straight.
 *
 * @author Joshua Sizer
 */
public class DriveTime extends TimeoutState {

  private final Drive mDrive;
  private final Navigation mNav;
  private final double kP = 0.03;
  private double mSpeed;
  private double mStartYaw;

  /**
   * @param speed A percentage of maximum motor output. 1.0 for forward, -1.0 for reverse.
   * @param timeout The time, in milliseconds, to run this state for.
   */
  public DriveTime(double speed, int timeout) {
    super("DriveTime", timeout);
    mSpeed = speed;
    mDrive = Drive.getInstance();
    mNav = Navigation.getInstance();
  }

  /**
   * Finds the start angle of the system, and tells the drive to not use the alpha filter
   */
  @Override
  public void enter() {
    super.enter();
    mStartYaw = mNav.getHeadingInDegrees();
    mDrive.useAlphaFilter(false);
  }

  /**
   * Uses a P gain to determine what turn value to send to the drive base.
   */
  @Override
  public void running() {
    double turn = getYawError() * kP;
    mDrive.setSpeedTurn(mSpeed, turn);
  }

  /**
   * Returns the robot drive base to a known state of stopped.
   */
  @Override
  public void exit() {
    mDrive.setSpeed(0.0, 0.0);
    mDrive.useAlphaFilter(true);
  }

  /**
   * @return The angle error from when this state first started
   */
  private double getYawError() {
    double error = DaisyMath.boundAngle0to360Degrees(mStartYaw - mNav.getHeadingInDegrees());
    return error > 180 ? error -= 360 : error;
  }
}
