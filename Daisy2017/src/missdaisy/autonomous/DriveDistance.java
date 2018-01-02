package missdaisy.autonomous;

import missdaisy.loops.controllers.DriveDistanceController;
import missdaisy.subsystems.Drive;

/**
 * Attempts to drive a distance at a certain speed. The distance is in actual real world units. The
 * speed is some percent value of maximum motor power.
 */
public class DriveDistance extends State {

  private Drive mDrive;
  private DriveDistanceController mDriveController;
  private double mDistance;
  private double mSpeed;

  /**
   * @param distance The distance to travel, in say inches.
   * @param speed A number between -1.0 and 1.0
   */
  public DriveDistance(double distance, double speed) {
    super("DriveDistance");
    mDrive = Drive.getInstance();
    mDriveController = DriveDistanceController.getInstance();
    mDistance = distance;
    mSpeed = speed;
  }

  /**
   * Sets the drive base's current controller to be the drive distance controller
   */
  @Override
  public void enter() {
    mDriveController.setGoal(mDistance, mSpeed);
    mDrive.setCurrentController(mDriveController);
  }

  @Override
  public void running() {}

  /**
   * Ensures the robot's drive base is in an expected state.
   */
  @Override
  public void exit() {
    mDrive.setOpenLoop();
    mDrive.setSpeedTurn(0.0, 0.0);
  }

  /**
   * This state is considered done if the drive distance controller is on target
   */
  @Override
  public boolean isDone() {
    return mDriveController.onTarget();
  }
}
