package missdaisy.autonomous;

import missdaisy.Vision;
import missdaisy.loops.Navigation;
import missdaisy.loops.controllers.DriveDistanceController;
import missdaisy.subsystems.Drive;

/**
 * A lot of these autonomous states were not used in the actual robot.
 */
public class DriveToTarget extends State {

  private Drive mDrive;
  private DriveDistanceController mDriveController;
  private Vision mVision;
  private double mSpeed;
  private double mDesiredDistance;
  private double mCurrentDistance;
  private double mDistanceToTravel;
  private Navigation mNav;

  public DriveToTarget(double distanceFromTarget, double speed) {
    super("DriveToTarget");
    mDrive = Drive.getInstance();
    mNav = Navigation.getInstance();
    mDriveController = DriveDistanceController.getInstance();
    mDesiredDistance = distanceFromTarget;
    mSpeed = speed;
  }

  @Override
  public void enter() {
    mCurrentDistance = mNav.getAverageEncoderDistance();
    mDistanceToTravel = mCurrentDistance - mDesiredDistance;
    mDriveController.setGoal(mDistanceToTravel, mSpeed);
    mDrive.setCurrentController(mDriveController);
  }

  @Override
  public void running() {}

  @Override
  public void exit() {
    mDrive.setOpenLoop();
  }

  @Override
  public boolean isDone() {
    return mDriveController.onTarget();
  }

}
