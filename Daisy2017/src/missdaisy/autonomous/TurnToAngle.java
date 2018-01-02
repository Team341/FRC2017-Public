package missdaisy.autonomous;

import missdaisy.Vision;
import missdaisy.loops.controllers.DriveTurnController;
import missdaisy.subsystems.Drive;
import missdaisy.utilities.DaisyMath;

/**
 * This is different from the Turn state. This state will turn to an absolute angle, instead of an
 * angle that is x degrees from the current heading.
 *
 * @author Josh Sizer
 */
public class TurnToAngle extends State {

  private Drive mDrive;
  private DriveTurnController mDriveController;
  private double mAngle;
  private Vision mVision;

  public TurnToAngle(double angle) {
    super("TurnToAngle");
    mDrive = Drive.getInstance();
    mDriveController = DriveTurnController.getInstance();
    mAngle = DaisyMath.boundAngle0to360Degrees(angle);
  }

  @Override
  public void enter() {
    mDriveController.setGoal(mAngle);
    mDrive.setCurrentController(mDriveController);
  }

  @Override
  public void running() {}

  @Override
  public void exit() {
    mDrive.setOpenLoop();
    mDrive.setSpeedTurn(0.0, 0.0);
  }

  @Override
  public boolean isDone() {
    return mDriveController.onTarget();
  }
}
