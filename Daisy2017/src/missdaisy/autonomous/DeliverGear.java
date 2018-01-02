package missdaisy.autonomous;

import missdaisy.subsystems.Drive;
import missdaisy.subsystems.Gearipor;

public class DeliverGear extends TimeoutState {
  private Drive mDrive;
  private Gearipor mGearipor;
  private double startTime;
  private boolean gearDelivered = false;
  private int driveBackCounter = 0;
  private int droppedGearCounter = 0;

  // todo: Set timeout to be a parameter
  public DeliverGear() {
    super("DeliverGear", 1000);
    mDrive = Drive.getInstance();
    mGearipor = Gearipor.getInstance();
  }

  /**
   * Sets the drive base's current controller to be the drive distance controller
   */
  @Override
  public void enter() {
    super.enter();
    gearDelivered = false;
    droppedGearCounter = 0;
    mDrive.setOpenLoop();
  }

  @Override
  public void running() {
    // if (!mGearipor.seesHook()){
    // mDrive.setSpeedTurn(0.05, 0.0);
    // droppedGearCounter = 0;
    // } else {
    if (driveBackCounter < 4) {
      mDrive.setSpeed(0.3, 0.3);
    } else {
      mDrive.setSpeed(0.0, 0.0);
      mGearipor.openScorer();
      droppedGearCounter++;
    }
    driveBackCounter++;
    gearDelivered = droppedGearCounter >= 10;
    // }
  }

  /**
   * Ensures the robot's drive base is in an expected state.
   */
  @Override
  public void exit() {
    mDrive.setOpenLoop();
    mDrive.setSpeedTurn(0.0, 0.0);
  }

  @Override
  public boolean isDone() {
    return gearDelivered || super.isDone();
  }
}
