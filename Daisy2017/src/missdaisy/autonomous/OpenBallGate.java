package missdaisy.autonomous;

import missdaisy.subsystems.Gearipor;

public class OpenBallGate extends State {
  private Gearipor mGearipor;

  // todo: Set timeout to be a parameter
  public OpenBallGate() {
    super("OpenBallGate");
    mGearipor = Gearipor.getInstance();
  }

  /**
   * Sets the drive base's current controller to be the drive distance controller
   */
  @Override
  public void enter() {
    super.enter();
    mGearipor.openLoader();
    mGearipor.openBallGate();
  }

  @Override
  public void running() {
    mGearipor.openLoader();
    mGearipor.openBallGate();
  }

  /**
   * Ensures the robot's drive base is in an expected state.
   */
  @Override
  public void exit() {}

  @Override
  public boolean isDone() {
    return true;
  }

}
