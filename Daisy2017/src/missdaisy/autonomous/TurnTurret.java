package missdaisy.autonomous;

import missdaisy.subsystems.Turret;

/**
 * Drive the specified segment of the trajectory
 * 
 */

public class TurnTurret extends State {
  private Turret mTurret;
  private double turnAngle;

  /**
   * @param distance The distance to travel, in say inches.
   * @param speed A number between -1.0 and 1.0
   */
  public TurnTurret(double angle) {
    super("TurnTurret");
    mTurret = Turret.getInstance();
    turnAngle = angle;
  }

  /**
   * Sets the drive base's current controller to be the drive distance controller
   */
  @Override
  public void enter() {
    mTurret.setAngle(turnAngle);
  }

  @Override
  public void running() {}

  /**
   * Ensures the robot's drive base is in an expected state.
   */
  @Override
  public void exit() {}

  /**
   * This state is considered done if the drive distance controller is on target
   */
  @Override
  public boolean isDone() {
    return true;
  }
}
