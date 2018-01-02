package missdaisy.autonomous;

import missdaisy.subsystems.Intake;

public class RunIntake extends TimeoutState{
  private Intake mIntake;
  private double mSpeed = 0.0;

  // todo: Set timeout to be a parameter
  public RunIntake(double speed) {
    super("RunIntake", 1000);
    mIntake = Intake.getInstance();
    mSpeed = speed;
  }

  /**
   * Sets the drive base's current controller to be the drive distance controller
   */
  @Override
  public void enter() {
    super.enter();
    mIntake.setIntakeSpeed(mSpeed);
  }

  @Override
  public void running() {
    
  }

  /**
   * Ensures the robot's drive base is in an expected state.
   */
  @Override
  public void exit() {
    
  }

  @Override
  public boolean isDone() {
    return true;
  }
}