package missdaisy.autonomous;

import missdaisy.subsystems.Shooter;

public class SpinUpShooter extends TimeoutState {
  
  Shooter mShooter;

  public SpinUpShooter(int timeout) {
    super("SpinUpShooter", timeout);
    // TODO Auto-generated constructor stub
    mShooter = Shooter.getInstance();
  }

  public void enter() {
    super.enter();
    mShooter.setOpenLoop();
    mShooter.setRpm(3750);
  }
  
  @Override
  public void running() {
    mShooter.setRpm(3750);
  }

  @Override
  public boolean isDone() {
    return super.isDone();
  }

}
