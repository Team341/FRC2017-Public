package missdaisy.autonomous;

import edu.wpi.first.wpilibj.Timer;
import missdaisy.subsystems.Shooter;

public class JustShoot extends State {

  private Shooter mShooter;
  private int onTargetCount;
  private double mShootAnywayTime = 1;
  private double stateStartTime;
  private double mAngle;
  private double mRPMS;

  public JustShoot(double angle, double rpm) {
    super("JustShoot");
    mShooter = Shooter.getInstance();
    mAngle = angle;
    mRPMS = rpm;
  }

  @Override
  public void enter() {
    onTargetCount = 0;
    System.out.println("Just Shoot Entered RPMS: " + mRPMS);
    mShooter.setRpm(mRPMS);
    stateStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void running() {
    mShooter.setRpm(mRPMS);
    if (mShooter.onTarget()) {
      onTargetCount++;
    }
    if ((onTargetCount > 5 || (mShootAnywayTime + stateStartTime < Timer.getFPGATimestamp()))
        && mShooter.getRPM() > 1000) {
      mShooter.feedFuel();
    }
  }

  public void exit() {
    mShooter.setPercVoltage(0.0);
  }

  @Override
  public boolean isDone() {
    return stateStartTime + 10.0 < Timer.getFPGATimestamp();
  }
}
