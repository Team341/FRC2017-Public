package missdaisy.autonomous;

import missdaisy.loops.controllers.ShooterSpeedController;
import edu.wpi.first.wpilibj.Timer;
import missdaisy.loops.controllers.AutoAimTurretController;
import missdaisy.subsystems.Shooter;
import missdaisy.subsystems.Turret;

public class JustShoot extends State {

  private Turret mTurret;
  private Shooter mShooter;
  private int onTargetCount;
  private double mShootAnywayTime = 1;
  private double stateStartTime;
  private double mAngle;
  private double mRPMS;

  public JustShoot(double angle, double rpm) {
    super("JustShoot");
    mTurret = Turret.getInstance();
    mShooter = Shooter.getInstance();
    mAngle = angle;
    mRPMS = rpm;
  }

  @Override
  public void enter() {
    onTargetCount = 0;
    mTurret.setAngle(mAngle);
    System.out.println("Just Shoot Entered RPMS: " + mRPMS);
    mShooter.enableSpeedControlMode(mRPMS);
    stateStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void running() {
    mShooter.enableSpeedControlMode(mRPMS);
    if (mTurret.isOnTarget() && mShooter.onTarget()) {
      onTargetCount++;
    }
    if ((onTargetCount > 5 || (mShootAnywayTime + stateStartTime < Timer.getFPGATimestamp()))
        && mShooter.getRPM() > 1000) {
      mShooter.feedFuel();
    }
  }

  public void exit() {
    mShooter.setSpeed(0.0);
  }

  @Override
  public boolean isDone() {
    return stateStartTime + 10.0 < Timer.getFPGATimestamp();
  }
}
