package missdaisy.autonomous;

import missdaisy.loops.controllers.ShooterSpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.loops.controllers.AutoAimTurretController;
import missdaisy.subsystems.Shooter;
import missdaisy.subsystems.Turret;

public class AutoAimAndShoot extends State {

  private Turret mTurret;
  private Shooter mShooter;
  private ShooterSpeedController mShooterSpeedController;
  private AutoAimTurretController mTurretController;
  private int onTargetCount;
  private double mShootAnywayTime = 1.5;
  private double stateStartTime;
  private double RPM;
  private boolean turretAngleSet = false;

  public AutoAimAndShoot(double RPM) {
    super("AutoAimAndShoot");
    mTurret = Turret.getInstance();
    mShooter = Shooter.getInstance();
    this.RPM = RPM;
    if (this.RPM < 10) {
      RPM = 0;
    }
    mShooterSpeedController = ShooterSpeedController.getInstance();
    mTurretController = AutoAimTurretController.getInstance();
  }

  @Override
  public void enter() {
    onTargetCount = 0;
    mTurret.setCurrentController(mTurretController);
    if (this.RPM != 0) {
      mShooter.enableSpeedControlMode(this.RPM);
    } else {
      mShooter.setCurrentController(mShooterSpeedController);
    }
    mShooterSpeedController.setRPMSOffest(SmartDashboard.getNumber("RPMS Offest", 0.0));
    stateStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void running() {
    SmartDashboard.putNumber("AASAuton RPM Value", this.RPM);
    if (this.RPM != 0) {
      mShooter.enableSpeedControlMode(this.RPM);
    }
    if (mTurretController.onTarget()) {
      onTargetCount++;
    }
    if (onTargetCount > 5 || (mShootAnywayTime + stateStartTime < Timer.getFPGATimestamp())) {
      if (mShooter.getRPM() > 1000) {
        mShooter.feedFuel();
      }
      if (!turretAngleSet) {
        // We found the goal, now shoot at it and ignore any balls that fall in front of the camera
        mTurret.setOpenLoop();
        mTurret.setAngle(mTurret.getTurretAngle());
        turretAngleSet = true;
      }
    } else {
      turretAngleSet = false;
    }
  }

  @Override
  public boolean isDone() {
    return false;
  }
}
