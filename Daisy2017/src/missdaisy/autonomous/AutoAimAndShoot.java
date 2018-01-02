package missdaisy.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;
import missdaisy.Vision;
import missdaisy.loops.Navigation;
import missdaisy.loops.controllers.AutoAimDriveController;
import missdaisy.loops.controllers.ShooterSpeedControllerOLD;
import missdaisy.subsystems.Drive;
import missdaisy.subsystems.Intake;
import missdaisy.subsystems.Shooter;
import missdaisy.utilities.DaisyMath;

/**
 * This state will attempt to aim at the target, and shoot when the robot is on target. In this
 * class is some logic that will move the robot forward/backward to ensure a correct distance from
 * the target. If the target is not visible to the robot, it will attempt a sweep left/right to find
 * the target.
 *
 * @author Adam N.
 */
public class AutoAimAndShoot extends State {
  private final Drive mDrive;
  private final Shooter mShooter;
  private final Intake mIntake;
  // The controllers that will attempt to aim and shoot
  private final AutoAimDriveController mDriveController;
  private final ShooterSpeedControllerOLD mShooterController;
  private final Navigation mNavigation;
  private boolean mBallFired;
  private long startTime = -1;
  private long driveStartTime = 0;
  private long turnStartTime = 0;
  private long waitTime = 250;
  private int onTargetCounter;
  private double aimAngle;
  private double minRange = 230; // (112) for competition
  private double maxRange = 300; // (176) for competition
  private double initialTurnDirection = 0.0;
  private double stateStartTime = -1;
  private double kShootAnyTime = 2;
  private double mDriveOnTargetCounter = 0;

  /**
   * 
   * @param desiredAngle The angle we think the target will be at.
   */
  public AutoAimAndShoot(double desiredAngle) {
    super("AutoAimAndShoot");
    mDrive = Drive.getInstance();
    mShooter = Shooter.getInstance();
    mIntake = Intake.getInstance();
    mDriveController = AutoAimDriveController.getInstance();
    mShooterController = ShooterSpeedControllerOLD.getInstance();
    mNavigation = Navigation.getInstance();
    mBallFired = false;
    aimAngle = 0.0;
  }

  public void enter() {
    // ensures all subsystems are in the correct state
    // mDrive.setCurrentController(mDriveController);
    //mShooter.setCurrentController(mShooterController);
    startTime = -1;
    onTargetCounter = 0;
    driveStartTime = System.currentTimeMillis();
    stateStartTime = Timer.getFPGATimestamp();
    mShooter.setRpm(SmartDashboard.getNumber("shooterRPM", 3750) + 350);
  }

  @Override
  public void running() {    
    // logging for debugging purposes
    SmartDashboard.putBoolean("AAS_ShooterOnTarget", mShooterController.onTarget());
    SmartDashboard.putBoolean("AAS_DriveOnTarget", mDriveController.onTarget());
  
    mShooter.setRpm(SmartDashboard.getNumber("shooterRPM", 3750) + 250);
    mDrive.setCurrentController(mDriveController);
    
    if (Timer.getFPGATimestamp() - stateStartTime > kShootAnyTime) {
      mShooter.feedFuel();
    } else if (mDriveController.onTarget() && mShooterController.onTarget()) {
      onTargetCounter++;
      if (onTargetCounter > 10) {
        mShooter.feedFuel();
        if (startTime < 0.0)
          startTime = System.currentTimeMillis();
      }
    }
    
    if (mDriveController.onTarget()) {
      mDriveOnTargetCounter++;
    } else {
      mDriveOnTargetCounter = 0;
    }
    
    if (mDriveOnTargetCounter > 5) {
      mDrive.setOpenLoop();
    }

    SmartDashboard.putNumber("AAS_StartTime", startTime);
   
    SmartDashboard.putNumber("AAS_ElapsedTime", System.currentTimeMillis() - startTime);

    SmartDashboard.putBoolean("AAS_BallFired", mBallFired);
  }

  @Override
  public boolean isDone() {
    return false;
  }

  /**
   * Ensures the robot is in the desired state when this routine is done
   */
  public void exit() {
    mShooter.setOpenLoop();
    mShooter.setPercVoltage(0.0);
    mDrive.setOpenLoop();
    mDrive.setSpeed(0.0, 0.0);
    mShooter.stopFeed();
  }
}