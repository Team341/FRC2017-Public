package missdaisy.loops.controllers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.subsystems.Shooter;
import missdaisy.utilities.MovingAverageFilter;

public class ShooterSpeedController implements Controller {
  
  public enum State {
    SPIN_UP,
    KF_SAMPLING,
    KF_ONLY,
    MIDSTREAM_INCREASE,
    OPEN_LOOP
  }
  
  private static ShooterSpeedController mInstance = new ShooterSpeedController();
  
  public static ShooterSpeedController getInstance() {
    return mInstance;
  }
  
  private static final double kP = 0.00001;
  private static final double kI = 0.000000000001;
  private static final double kD = 0.00025;
  private static final double kF = 0.018;
  private static final int PIDF_PROFILE = 0;
  private static final int F_ONLY_PROFILE = 1;
  private static final double kAllowableError = 200;
  private static final int kKfSampleSize = 100;
  private double kMidstreamIncreaseTime = 1.5; // seconds
  private double kMidstreamIncreaseRpm = 500;
  
  private Shooter mShooter;
  private State mCurrentState;
  private MovingAverageFilter mKfSamples;
  private double mRpmSetpoint;
  private boolean mIsInTuningMode;
  private boolean mOnTarget;
  private double mNumKfSamples;
  private boolean mStateChanged;
  private double mStateStartTime;
  private double mStartShootingTime;
  private boolean mIsShooting;

  private ShooterSpeedController() {
    mShooter = Shooter.getInstance();
    mCurrentState = State.OPEN_LOOP;
    mKfSamples = new MovingAverageFilter(kKfSampleSize);
    mOnTarget = false;
    mNumKfSamples = 0.0;
    mStateStartTime = Timer.getFPGATimestamp();
    mStartShootingTime = Double.MAX_VALUE;
    mIsShooting = false;
    
    if (!SmartDashboard.containsKey("Shooter Tuning Mode")) {
      SmartDashboard.putBoolean("Shooter Tuning Mode", false);
    }
  }
  
  @Override
  public void run() {
    mIsInTuningMode = SmartDashboard.getBoolean("Shooter Tuning Mode", false);
    SmartDashboard.putString("Shooter state: ", "" + mCurrentState.toString());
    State newState = mCurrentState;
    double now = Timer.getFPGATimestamp();
    
    kMidstreamIncreaseRpm = SmartDashboard.getNumber("Shooter increase", kMidstreamIncreaseRpm);
    kMidstreamIncreaseTime = SmartDashboard.getNumber("Shooter increase delay", kMidstreamIncreaseTime);
    
    if (mStateChanged) {
      mStateStartTime = Timer.getFPGATimestamp();
    }
    
    if (mCurrentState == State.OPEN_LOOP) {
      mKfSamples.reset();
      mNumKfSamples = 0;
      mOnTarget = false;
    } else if (mCurrentState == State.SPIN_UP) {
      
      if (mIsInTuningMode) {
        mShooter.setPIDF(
            SmartDashboard.getNumber("Shooter kP", kP),
            SmartDashboard.getNumber("Shooter kI", kI),
            SmartDashboard.getNumber("Shooter kD", kD),
            SmartDashboard.getNumber("Shooter kF", kF));
      }
     
      if (onTarget(kAllowableError) && !mIsInTuningMode) {
        newState = State.KF_SAMPLING;
        configureForKfSampling();
      }
    } else if (mCurrentState == State.KF_SAMPLING) {
      if (onTarget(kAllowableError)) {
        double calcKf = calculateKf(mShooter.getRPM(), Math.abs(mShooter.getVoltage()));
        
        SmartDashboard.putNumber("Calculated kF", calcKf);
        
        mKfSamples.setInput(calcKf);
        mKfSamples.run();
        mNumKfSamples++;
      }
      
      if (mNumKfSamples >= kKfSampleSize) {
        newState = State.KF_ONLY;
        configureForKfOnly();
      }
    } else if (mCurrentState == State.KF_ONLY) {
      SmartDashboard.putNumber("KF_ONLY MotorKf", mShooter.getKf());
      mOnTarget = true;
      
      if (mIsShooting && (now - mStartShootingTime > kMidstreamIncreaseTime)) {
        configureForMidstreamIncrease();
        newState = State.MIDSTREAM_INCREASE;
      }
    } else if (mCurrentState == State.MIDSTREAM_INCREASE) {
      mOnTarget = true;
    }
    
    if (!newState.equals(mCurrentState)) {
      mStateChanged = true;
      mCurrentState = newState;
    } else {
      mStateChanged = false;
    }
    synchronized (this) {
      if (mCurrentState != State.OPEN_LOOP) {
        mShooter.setRpm(mRpmSetpoint);
        SmartDashboard.putNumber("ShooterSpeedController rpmsetpoint", mRpmSetpoint);
      } 
    }
    
  }
  
  public double calculateKf(double rpm, double voltage) {
    final double speed_in_ticks_per_100ms = 4096.0 / 600.0 * rpm;
    final double output = 1023.0 / 12.0 * voltage;
    return output / speed_in_ticks_per_100ms;
  }
  
  public synchronized void setRpmSetpoint(double rpm) {
    if (mCurrentState == State.OPEN_LOOP) {
      configureForSpinUp();
      mCurrentState = State.SPIN_UP;
      mStateChanged = true;
    }
    
    if (mCurrentState == State.MIDSTREAM_INCREASE) {
      mRpmSetpoint = rpm + kMidstreamIncreaseRpm;
    } else {
      mRpmSetpoint = rpm;
    }
  }
  
  private boolean onTarget(double allowableError) {
    return Math.abs(mShooter.getRPM() - mRpmSetpoint) < allowableError;
  }
  
  private void configureForSpinUp() {
    mShooter.disableNominalClosedLoopVoltage();
    mShooter.setProfile(PIDF_PROFILE);
  }
  
  private void configureForKfSampling() {
    mShooter.disableNominalClosedLoopVoltage();
  }
  
  private void configureForKfOnly() {
    mShooter.setNominalClosedLoopVoltage(12.0);
    mShooter.setProfile(F_ONLY_PROFILE);
    mShooter.setPIDF(0.0, 0.0, 0.0, mKfSamples.getAverage());
  }
  
  private void configureForMidstreamIncrease() {
    mRpmSetpoint += kMidstreamIncreaseRpm;
  }
  
  public void notifyShooting() {
    SmartDashboard.putBoolean("Notified Shooting", true);
    mIsShooting = true;
    mStartShootingTime = Timer.getFPGATimestamp();
  }
  
  public void notifyStoppedShooting() {
    SmartDashboard.putBoolean("Notified Shooting", false);
    mIsShooting = false;
    mStartShootingTime = Double.MAX_VALUE;
  }

  @Override
  public void reset() {
    mCurrentState = State.OPEN_LOOP;
    mNumKfSamples = 0;
    mKfSamples.reset();
    mOnTarget = false;
    mIsShooting = false;
    mStartShootingTime = Double.MAX_VALUE;
  }

  @Override
  public boolean onTarget() {
    return mOnTarget;
  }

  @Override
  public void loadProperties() {
    // TODO Auto-generated method stub
  }
}
