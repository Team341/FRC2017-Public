package missdaisy.loops.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;
import missdaisy.loops.Navigation;
import missdaisy.subsystems.Drive;
import missdaisy.utilities.Trajectory;
import missdaisy.utilities.TrajectoryFollower;
import missdaisy.utilities.TrajectoryGenerator;

/**
 * Turns to a heading using trajectories for the left and right drive. By turning in opposite directions, we achieve a rotation
 * about the center of our robot. 
 * 
 * @author Joshua Sizer
 *
 */
public class TurnTrajectoryController implements Controller {
  
  private static TurnTrajectoryController mInstance = new TurnTrajectoryController();
  
  private Trajectory mLeftTraj;
  private Trajectory mRightTraj;
  private TrajectoryFollower mLeftTrajFollower;
  private TrajectoryFollower mRightTrajFollower;
  private Navigation mNav;
  private Drive mDrive;
  private double mLeftEncStartPos;
  private double mRightEncStartPos;
  private double mSetpoint;
  private static boolean mPIDFromSDB = true;
  
  private double kHighGearP = 1.0;
  private double kHighGearI = 0.0;
  private double kHighGearD = 10.0;
  
  private double kLowGearP = 1.0;
  private double kLowGearI = 0.0;
  private double kLowGearD = 10.0;
  
  // these are not physical maximums, but the trajectory maximums
  private static final double kLowGearMaximumVelocity = 60.0; // inches/s
  private static final double kHighGearMaximumVelocity = 60.0; // inches/s
  
  private static final double kLowGearMaximumAcceleration = 120.0;
  private static final double kHighGearMaximumAcceleration = 120.0;

  // this constraint is ignored for trapezoidal motion profiles (which we are using here)
  private static final double kMaxJerk = 500;
  
  // feet are converted to inches, the native unit of the encoders
  private double kHighGearV = 1.0 / 192.0; // (16 ft/s) guess
  private double kLowGearV = 1.0 / 72.0; // ( 6 ft/s) guess
  private double kHighGearA = 1.0 / 384.0; // (32 ft/s/s) guess
  private double kLowGearA = 1.0 / 528.0; // (44 ft/s/s) guess
  
  private static final double kWheelBaseWidth = 36.0; // inches guess
  private static final double kAngleTolerance = 0.5; // degrees
  
  public static TurnTrajectoryController getInstance() {
    return mInstance;
  }
  
  private TurnTrajectoryController() {
    mLeftTrajFollower = new TrajectoryFollower("Left Side Turn Trajectory Follower");
    mRightTrajFollower = new TrajectoryFollower("Right Side Turn Trajectory Follower");
    mNav = Navigation.getInstance();
    mDrive = Drive.getInstance();
    configureSmartDashboard();
  }
  
  /**
   * The absolute heading to turn to. To turn an angle relative to the robot's current angular position,
   * input Navigation.getCurrentHeadingInDegrees() + angle. Set should ideally be called once per move.
   * 
   * @param setpoint The absolute angle (in degrees) to turn to
   */
  public synchronized void set(double setpoint) {
    mSetpoint = setpoint;
    
    TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();
    config.dt = (double)(Constants.Properties.FAST_LOOP_TIMER_PERIOD) / (1000.0);
    config.max_jerk = kMaxJerk;
    
    if (mPIDFromSDB) {
      loadProperties();
    }
    
    if (mDrive.isHighGear()) {
      mLeftTrajFollower.configure(kHighGearP, kHighGearI, kHighGearD, kHighGearV, kHighGearA);
      mRightTrajFollower.configure(kHighGearP, kHighGearI, kHighGearD, kHighGearV, kHighGearA);
      config.max_acc = kHighGearMaximumAcceleration;
      config.max_vel = kHighGearMaximumVelocity;
    } else {
      mLeftTrajFollower.configure(kLowGearP, kLowGearI, kLowGearD, kLowGearV, kLowGearA);
      mRightTrajFollower.configure(kLowGearP, kLowGearI, kLowGearD, kLowGearV, kLowGearA);
      config.max_acc = kLowGearMaximumAcceleration;
      config.max_vel = kLowGearMaximumVelocity;
    }
    
    // negative error indicates we need to turn left
    // so the left profiles needs a negative distance
    // however, the generator does not support negative distances,
    // so we'll deal with this in the following part
   
    double error = Math.abs(calculateError(setpoint - mNav.getHeadingInDegrees(), 0, 360));
    double linearDistance = (error / 360.0) * Math.PI * kWheelBaseWidth;
    
    mLeftTraj = TrajectoryGenerator.generate(config, // the velocity, acceleration, and jerk limits
        TrajectoryGenerator.TrapezoidalStrategy, // type of motion profile
        mNav.getLeftEncoderRate(), // start velocity
        mNav.getHeadingInDegrees(), // start heading
        linearDistance, // distance
        0.0, // end velocity
        setpoint); // goal heading
    
    mRightTraj = TrajectoryGenerator.generate(config, // the velocity, acceleration, and jerk limits
        TrajectoryGenerator.TrapezoidalStrategy, // type of motino profile
        mNav.getRightEncoderRate(), // start velocity
        mNav.getHeadingInDegrees(), // start heading
        linearDistance, // distance
        0.0, // end velocity
        setpoint); // goal heading
    
    mLeftTrajFollower.setTrajectory(mLeftTraj);
    mRightTrajFollower.setTrajectory(mRightTraj);
    
    mLeftEncStartPos = mNav.getLeftEncoderDistance();
    mRightEncStartPos = mNav.getRightEncoderDistance();
  }

  /**
   * MUST CALL SET AT LEAST ONCE BEFORE RUN IS CALLED. Ie, you must call set before you set this controller
   * to be the Drive base's current controller.
   */
  @Override
  public void run() { 
    double leftOutput = 0.0;
    double rightOutput = 0.0;
    
    // if set is called while this loop is running, we want our current output to be calculated and sent to
    // the drive before we change our trajectories
    synchronized (this) {
      if (mLeftTrajFollower != null && mRightTrajFollower != null) {
        double error = calculateError(mSetpoint - mNav.getHeadingInDegrees(), 0, 360);
    
        // negative indicates left wheel needs to run in reverse
        if (error < 0) {
          leftOutput = -1.0 * mLeftTrajFollower.calculate(Math.abs(mNav.getLeftEncoderDistance() - mLeftEncStartPos));
          rightOutput = mRightTrajFollower.calculate(Math.abs(mNav.getRightEncoderDistance() - mRightEncStartPos));
        } else {
          leftOutput = mLeftTrajFollower.calculate(Math.abs(mNav.getLeftEncoderDistance() - mLeftEncStartPos));
          rightOutput = -1.0 * mRightTrajFollower.calculate(Math.abs(mNav.getRightEncoderDistance() - mRightEncStartPos));
        }
      }
    }
    
    if (onTarget()) {
      if (Math.abs(mNav.getHeadingInDegrees() - mSetpoint) > kAngleTolerance) {
        // whoops, we've completed a trajectory but we're still not where we want to be!
        set(mSetpoint); 
      } else {
        mDrive.setSpeed(0.0, 0.0);
      }
    } else {
      mDrive.setSpeed(leftOutput, rightOutput);
    }
  }
  
  /**
   * This will calculate the smaller error assuming the input is continuous
   * 
   * @param error The error was we normally calculate it
   * @param minimumInput The smallest input possible
   * @param maximumInput The largest input possible 
   * @return
   */
  private double calculateError(double error, double minimumInput, double maximumInput) {
    if (Math.abs(error) > (maximumInput - minimumInput) / 2) {
      if (error > 0) {
        error = error - maximumInput + minimumInput;
      } else {
        error = error + maximumInput - minimumInput;
      }
    }
    return error;
  }

  /**
   * Resets the starting encoder positions to be the current encoder positions
   */
  @Override
  public void reset() {
    mLeftEncStartPos = mNav.getLeftEncoderDistance();
    mRightEncStartPos = mNav.getRightEncoderDistance();
  }

  /**
   * Returns true if both the left and right trajectories are finished. They should finish at the same time.
   * 
   */
  @Override
  public boolean onTarget() {
    // TODO Auto-generated method stub
    return mLeftTrajFollower != null && mRightTrajFollower != null &&
        mLeftTrajFollower.isFinishedTrajectory() && mRightTrajFollower.isFinishedTrajectory();
  }

  @Override
  public void loadProperties() {
    updateLowGearPIDFromSmartDashboard();
    updateHighGearPIDFromSmartDashboard();
    updateLowGearFeedForward();
    updateHighGearFeedForward();
  }
  
  // TTJ: Turn Trajectory
  public static final String kHighGearPKey = "TTJ: HG P";
  public static final String kHighGearIKey = "TTJ: HG I";
  public static final String kHighGearDKey = "TTJ: HG D";
  public static final String kLowGearPKey = "TTJ: LG P";
  public static final String kLowGearIKey = "TTJ: LG I";
  public static final String kLowGearDKey = "TTJ: LG D";
  public static final String kLowGearVKey = "TTJ: LG kV";
  public static final String kHighGearVKey = "TTJ: HG kV";
  public static final String kLowGearAKey = "TTJ: LG kA";
  public static final String kHighGearAKey = "TTJ: HG kA";

  private void configureSmartDashboard() {
    if (!SmartDashboard.containsKey(kHighGearPKey)) {
      SmartDashboard.putNumber(kHighGearPKey, kHighGearP);
    }
    
    if (!SmartDashboard.containsKey(kHighGearIKey)) {
      SmartDashboard.putNumber(kHighGearIKey, kHighGearI);
    }
    
    if (!SmartDashboard.containsKey(kHighGearDKey)) {
      SmartDashboard.putNumber(kHighGearDKey, kHighGearD);
    }
    
    if (!SmartDashboard.containsKey(kLowGearPKey)) {
      SmartDashboard.putNumber(kLowGearPKey, kLowGearP);
    }
    
    if (!SmartDashboard.containsKey(kLowGearIKey)) {
      SmartDashboard.putNumber(kLowGearIKey, kLowGearI);
    }
    
    if (!SmartDashboard.containsKey(kLowGearDKey)) {
      SmartDashboard.putNumber(kLowGearDKey, kLowGearD);
    }
    
    if (!SmartDashboard.containsKey(kLowGearVKey)) {
      SmartDashboard.putNumber(kLowGearVKey, kLowGearV);
    }
    
    if (!SmartDashboard.containsKey(kHighGearVKey)) {
      SmartDashboard.putNumber(kHighGearVKey, kHighGearV);
    }
    
    if (!SmartDashboard.containsKey(kLowGearAKey)) {
      SmartDashboard.putNumber(kLowGearAKey, kLowGearA);
    }
    
    if (!SmartDashboard.containsKey(kHighGearAKey)) {
      SmartDashboard.putNumber(kHighGearAKey, kHighGearA);
    }
  }
  
  private void updateLowGearPIDFromSmartDashboard() {
    kLowGearP = SmartDashboard.getNumber(kLowGearPKey, kLowGearP);
    kLowGearI = SmartDashboard.getNumber(kLowGearIKey, kLowGearI);
    kLowGearD = SmartDashboard.getNumber(kLowGearDKey, kLowGearD);
  }
  
  private void updateHighGearPIDFromSmartDashboard() {
    kHighGearP = SmartDashboard.getNumber(kHighGearPKey, kHighGearP);
    kHighGearI = SmartDashboard.getNumber(kHighGearIKey, kHighGearI);
    kHighGearD = SmartDashboard.getNumber(kHighGearDKey, kHighGearD);
  }
  
  private void updateLowGearFeedForward() {
    kLowGearV = SmartDashboard.getNumber(kLowGearVKey, kLowGearV);
    kLowGearA = SmartDashboard.getNumber(kLowGearAKey, kLowGearA);
  }
  
  private void updateHighGearFeedForward() {
    kHighGearV = SmartDashboard.getNumber(kHighGearVKey, kHighGearV);
    kHighGearA = SmartDashboard.getNumber(kHighGearAKey, kHighGearA);
  }
}
