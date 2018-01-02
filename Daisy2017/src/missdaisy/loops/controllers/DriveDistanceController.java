package missdaisy.loops.controllers;

import missdaisy.Constants;
import missdaisy.fileio.PropertySet;
import missdaisy.loops.Navigation;
import missdaisy.loops.SynchronousPID;
import missdaisy.subsystems.Drive;

/**
 * Uses PID calculations to drive a certain distance in a straight line.
 */
// The class's inherited PID is the PID that controls distance.
public class DriveDistanceController extends SynchronousPID implements Controller {

  private static DriveDistanceController driveDistanceControllerInstance = null;
  private Drive mDrive;
  private Navigation mNavigation;
  private SynchronousPID mTurnPID; // This makes sure we drive in a straight line
  private double mGoalAngle = 0.0;
  private double mCurrentAngle;

  private double mDistanceTolerance;

  private boolean mDistanceOnTarget = false;

  /**
   * Gets the instance of the drive straight controller. Used in order to never have more than one
   * drive straight controller object, ever.
   *
   * @return The one and only instance of the drive straight controller
   */
  public static DriveDistanceController getInstance() {
    if (driveDistanceControllerInstance == null) {
      driveDistanceControllerInstance = new DriveDistanceController();
    }
    return driveDistanceControllerInstance;
  }

  private DriveDistanceController() {
    mTurnPID = new SynchronousPID();
    mTurnPID.setContinuous(true);
    mTurnPID.setInputRange(0, 360);
    mTurnPID.setOutputRange(-0.2, 0.2);
    mDrive = Drive.getInstance();
    mNavigation = Navigation.getInstance();
    loadProperties();
  }

  /**
   * Set the desired distance to travel and the maximum output of the PID, a number between 0.0 and
   * 1.0. The value between 0.0 and 1.0 indicates speeds between full stop and full power The
   * distance to travel should be positive or negative, the max power should be only positive
   *
   * @param distance The distance to travel, either backwards or forwards (negative or positive)
   * @param maxoutput The maximum output for the drive motors
   */
  public synchronized void setGoal(double distance, double speed) {
    super.reset();
    mTurnPID.reset();
    mNavigation.resetEncoders();
    mGoalAngle = mNavigation.getHeadingInDegrees();
    // makes sure that the maxVelocity is a positive number and not greater than 1.0
    if (speed < 0.0) {
      speed *= -1.0;
    }
    if (speed > 1.0) {
      speed = 1.0;
    }
    super.setOutputRange(-speed, speed);

    // we want the difference between where we want to be and where we are to be 0
    super.setSetpoint(mNavigation.getAverageEncoderDistance() + distance);
    mTurnPID.setSetpoint(mGoalAngle);
  }

  /**
   * Attempts to drive the robot a certain distance in a straight line
   */
  @Override
  public synchronized void run() {
    // what angle you're at right now
    mCurrentAngle = mNavigation.getHeadingInDegrees();
    // how far you've traveled since you've set your goal

    if (!onTarget()) {
      double speed = super.calculate(mNavigation.getAverageEncoderDistance());
      double turn = mCurrentAngle * mTurnPID.getP();
      mDrive.setSpeedTurn(speed, turn);
    } else {
      mDrive.setSpeedTurn(0.0, 0.0);
    }

    // checks to see if we are on target
    mDistanceOnTarget = super.onTarget(mDistanceTolerance);
  }

  /**
   * Resets all internal variables
   */
  @Override
  public synchronized void reset() {
    super.reset();
    mTurnPID.reset();
    mGoalAngle = 0.0;
    mCurrentAngle = mNavigation.getHeadingInDegrees();
  }

  /**
   * Returns true if the robot has driven the desired distance and its heading is the same as it
   * started
   */
  @Override
  public boolean onTarget() {
    return mDistanceOnTarget;
  }

  /**
   * Loads the PID multipliers and the angle and distance tolerances
   */
  @Override
  public void loadProperties() {
    PropertySet mPropertySet = PropertySet.getInstance();
    double kp = mPropertySet.getDoubleValue("distanceKp", 0.05);
    double ki = mPropertySet.getDoubleValue("distanceKi", 0.0);
    double kd = mPropertySet.getDoubleValue("distanceKd", 0.0);
    super.setPID(kp, ki, kd);
    kp = mPropertySet.getDoubleValue("angleKp", 0.01);
    mTurnPID.setPID(kp, 0.0, 0.0);
    mDistanceTolerance = mPropertySet.getDoubleValue("distanceTolerance",
        Constants.Properties.PID_DRIVE_DISTANCE_TOLERANCE);
  }

  @Override
  public String toString() {
    return "DriveDistanceController";
  }
}
