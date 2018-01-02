package missdaisy.loops.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;
import missdaisy.fileio.PropertySet;
import missdaisy.loops.Navigation;
import missdaisy.loops.SynchronousPID;
import missdaisy.subsystems.Drive;
import missdaisy.utilities.DaisyMath;

/**
 * Turns the robot in a closed loop by using PID calculations to calculate desired output to the
 * drive base motors.
 */
public class DriveTurnControllerRelative extends SynchronousPID implements Controller {

  private static DriveTurnControllerRelative driveTurnControllerInstance = null;
  private Drive mDrive;
  private Navigation mNavigation;

  // the allowed angle difference between the goal and actual robot angle
  private double currentAngle; // the angle the robot is at now
  private double mGoal; // the angle we want to turn to
  private double mMinMotorOutput = 0.2;
  private double mMaxMotorOutput = 0.5;
  private boolean mPIDFromSmartDashboard = true;
  private double mInitError;
  private final double kffw = mMaxMotorOutput / 180;

  /**
   * Gets the instance of the drive turn controller. Used in order to never have more than one drive
   * turn controller object, ever.
   *
   * @return The one and only instance of the drive turn controller
   */
  public static DriveTurnControllerRelative getInstance() {
    if (driveTurnControllerInstance == null) {
      driveTurnControllerInstance = new DriveTurnControllerRelative();
    }
    return driveTurnControllerInstance;
  }

  private DriveTurnControllerRelative() {
    mDrive = Drive.getInstance();
    mNavigation = Navigation.getInstance();
    loadProperties();
    // the angle of the robot is continuous
    setContinuous(true);
    super.setInputRange(0, 360);
    // super.setOutputRange(-mMaxMotorOutput, mMaxMotorOutput);
    SmartDashboard.putNumber("DTC_kP", 0.003);
    SmartDashboard.putNumber("DTC_kI", 0.000); // 0.02 seems to work for teleop
    SmartDashboard.putNumber("DTC_kD", -0.02);
  }

  /**
   * Set the angle for the robot to turn to
   *
   * @param angle the desired angle to turn to
   */
  public synchronized void setGoal(double angle) {
    super.reset();
    mGoal = DaisyMath.boundAngle0to360Degrees(mNavigation.getHeadingInDegrees() + angle);
    if (Constants.DEBUG_MODE) {
      SmartDashboard.putNumber("DTC_Goal", mGoal);
    }
    // the angle you want to be at will be considered 0.0.
    // the difference between where you are and where you want to be
    // is calculated, and you want that difference to be 0.0.
    setSetpoint(mGoal);
    loadProperties();
  }

  /**
   * Turns the robot to the desired angle specified by setGoal
   */
  @Override
  public synchronized void run() {
    // finds where you are now.
    currentAngle = mNavigation.getHeadingInDegrees();
    // calculate the output for drive motors, based on the difference between
    // where we are and where we want to be
    double cmd = super.calculate(currentAngle);
    double turn =
        Math.signum(cmd) * Math.max(Math.abs(cmd), Constants.Properties.PID_DRIVE_TURN_MIN_OUTPUT);
    // double turn = cmd;

    double error = DaisyMath.boundAngle0to360Degrees(mGoal - currentAngle);
    if (error > 180.0) {
      error -= 360.0;
    }

    // double turn = error * super.getP();
    if (Math.abs(error) < Constants.Properties.PID_DRIVE_ANGLE_TOLERANCE) {
      turn = 0.0;
    }
    
    if (Constants.DEBUG_MODE) {
      SmartDashboard.putNumber("DriveTurnError", error);
      SmartDashboard.putNumber("DTC_TurnCommand", turn);
      SmartDashboard.putBoolean("DTCOnTarget", onTarget());
    }
    /*
     * if (turn < -mMinMotorOutput) turn = -mMinMotorOutput; else if (turn > mMinMotorOutput) turn =
     * mMinMotorOutput;
     */

    mDrive.setSpeedTurn(0.0, turn);
  }

  public synchronized void setMinOutput(double min) {
    mMinMotorOutput = min;
  }

  /**
   * Resets the turn controller to have the goal be the angle the robot is currently at
   */
  @Override
  public synchronized void reset() {
    super.reset(); // resets the PID terms
    // the robots goal is where it is now if reset
    currentAngle = mNavigation.getHeadingInDegrees();
    setGoal(currentAngle);
  }

  /**
   * Returns true if the robot is at the goal angle, false if it is not
   */
  @Override
  public synchronized boolean onTarget() {
    double error = DaisyMath.boundAngle0to360Degrees(mGoal - currentAngle);
    if (error > 180.0) {
      error -= 360.0;
    }

    return Math.abs(error) < Constants.Properties.PID_DRIVE_ANGLE_TOLERANCE;
  }

  @Override
  public void loadProperties() {
    if (mPIDFromSmartDashboard) {
      double kp = SmartDashboard.getNumber("DTC_kP", 0.1);
      double ki = SmartDashboard.getNumber("DTC_kI", 0.000);
      double kd = SmartDashboard.getNumber("DTC_kD", -0.02);
      setPID(kp, ki, kd);
    } else {
      PropertySet mPropertySet = PropertySet.getInstance();
      double kp = mPropertySet.getDoubleValue("angleTurnKp", 0.1); // .09//0.11
      double ki = mPropertySet.getDoubleValue("angleTurnKi", 0.000);
      double kd = mPropertySet.getDoubleValue("angleTurnKd", -0.02);// -0.001//-0.06
      mMaxMotorOutput = mPropertySet.getDoubleValue("turnPIDMaxMotorOutput",
          Constants.Properties.PID_DRIVE_TURN_MAX_OUTPUT);
      mMinMotorOutput = mPropertySet.getDoubleValue("turnPIDMinMotorOutput",
          Constants.Properties.PID_DRIVE_TURN_MIN_OUTPUT);
      setPID(kp, ki, kd);
    }
  }

  @Override
  public String toString() {
    return "DriveTurnController";
  }
}
