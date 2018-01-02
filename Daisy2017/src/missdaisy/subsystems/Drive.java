package missdaisy.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;
import missdaisy.loops.Navigation;
import missdaisy.utilities.AlphaFilter;

/**
 * The drive base of the robot. Has methods to set the speed of the left and right motors.
 *
 * @author Josh Sizer, AJN
 */
public final class Drive extends DaisySubsystem {

  private static Drive instance = new Drive();
  private static Navigation mNavigation;
  private Talon mLeftDriveMotor;
  private Talon mRightDriveMotor;
  private Solenoid mShifterPiston;
  private AlphaFilter mAlphaFilter;
  private boolean mUseAlphaFilter = false;
  private boolean mIsInHighGear = false;
  private int mShiftToHighGearCounter = 0;

  /**
   * Gets the instance of the drive base. Used in order to never have more than one drive base
   * object, ever.
   *
   * @return The one and only instance of the drive base
   */
  public static Drive getInstance() {
    return instance;
  }

  private Drive() {
    mNavigation = Navigation.getInstance();
    mLeftDriveMotor = new Talon(Constants.PWMs.DRIVE_LEFT_MOTOR);
    mRightDriveMotor = new Talon(Constants.PWMs.DRIVE_RIGHT_MOTOR);
    mRightDriveMotor.setInverted(true);
    mAlphaFilter = new AlphaFilter(Constants.Properties.DRIVE_ALPHA_FILTER_GAIN_LOW_SPEED);
    mShifterPiston = new Solenoid(Constants.Solenoids.DRIVE_SHIFTER);
  }

  /**
   * Set the speed of each motor individually. Useful for tank drive or any other case that each
   * motor must be set precisely and separate from each other
   *
   * @param leftMotorSpeed a double between -1.0 and 1.0, representing full forward or full reverse
   *        for the left motor.
   * @param rightMotorSpeed a double between -1.0 and 1.0, representing full forward or full reverse
   *        for the right motor.
   */
  public void setSpeed(double leftMotorSpeed, double rightMotorSpeed) {
    set(leftMotorSpeed, rightMotorSpeed);
  }

  /**
   * Set the speed and the turn of the robot, rather than each motor individually. Useful for arcade
   * drive.
   *
   * @param speed a double between -1.0 and 1.0, representing the full forward or full reverse.
   * @param turn a double between -1.0 and 1.0, representing turn anti-clockwise or clockwise.
   */
  public void setSpeedTurn(double speed, double turn) {
    if (mUseAlphaFilter) {
      SmartDashboard.putNumber("AlphaFilterBeforeSpeed", speed);
      speed = mAlphaFilter.calculate(speed);
      SmartDashboard.putNumber("AlphaFilterAfterSpeed", speed);
    }

    double leftMotorSpeed = speed + turn;
    double rightMotorSpeed = speed - turn;
    set(leftMotorSpeed, rightMotorSpeed);
  }

  /**
   * Tells the drive to use the alpha filter or not to limit acceleration
   */
  public void useAlphaFilter(boolean use) {
    mUseAlphaFilter = use;
  }

  /**
   * Set's the alpha gain for the alpha filter
   *
   * @param alpha The alpha gain
   */
  public void setAlpha(double alpha) {
    mAlphaFilter.setAlpha(alpha);
  }

  /**
   * This is to simplify other methods that set the motor speed. The arguments can only be values
   * from -1.0 to 1.0, a scaled number representing the output voltage of the speed controller (-12v
   * to 12v).
   */
  private void set(double leftMotorSpeed, double rightMotorSpeed) {
    mLeftDriveMotor.set(leftMotorSpeed);
    mRightDriveMotor.set(rightMotorSpeed);
  }

  /**
   * Set the drive to low gear
   */
  public void setLowGear() {
    mShifterPiston.set(true);

    if (mIsInHighGear) {
      mIsInHighGear = false;
    }
  }

  /**
   * Set the drive to high gear
   */
  public void setHighGear() {
    mShifterPiston.set(false);

    if (!mIsInHighGear) {
      mIsInHighGear = true;
      mShiftToHighGearCounter++;
    }
  }

  /**
   * Returns true for high gear and false for low gear
   * @return
   */
  public boolean isHighGear() {
    return mIsInHighGear;
  }

  /**
   * Checks if the drive base fits the conditions to shift into high gear
   * 
   * @return
   */
  public boolean autoShiftCheck() {
    // Check if the current speed of the robot is above the speed threshold
    return Math.abs(mNavigation
        .getAverageEncoderRate()) > Constants.Properties.DRIVE_HIGH_GEAR_SHIFT_SPEED_THRESHOLD;
  }

  /**
   * Set's the robot to a known state of stopped!
   */
  @Override
  public synchronized void reset() {
    set(0.0, 0.0);
  }

  @Override
  public void loadProperties() {
    // Nothing to load!
  }

  /**
   * Logs this subsystem's specific properties to smartdashboard.
   */
  public void logToDashBoard() {
    if (getCurrentController() != null) {
      SmartDashboard.putString("DriveCurrentController", getCurrentController().toString());
    } else {
      SmartDashboard.putString("DriveCurrentController", "OpenLoop");
    }
  }
}
