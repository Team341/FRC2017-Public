package missdaisy.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;

/**
 * The Turret, the subsystem that enables the robot to turn the shooter towards the turret
 *
 * @author AJN
 */
public final class Turret extends DaisySubsystem {

  private static Turret turretInstance = new Turret();
  private CANTalon mTurretMotor;

  /**
   * Gets the instance of the turret. Used in order to never have more than one turret object, ever.
   *
   * @return The one and only instance of the turret
   */
  public static Turret getInstance() {
    return turretInstance;
  }

  private Turret() {
    mTurretMotor = new CANTalon(Constants.CAN.TURRET_TALONSRX_ID);
    mTurretMotor.enableBrakeMode(true);
    mTurretMotor.changeControlMode(TalonControlMode.Position);
    mTurretMotor.setInverted(false);
    mTurretMotor.reverseSensor(false);
    // mTurretMotor.enableLimitSwitch(true, true);
    // mTurretMotor.ConfigFwdLimitSwitchNormallyOpen(true);
    // mTurretMotor.ConfigRevLimitSwitchNormallyOpen(true);
    mTurretMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
    mTurretMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    mTurretMotor.setAllowableClosedLoopErr(20);
    mTurretMotor.enableForwardSoftLimit(true);
    mTurretMotor.enableReverseSoftLimit(true);
    mTurretMotor.setForwardSoftLimit(-1.0 * Constants.Properties.SOFT_MAX_TURRET_ANGLE
        / (360.0 * Constants.Properties.TURRET_ROTATIONS_PER_TICK));
    mTurretMotor.setReverseSoftLimit(-1.0 * Constants.Properties.SOFT_MIN_TURRET_ANGLE
        / (360.0 * Constants.Properties.TURRET_ROTATIONS_PER_TICK));

    mTurretMotor.setPID(1.5, 0.01, 0, 0, 100, 0, 0);
  }

  /**
   * Set the voltage to be applied to the turret motor. This is a value between -1.0 and 1.0 to 
   * represent a percentage of applied voltage
   * 
   * @param speed A value between -1.0 and 1.0
   */
  public void setSpeed(double speed) {
    mTurretMotor.changeControlMode(TalonControlMode.PercentVbus);
    mTurretMotor.set(speed);
  }
  
  public double wrapAngle(double angle) {
	if (angle < 0 && angle + 360 < Constants.Properties.SOFT_MAX_TURRET_ANGLE) {
	  angle += 360;
	} else {
		if (angle < Constants.Properties.SOFT_MIN_TURRET_ANGLE + 1) {
			angle = Constants.Properties.SOFT_MIN_TURRET_ANGLE + 1;
		}
	}
	return angle;
  }

  /**
   * Set the position of the turret in degrees.
   * This is relative to the zeroed position.
   * 
   * @param angle The angle in degrees.
   */
  public void setAngle(double angle) {
	//angle = wrapAngle(angle);
    if (Constants.DEBUG_MODE) {
      SmartDashboard.putNumber("Acutal Turret Wrap Angle", angle);
    }
    mTurretMotor.changeControlMode(TalonControlMode.Position);
    mTurretMotor
        .setSetpoint((-1.0 * angle / (360.0 * Constants.Properties.TURRET_ROTATIONS_PER_TICK)));
  }

  /**
   * Get the turret angle in degrees
   * 
   * @return The curretent turret angle
   */
  public double getTurretAngle() {
    return -1.0 * mTurretMotor.getPosition() * 360.0
        * Constants.Properties.TURRET_ROTATIONS_PER_TICK;
  }

  /**
   * Get the angular velocity of the turret
   * 
   * @return The angular velocity of the turret
   */
  public double getTurretAngleRate() {
    return mTurretMotor.getEncVelocity() * 360.0 * Constants.Properties.TURRET_ROTATIONS_PER_TICK;
  }

  /**
   * @return The state of the foward limit switch
   */
  public synchronized boolean getForwardLimitSwitch() {
    return mTurretMotor.isFwdLimitSwitchClosed();
  }

  /**
   * @return The state of the reverse limit switch
   */
  public synchronized boolean getReverseLimitSwitch() {
    return mTurretMotor.isRevLimitSwitchClosed();
  }

  /**
   * The currently set setpoint, in degrees
   * @return
   */
  public synchronized double getSetpoint() {
    return mTurretMotor.getSetpoint() * Constants.Properties.TURRET_ROTATIONS_PER_TICK * 360.0;
  }

  /**
   * The difference between the current turret angle and the setpoint
   * @return
   */
  private synchronized double getError() {
    return getTurretAngle() - getSetpoint();
  }

  /**
   * This will return true if we are within the angular tolerance specified in the constants file
   * and we are in position control mode
   * 
   * @return True for on target, false otherwise
   */
  public synchronized boolean isOnTarget() {
    return (mTurretMotor.getControlMode() == CANTalon.TalonControlMode.Position
        && Math.abs(getError()) < Constants.Properties.TURRET_ON_TARGET_TOLERANCE);
  }

  /**
   * Constantly checks foward and reverse hard limit switches, in order to put the turret
   * into a known position
   */
  @Override
  public synchronized void runInputFilters() {
    if (mTurretMotor.isFwdLimitSwitchClosed()) {
      mTurretMotor.setPosition(Constants.Properties.SOFT_MAX_TURRET_ANGLE * Math.PI / 180.0
          / (2 * Math.PI * Constants.Properties.TURRET_ROTATIONS_PER_TICK));
    } else if (mTurretMotor.isRevLimitSwitchClosed()) {
      mTurretMotor.setPosition(Constants.Properties.SOFT_MIN_TURRET_ANGLE * Math.PI / 180.0
          / (2 * Math.PI * Constants.Properties.TURRET_ROTATIONS_PER_TICK));
    }
  }

  /**
   * Set the control mode to percent voltage and sets the output voltage to 0
   */
  @Override
  public synchronized void reset() {
    setSpeed(0.0);
  }

  /**
   * Sets the current turret angle to be 0 degrees. This will affect the soft foward and reverse
   * limit switches.
   */
  public void resetAngle() {
    mTurretMotor.setPosition(0);
  }

  public void logToDashboard() {
    SmartDashboard.putNumber("TurretAngle", getTurretAngle());

    if (getCurrentController() != null)
      SmartDashboard.putString("TurretCurrentController", getCurrentController().toString());
    else
      SmartDashboard.putString("TurretCurrentController", "OpenLoop");
  }
}
