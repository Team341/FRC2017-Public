package missdaisy.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;

/**
 * The Gearipor, the subsystem that enables the robot to manipulate the gear
 *
 * @author AJN
 */
public final class Gearipor extends DaisySubsystem {

  private static Gearipor geariporInstance = new Gearipor();
  private CANTalon mGearMotor;
  private Solenoid mLoadGate;
  private Solenoid mScoreGate;
  private Solenoid mBallGate;
  private DigitalInput mHookSensor;

  private Servo mLoadGateLeftServo;
  private Servo mLoadGateRightServo;
  private Servo mScoreGateLeftServo;
  private Servo mScoreGateRightServo;

  private DigitalInput mForwardLimit;
  private DigitalInput mReverseLimit;

  private boolean mLoaderGateOpen = false;
  private boolean mScoringGateOpen = false;
  private boolean mBallGateOpen = false;

  /**
   * Gets the instance of the Gearipor. Used in order to never have more than one Gearipor object,
   * ever.
   *
   * @return The one and only instance of the Gearipor
   */
  public static Gearipor getInstance() {
    return geariporInstance;
  }

  private Gearipor() {
    mGearMotor = new CANTalon(Constants.CAN.GEARIPOR_TALONSRX_ID);
    mGearMotor.enableBrakeMode(true);
    mGearMotor.setInverted(false);
    mGearMotor.configPeakOutputVoltage(12.0, -12.0);
    mGearMotor.enableForwardSoftLimit(false);
    mGearMotor.enableReverseSoftLimit(false);

    mLoadGate = new Solenoid(Constants.Solenoids.GEARIPOR_LOAD_GATE);
    mScoreGate = new Solenoid(Constants.Solenoids.GEARIPOR_SCORE_GATE);
    mBallGate = new Solenoid(Constants.Solenoids.GEARIPOR_BALL_GATE);
    mLoadGateLeftServo = new Servo(Constants.SERVOs.GEARIPOR_UPPER_LEFT_SERVO);
    mLoadGateRightServo = new Servo(Constants.SERVOs.GEARIPOR_UPPER_RIGHT_SERVO);

    //mForwardLimit = new DigitalInput(Constants.DigitalInputs.GEARIPOR_FORWARD_LIMIT);
    //mReverseLimit = new DigitalInput(Constants.DigitalInputs.GEARIPOR_REVERSE_LIMIT);

    //mHookSensor = new DigitalInput(Constants.DigitalInputs.GEARIPOR_HOOK_SENSOR);
    
    if (Constants.DEBUG_MODE) {
      SmartDashboard.putNumber("LeftServoOpenValue", Constants.Properties.GEARIPOR_LEFT_LOADER_SERVO_OPEN);
      SmartDashboard.putNumber("LeftServoCloseValue", Constants.Properties.GEARIPOR_LEFT_LOADER_SERVO_CLOSED);
      SmartDashboard.putNumber("RightServoOpenValue", Constants.Properties.GEARIPOR_RIGHT_LOADER_SERVO_OPEN);
      SmartDashboard.putNumber("RightServoCloseValue", Constants.Properties.GEARIPOR_RIGHT_LOADER_SERVO_CLOSED);
    }
  }

  public void setPosition(double distFromCenter) {
    mGearMotor.changeControlMode(TalonControlMode.Position);
    mGearMotor.setSetpoint((distFromCenter / 4.0)); // 4 inches per rotation
    // SmartDashboard.putNumber("TurretError", mTurretMotor.getSetpoint());
  }

  /**
   * Used to set the position of the gearipor, side to side.
   * 
   * @param newPos The new position
   */
  public void resetPositon(double newPos) {
    mGearMotor.setPosition(newPos);
  }

  /**
   * Set the speed of the gearipor carriage
   * 
   * @param motorSpeed a value between -1.0 and 1.0
   */
  public void setSpeed(double motorSpeed) {
    //checkLimits(motorSpeed);
    mGearMotor.changeControlMode(TalonControlMode.PercentVbus);
    mGearMotor.set(motorSpeed);
  }

  /**
   * Puts the gear loader flap down.
   */
  public void openLoader() {
    mLoadGate.set(true);
    //if (!mLoaderGateOpen) {
    	mLoadGateLeftServo.setAngle(SmartDashboard.getNumber("LeftServoOpenValue", Constants.Properties.GEARIPOR_LEFT_LOADER_SERVO_OPEN));
    	mLoadGateRightServo.setAngle(SmartDashboard.getNumber("RightServoOpenValue", Constants.Properties.GEARIPOR_RIGHT_LOADER_SERVO_OPEN));
    	//mLoadGateLeftServo.setAngle(Constants.Properties.GEARIPOR_LEFT_LOADER_SERVO_OPEN);
    	//mLoadGateRightServo.setAngle(Constants.Properties.GEARIPOR_RIGHT_LOADER_SERVO_OPEN);
    	mLoaderGateOpen = true;
   // }
  }

  /**
   * Puts the gear loader flap up;
   */
  public void closeLoader() {
    mLoadGate.set(false);
    //if (mLoaderGateOpen) {
    	mLoadGateLeftServo.setAngle(SmartDashboard.getNumber("LeftServoCloseValue", Constants.Properties.GEARIPOR_LEFT_LOADER_SERVO_CLOSED));
    	mLoadGateRightServo.setAngle(SmartDashboard.getNumber("RightServoCloseValue", Constants.Properties.GEARIPOR_RIGHT_LOADER_SERVO_CLOSED));
    	//mLoadGateLeftServo.setAngle(Constants.Properties.GEARIPOR_LEFT_LOADER_SERVO_CLOSED);
    	//mLoadGateRightServo.setAngle(Constants.Properties.GEARIPOR_RIGHT_LOADER_SERVO_CLOSED);
    	mLoaderGateOpen = false;
    //}
  }

  /**
   * Release the gear
   */
  public void openScorer() {
    mScoreGate.set(true);
    mScoringGateOpen = true;
  }

  /**
   * Close the gear scoring flap
   */
  public void closeScorer() {
    mScoreGate.set(false);
    mScoringGateOpen = false;
  }

  /**
   * Set the ball gate flap to be down to allow balls to flow over the gear intake
   */
  public void openBallGate() {
    mBallGate.set(true);
    mBallGateOpen = true;
  }

  /**
   * Set the ball gate flap up to allow for intaking of gears
   */
  public void closeBallGate() {
    mBallGate.set(false);
    mBallGateOpen = false;
  }

  
  public synchronized double getGearPosition() {
    return mGearMotor.getPosition() * 4.0 / 1024.0;
  }

  public synchronized double getSetpoint() {
    return mGearMotor.getSetpoint() * 4.0 / 1024.0;
  }

  private synchronized double getError() {
    return getGearPosition() - getSetpoint();
  }

  public boolean isLoaderOpen() {
    return mLoaderGateOpen;
  }

  public boolean isScoringGateOpen() {
    return mScoringGateOpen;
  }

  public boolean seesHook() {
    return false;
  }

  // We are "OnTarget" if we are in position mode and close to the setpoint.
  public synchronized boolean isOnTarget() {
    return (mGearMotor.getControlMode() == CANTalon.TalonControlMode.Position
        && Math.abs(getError()) < Constants.Properties.TURRET_ON_TARGET_TOLERANCE);
  }

  @Override
  public synchronized void runOutputFilters() {
    //checkLimits(mGearMotor.getOutputVoltage() / 12.0);
	  /*
	if (isLoaderOpen()) {
	  mLoadGateLeftServo.setAngle(SmartDashboard.getNumber("LeftServoOpenValue", Constants.Properties.GEARIPOR_LEFT_LOADER_SERVO_OPEN));
	  mLoadGateRightServo.setAngle(SmartDashboard.getNumber("RightServoOpenValue", Constants.Properties.GEARIPOR_RIGHT_LOADER_SERVO_OPEN));
	} else {
	  mLoadGateLeftServo.setAngle(SmartDashboard.getNumber("LeftServoCloseValue", Constants.Properties.GEARIPOR_LEFT_LOADER_SERVO_CLOSED));
	  mLoadGateRightServo.setAngle(SmartDashboard.getNumber("RightServoCloseValue", Constants.Properties.GEARIPOR_RIGHT_LOADER_SERVO_CLOSED));
	}
	*/
  }

  /*
  public synchronized void checkLimits(double speed) {
    SmartDashboard.putNumber("GeariporLimitSpeed", speed);
    if ((speed > 0 && getForwardLimitSwitch()) || (speed < 0 && getReverseLimitSwitch())) {
      mGearMotor.changeControlMode(TalonControlMode.PercentVbus);
      mGearMotor.set(0.0);
    } else {
      mGearMotor.changeControlMode(TalonControlMode.PercentVbus);
      mGearMotor.set(speed);
    }
  }
  */

  @Override
  public synchronized void reset() {
    setSpeed(0.0);
  }

  public void logToDashboard() {
    if (Constants.DEBUG_MODE) {
      SmartDashboard.putBoolean("Gearipor_LoadingGateOpen", mLoaderGateOpen);
      SmartDashboard.putBoolean("Gearipor_ScoringGateOpen", mScoringGateOpen);
      SmartDashboard.putBoolean("Gearipor_BallGateOpen", mBallGateOpen);
    }
  }
}
