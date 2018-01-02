package missdaisy.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;

/**
 * The shooter of the robot which launches boulders into goals
 *
 * @author Josh Sizer
 */
public final class Shooter extends DaisySubsystem {

  private static Shooter instance = new Shooter();
  //private static DataLogger dataLogger = null;
  private CANTalon mWheelMotor;
  private Victor mBallConveyor;
  private Victor mBallAgitator;
  private double desiredSpeed;

  /**
   * Gets the instance of the shooter. Used in order to never have more than one shooter object,
   * ever.
   *
   * @return The one and only instance of the shooter
   */
  public static Shooter getInstance() {
    return instance;
  }

  private Shooter() {
    mWheelMotor = new CANTalon(Constants.CAN.SHOOTER_TALONSRX_ID);
    mWheelMotor.enableBrakeMode(false);
    mWheelMotor.setInverted(false);
    mWheelMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    mWheelMotor.reverseSensor(true);
    mWheelMotor.configPeakOutputVoltage(12.0, 0.0);
    mWheelMotor.configNominalOutputVoltage(+0.0f, -0.0f);
    mWheelMotor.setProfile(0);
    mWheelMotor.setF(0.024);
    mWheelMotor.setP(0.3);
    mWheelMotor.setI(0);
    mWheelMotor.setD(0.02);
    mWheelMotor.enableForwardSoftLimit(false);
    mWheelMotor.enableReverseSoftLimit(false);

    mBallConveyor = new Victor(Constants.PWMs.CONVEYOR_MOTOR);
    mBallConveyor.setInverted(false);

    mBallAgitator = new Victor(Constants.PWMs.AGITATOR_MOTOR);
    mBallAgitator.setInverted(false);
    
    loadProperties();
  }

  /**
   * Set the speed of the shooter's motor.
   *
   * @param motorSpeed A value between -1.0 and 1.0, representing full reverse and full forward.
   */
  public void setSpeed(double targetSpeed) {
    mWheelMotor.set(targetSpeed);
    desiredSpeed = targetSpeed;
  }

  /**
   * Gets the average shooter RPM
   *
   * @return the shooter's average RPM
   */
  public double getRPM() {
    return mWheelMotor.getSpeed();
  }

  public double getVoltage() {
    return mWheelMotor.getOutputVoltage();
  }

  public double getCurrent() {
    return mWheelMotor.getOutputCurrent();
  }

  public void enableSpeedControlMode(double rpms) {
    desiredSpeed = rpms;
    mWheelMotor.setSetpoint(rpms);
    mWheelMotor.changeControlMode(TalonControlMode.Speed);
  }

  public void disableSpeedControlMode() {
    mWheelMotor.changeControlMode(TalonControlMode.PercentVbus);
  }
  
  public boolean onTarget(){
    return desiredSpeed > 0.0 && (Math.abs(getRPM() - desiredSpeed) <=  Constants.Properties.SHOOTER_SPEED_TOLERANCE);
  }

  public void setConveyorSpeed(double speed) {
    mBallConveyor.set(speed);
  }

  public void setAgitatorSpeed(double speed) {
    mBallAgitator.set(speed);
  }

  public void feedFuel() {
    setConveyorSpeed(Constants.Properties.BALL_CONVEYOR_SPEED);
    setAgitatorSpeed(Constants.Properties.BALL_AGITATOR_SPEED);
    Intake.getInstance().setIntakeSpeed(0.5);
  }

  public void stopFeed() {
    setConveyorSpeed(0.0);
    setAgitatorSpeed(0.0);
  }

  public void unjamAgitator() {
    setAgitatorSpeed(Constants.Properties.BALL_AGITATOR_UNJAM_SPEED);
  }

  /**
   * Turns the shooter off and resets the moving average filter
   */
  @Override
  public synchronized void reset() {
    disableSpeedControlMode();
    stopFeed();
    setSpeed(0.0);
  }

  @Override
  public synchronized void loadProperties() {
    
  }

  public void logToDashboard() {
    SmartDashboard.putNumber("Shooter Measured RPM", getRPM());
    if (getCurrentController() != null)
      SmartDashboard.putString("ShooterCurrentController", getCurrentController().toString());
    else
      SmartDashboard.putString("ShooterCurrentController", "OpenLoop");
  }
}
