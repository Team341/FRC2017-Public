package missdaisy.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;
import missdaisy.utilities.MovingAverageFilter;

/**
 * The shooter of the robot which launches boulders into goals
 *
 * @author Josh Sizer
 */
public final class Shooter extends DaisySubsystem {

  private static Shooter instance = new Shooter();
  //private static DataLogger dataLogger = null;
  private CANTalon mWheelMotor;
  private CANTalon mSlave;
  private Victor mBallConveyor;
  private Victor mBallAgitator;
  private double desiredSpeed;
  private MovingAverageFilter kFSamples;
  
  private boolean useFeedfowardSampling = false;
  private boolean useRPMSetpointIncrease = false;
  private double rpmOffset = 325;
  private double kIncreaseRPMTime = 3; // seconds
  private double kSamplePeriod = 0.5; // seconds
  private double shootingStartTime = 0;
  private double flyWheelOnTargetStartTime = 0;
  private boolean useOnlyKf = false;
  private double kFlywheelOff = 500; // rpm
  private double kFSamplesSize = 0;
  private double kMinShooterRpm = 2000;

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
    mWheelMotor.setInverted(true);
    mWheelMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    mWheelMotor.reverseSensor(false);
    mWheelMotor.configPeakOutputVoltage(0.0, -12.0);
    mWheelMotor.configNominalOutputVoltage(+0.0f, -0.0f);
    mWheelMotor.setProfile(0);
    mWheelMotor.setF(0.013);
    mWheelMotor.setP(0.45);
    mWheelMotor.setI(0.000000000000);
    mWheelMotor.setD(0.00000);
    mWheelMotor.setIZone(0);
    mWheelMotor.setNominalClosedLoopVoltage(12.0);
    
    //mWheelMotor.setPID(0, 0, 0, 0.021, 0, 0, 1);
    
    mWheelMotor.enableForwardSoftLimit(false);
    mWheelMotor.enableReverseSoftLimit(false);
    
    mSlave = new CANTalon(Constants.CAN.SHOOTER_SLAVE_TALONSRX_ID);
    mSlave.changeControlMode(TalonControlMode.Follower);
    mSlave.set(Constants.CAN.SHOOTER_TALONSRX_ID);

    mBallConveyor = new Victor(Constants.PWMs.CONVEYOR_MOTOR);
    mBallConveyor.setInverted(false);

    mBallAgitator = new Victor(Constants.PWMs.AGITATOR_MOTOR);
    mBallAgitator.setInverted(false);
    
    kFSamples = new MovingAverageFilter(25);
    
    if (!SmartDashboard.containsKey("Shooter kP")) {
      SmartDashboard.putNumber("Shooter kP", 0.45);
    }
    if (!SmartDashboard.containsKey("Shooter kI")) {
      SmartDashboard.putNumber("Shooter kI", 0.0);
    }
    if (!SmartDashboard.containsKey("Shooter kD")) {
      SmartDashboard.putNumber("Shooter kD", 0.00025);
    }
    if (!SmartDashboard.containsKey("Shooter kF")) {
      SmartDashboard.putNumber("Shooter kF", 0.013);
    }
    if (!SmartDashboard.containsKey("Shooter increase delay")) {
      SmartDashboard.putNumber("Shooter increase delay", 2.0);
    }
    if (!SmartDashboard.containsKey("Shooter increase")){
      SmartDashboard.putNumber("Shooter increase", 500);
    }
    
    if (!SmartDashboard.containsKey("Run Intake While Shooting")) {
      SmartDashboard.putBoolean("Run Intake While Shooting", true);
    }
    
    
    loadProperties();
  }

  /**
   * Set the speed of the shooter's motor.
   *
   * @param motorSpeed A value between -1.0 and 1.0, representing full reverse and full forward.
   */
  public void setPercVoltage(double targetSpeed) {
    mWheelMotor.changeControlMode(TalonControlMode.PercentVbus);
    mWheelMotor.set(targetSpeed);
    desiredSpeed = targetSpeed;
  }

  /**
   * Gets the average shooter RPM
   *
   * @return the shooter's average RPM
   */
  public double getRPM() {
    return Math.abs(mWheelMotor.getSpeed());
  }
  
  public double getKf() {
    return mWheelMotor.getF();
  }
  
  public void setPIDF(double p, double i, double d, double f) {
    mWheelMotor.setP(p);
    mWheelMotor.setI(i);
    mWheelMotor.setD(d);
    mWheelMotor.setF(f);
  }

  public double getVoltage() {
    return mWheelMotor.getOutputVoltage();
  }
  
  public void disableNominalClosedLoopVoltage() {
    mWheelMotor.DisableNominalClosedLoopVoltage();
  }
  
  public void setNominalClosedLoopVoltage(double voltage) {
    mWheelMotor.setNominalClosedLoopVoltage(voltage);
  }
  
  public double getInputVoltage() {
    return mWheelMotor.getBusVoltage();
  }

  public double getCurrent() {
    return mWheelMotor.getOutputCurrent();
  }
  
  public void setRPMIncreaseDelay(double delay) {
    this.kIncreaseRPMTime = delay;
  }
  
  public void setRPMIncrease(double rpm) {
    this.rpmOffset = rpm;
  }

  public void setRpm(double rpms) {
    double finalRPM = 0;
    double curTime = Timer.getFPGATimestamp();
    desiredSpeed = rpms;
    mWheelMotor.changeControlMode(TalonControlMode.Speed);

    if (useRPMSetpointIncrease) {
      // we've been shooting for a little while
      if (curTime - shootingStartTime > kIncreaseRPMTime && shootingStartTime != 0) {
        if (mWheelMotor.getSpeed() - rpms < 300) {
          kFSamples.setInput(calculateKf(this.getRPM(), Math.abs(mWheelMotor.getOutputVoltage()/12.0)));
          kFSamples.run();
          kFSamplesSize++;
          SmartDashboard.putNumber("Calculated kF", kFSamples.getAverage());
        } else {
          kFSamplesSize = 0;
        }
        if (kFSamplesSize > 50) {
          //mWheelMotor.setF(kFSamples.getAverage());
          kFSamplesSize = 0;
        }
        finalRPM = rpms + rpmOffset;
        desiredSpeed = finalRPM;
        mWheelMotor.set(finalRPM);
      } else {
        finalRPM = rpms;
      }
      mWheelMotor.set(finalRPM);
    } else if (useFeedfowardSampling) {
      if (mWheelMotor.getSpeed() - rpms < Constants.Properties.SHOOTER_SPEED_TOLERANCE && !useOnlyKf) {
        if (flyWheelOnTargetStartTime == 0) {
          flyWheelOnTargetStartTime = curTime;
        }
        kFSamples.setInput(calculateKf(this.getRPM(), mWheelMotor.getOutputVoltage()/12.0));
        kFSamples.run();
      } else if (mWheelMotor.getSpeed() - rpms > Constants.Properties.SHOOTER_SPEED_TOLERANCE && useOnlyKf) {
        kFSamples.setInput(calculateKf(this.getRPM(), mWheelMotor.getOutputVoltage()/mWheelMotor.getBusVoltage()));
        kFSamples.run();
      }
   
      if (flyWheelOnTargetStartTime != 0 && curTime - flyWheelOnTargetStartTime > kSamplePeriod) {
        mWheelMotor.setProfile(1);
        mWheelMotor.setF(kFSamples.getAverage());
        useOnlyKf = true;
      }
      mWheelMotor.set(rpms);
    } else {
      mWheelMotor.set(rpms);
    }
    SmartDashboard.putNumber("RPMSetpoint", finalRPM);
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
    if (getRPM() > kMinShooterRpm) {
      setConveyorSpeed(Constants.Properties.BALL_CONVEYOR_SPEED);
      setAgitatorSpeed(Constants.Properties.BALL_AGITATOR_SPEED);
      if (shootingStartTime == 0) {
        shootingStartTime = Timer.getFPGATimestamp();
      }
    }
    if (SmartDashboard.getBoolean("Run Intake While Shooting", true)) {
      Intake.getInstance().setIntakeSpeed(1.0);
    }
  }

  public void stopFeed() {
    setConveyorSpeed(0.0);
    setAgitatorSpeed(0.0);
  }

  public void unjamAgitator() {
    setAgitatorSpeed(Constants.Properties.BALL_AGITATOR_UNJAM_SPEED);
  }
  
  public double calculateKf(double rpm, double percOutput) {
    double natUnitPer100ms = rpm * (4096/10/60);
    return (percOutput * 1023) / natUnitPer100ms;
  }

  /**
   * Turns the shooter off and resets the moving average filter
   */
  @Override
  public synchronized void reset() {
    setPercVoltage(0.0);
    stopFeed();
    setPercVoltage(0.0);
    this.useOnlyKf = false;
    this.shootingStartTime = 0.0;
    this.kFSamples.reset();
    this.flyWheelOnTargetStartTime = 0;
  }
  
  /** 
   * This is where we'll keep track of the measured state of our shooter.
   * 
   */
  @Override
  public void runInputFilters() {
    //double p = SmartDashboard.getNumber("Shooter kP", 0.000125);
    //double i = SmartDashboard.getNumber("Shooter kI", 0.0);
    //double d = SmartDashboard.getNumber("Shooter kD", 0.00025);
    //double f = SmartDashboard.getNumber("Shooter kF", 0.019);
    //this.setPIDF(p, i, d, f);
    //this.setRPMIncreaseDelay(SmartDashboard.getNumber("Shooter increase delay", 2));
    //this.setRPMIncrease(SmartDashboard.getNumber("Shooter increase", 300));
  }
  
  public void setProfile(int profile) {
    mWheelMotor.setProfile(profile);
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
