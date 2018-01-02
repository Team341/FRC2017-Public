package missdaisy.loops;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;
import missdaisy.tracking.ReferenceFrame;
import missdaisy.utilities.DaisyMath;

/**
 * The representation of the location, heading (angle), speed, and distance
 * (traveled) of the robot
 *
 * @author jrussell
 */
public class Navigation {

  private static Navigation navigationInstance = new Navigation();
  
  // Sensors
  private Encoder mLeftDriveEncoder;
  private Encoder mRightDriveEncoder;
  private AHRS mGyro;

  // Navigational state
  private State curNavState;
  private double theta0;
  private double thetaLast = 0.0;
  private double leftEncoderLast = 0.0;
  private double rightEncoderLast = 0.0;

  // Collision detection parameters
  private boolean forwardCollisionDetected = false;
  private boolean sideCollisionDetected = false;
  private double last_world_linear_accel_x;
  private double last_world_linear_accel_y;

  public static class State {

    public double x;                // in
    public double y;                // in
    public double heading;          // rad
    public ReferenceFrame refFrame;
    public double velocity;         // in/sec
    public double headingRate;      // rad/sec

    public State(double x, double y, double heading, double velocity, double headingRate) {
      this.x = x;
      this.y = y;
      this.heading = heading;
      this.velocity = velocity;
      this.headingRate = headingRate;
      updateState();
    }
    
    public void updateState(){
    	refFrame = new ReferenceFrame(this.x, this.y, this.heading);
    }
  }

  public static Navigation getInstance() {
    return navigationInstance;
  }

  private Navigation() {
    mLeftDriveEncoder = new Encoder(Constants.DigitalInputs.DRIVE_LEFT_ENCODER_1,
        Constants.DigitalInputs.DRIVE_LEFT_ENCODER_2);
    mRightDriveEncoder = new Encoder(Constants.DigitalInputs.DRIVE_RIGHT_ENCODER_1,
        Constants.DigitalInputs.DRIVE_RIGHT_ENCODER_2);
    mLeftDriveEncoder.setDistancePerPulse(Constants.Properties.DRIVE_DISTANCE_PER_PULSE);
    mRightDriveEncoder.setDistancePerPulse(Constants.Properties.DRIVE_DISTANCE_PER_PULSE);

    try {
      mGyro = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    curNavState = new State(0.0, 0.0, 0.0, 0.0, 0.0);
  }

  /**
   * Integrates robot position over time
   */
  public synchronized void run() {
    // Read sensors
    double left = mLeftDriveEncoder.getDistance();
    double right = mRightDriveEncoder.getDistance();
    double yaw = DaisyMath.boundAngle0to360Degrees(mGyro.getYaw() + theta0);
    // the distance the drive has gone since last cycle
    double distance = ((left + right) - (leftEncoderLast + rightEncoderLast)) / 2.0;
    double thetaRad = Math.toRadians(yaw);
    // calculate the current x + y coordinates, based on the distance
    // traveled the angle turned

    curNavState.x += distance * Math.cos(thetaRad);
    curNavState.y += distance * Math.sin(thetaRad);
    curNavState.heading = Math.toRadians(yaw);
    curNavState.velocity = this.getAverageEncoderRate();
    curNavState.headingRate = mGyro.getRate();
    curNavState.updateState();
    leftEncoderLast = left;
    rightEncoderLast = right;
    thetaLast = yaw;
    
    // Check for collisions
    double curr_world_linear_accel_x = mGyro.getWorldLinearAccelX();
    double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
    last_world_linear_accel_x = curr_world_linear_accel_x;
    double curr_world_linear_accel_y = mGyro.getWorldLinearAccelY();
    double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
    last_world_linear_accel_y = curr_world_linear_accel_y;

    if (Math.abs(currentJerkX) > Constants.Properties.DRIVE_COLLISION_DELTA_G_THRESHOLD) {
      forwardCollisionDetected = true;
    } else {
      forwardCollisionDetected = false;
    }

    if (Math.abs(currentJerkY) > Constants.Properties.DRIVE_COLLISION_DELTA_G_THRESHOLD) {
      sideCollisionDetected = true;
    } else {
      sideCollisionDetected = false;
    }

  }

  public State getNavigationState() {
    return this.curNavState;
  }

  public boolean hasForwardCollision() {
    return forwardCollisionDetected;
  }

  public boolean hasSideCollision() {
    return sideCollisionDetected;
  }

  /**
   * Reset your current location to specified coordinates
   *
   * @param x desired x coordinate
   * @param y desired y coordinate
   * @param theta desired robot heading (angle)
   */
  public synchronized void resetRobotPosition(double x, double y, double theta) {
    this.curNavState.x = x;
    this.curNavState.y = y;
    theta0 = theta;
    thetaLast = theta;
    mLeftDriveEncoder.reset();
    mRightDriveEncoder.reset();
    mGyro.reset();
  }

  public void resetEncoders() {
    mLeftDriveEncoder.reset();
    mRightDriveEncoder.reset();
  }

  /**
   * @return the current x coordinate of the robot
   */
  public synchronized double getXinInches() {
    return curNavState.x;
  }

  /**
   * @return the current y coordinate of the robot
   */
  public synchronized double getYinInches() {
    return curNavState.y;
  }

  /**
   * @return the current angle of the robot bound 0 to 360 degrees
   */
  public double getHeadingInDegrees() {
    return DaisyMath.boundAngle0to360Degrees(thetaLast + theta0);
  }

  public double getHeadingInRadians() {
    return getHeadingInDegrees() * Math.PI / 180.0;
  }

  public double getLeftEncoderDistance() {
    return mLeftDriveEncoder.getDistance();
  }

  public double getRightEncoderDistance() {
    return -1.0 * mRightDriveEncoder.getDistance();
  }

  public int getLeftEncoderCounts() {
    return mLeftDriveEncoder.get();
  }

  public int getRightEncoderCounts() {
    return mRightDriveEncoder.get();
  }

  public double getLeftEncoderRate() {
    return mLeftDriveEncoder.getRate();
  }

  public double getRightEncoderRate() {
	return -1.0 * mRightDriveEncoder.getRate();
  }

  /**
   * Uses the distance the robot moves per one shaft rotation to calculate the
   * speed of the robot
   *
   * @return the speed in units/sec, defined by the setDistancePerpulse.
   */
  public double getAverageEncoderRate() {
    double average = (getLeftEncoderRate() + getRightEncoderRate()) / 2;
    return average;
  }

  /**
   * Uses the distance the robot moves per one shaft rotation and the number
   * of rotations to calculate distance traveled.
   *
   * @return The distance traveled in units, defined by the setDistancePerPulse
   */
  public double getAverageEncoderDistance() {
    double average = (getLeftEncoderDistance() + getRightEncoderDistance()) / 2;
    return average;
  }

  public void logToDashboard() {
    SmartDashboard.putNumber("heading", getHeadingInDegrees());
    SmartDashboard.putNumber("LeftEncoderRate", getLeftEncoderRate());
    SmartDashboard.putNumber("RightEncoderRate", getRightEncoderRate());
    SmartDashboard.putNumber("LeftEncoderDistance", getLeftEncoderDistance());
    SmartDashboard.putNumber("RightEncoderDistance", getRightEncoderDistance());
    SmartDashboard.putBoolean("HasSideCollision", hasSideCollision());
    SmartDashboard.putBoolean("HasForwardCollision", hasForwardCollision());
  }
}
