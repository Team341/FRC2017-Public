package missdaisy.loops.controllers;

import missdaisy.Constants;
import missdaisy.Vision;
import missdaisy.subsystems.Shooter;

/**
 * A controller that sets the RPM according the value supplied by the vision processing, ie by how
 * far away from the target the robot is.
 */

public class ShooterSpeedController implements Controller {

  private static ShooterSpeedController autoAimShooterControllerInstance = null;
  private Shooter mShooter;
  private double RPMSOffset = 0.0;

  /**
   * Gets the instance of the auto-aim shooter speed controller. Used in order to never have more
   * than one auto-aim shooter speed controller object, ever.
   *
   * @return The one and only instance of the auto-aim shooter speed controller
   */
  public static ShooterSpeedController getInstance() {
    if (autoAimShooterControllerInstance == null) {
      autoAimShooterControllerInstance = new ShooterSpeedController();
    }
    return autoAimShooterControllerInstance;
  }

  private ShooterSpeedController() {
    mShooter = Shooter.getInstance();
    loadProperties();
  }
  
  public synchronized void setRPMSOffest(double offset) {
	this.RPMSOffset = offset;
  }

  /**
   * If the robot sees the target, the shooter will ramp up to the specified RPM based on distance
   * from the target. Otherwise, it will ramp up to the default RPM
   */
  @Override
  public synchronized void run() {
    if (Vision.getInstance().seesTarget()) {
      mShooter.enableSpeedControlMode(Vision.getInstance().getRPM() + RPMSOffset);
    } else {
      mShooter.enableSpeedControlMode(Constants.Properties.DEFAULT_SHOOTER_RPM + RPMSOffset);
    }
  }

  /**
   * Sets the shooter motor off
   */
  @Override
  public synchronized void reset() {
  }

  /**
   * Returns true if the shooter RPM is within an acceptable range of the goal RPM
   */
  @Override
  public synchronized boolean onTarget() {
    return mShooter.onTarget();
  }

  @Override
  public void loadProperties() {
  }

  @Override
  public String toString() {
    return "AutoAimShooterController";
  }
}
