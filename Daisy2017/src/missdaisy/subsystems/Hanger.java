package missdaisy.subsystems;

import edu.wpi.first.wpilibj.Victor;
import missdaisy.Constants;

/**
 * The hanger, the subsystem that enables the robot to hang in the end game
 *
 * @author Josh Sizer, AJN
 */
public final class Hanger extends DaisySubsystem {

  private static Hanger hangerInstance = null;
  private Victor mHangerMotor;
  private Victor mHangerMotor2;

  /**
   * Gets the instance of the hanger. Used in order to never have more than one hanger object, ever.
   *
   * @return The one and only instance of the hanger
   */
  public static Hanger getInstance() {
    if (hangerInstance == null) {
      hangerInstance = new Hanger();
    }
    return hangerInstance;
  }

  private Hanger() {
    mHangerMotor = new Victor(Constants.PWMs.HANGER_MOTOR);
    mHangerMotor.setInverted(true);
    
    mHangerMotor2 = new Victor(Constants.PWMs.HANGER_MOTOR_2);
    mHangerMotor2.setInverted(false);
  }

  public void setSpeed(double motorSpeed) {
    mHangerMotor.set(motorSpeed);
    mHangerMotor2.set(motorSpeed);
  }
  
  public void winch(boolean reverse) {
	if (reverse) {
	  winchDown();
	} else {
	  winchUp();
	}
  }

  public void winchUp() {
    setSpeed(1.0);
  }

  public void winchDown() {
    setSpeed(-1.0);
  }

  public void holdPosition() {
    setSpeed(0.0);
  }


  @Override
  public synchronized void reset() {
    setSpeed(0.0);
  }

  public void logToDashboard() {

  }
}
