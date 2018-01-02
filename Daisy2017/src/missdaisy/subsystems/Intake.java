package missdaisy.subsystems;

import edu.wpi.first.wpilibj.Victor;
import missdaisy.Constants;

/**
 * The intake of the robot which acquires boulders from the ground
 *
 * @author Josh Sizer, AJN
 */
public final class Intake extends DaisySubsystem {

  private static Intake intakeInstance = null;
  private Victor mIntakeMotor;
  private double intakeStartTime;

  /**
   * Gets the instance of the intake. Used in order to never have more than
   * one intake object, ever.
   *
   * @return The one and only instance of the intake
   */
  public static Intake getInstance() {
    if (intakeInstance == null) {
      intakeInstance = new Intake();
    }
    return intakeInstance;
  }

  private Intake() {
    mIntakeMotor = new Victor(Constants.PWMs.INTAKE_MOTOR);
    mIntakeMotor.setInverted(false);
  }

  /**
   * Set the speed of the intake's motors in order to pick up a ball
   *
   * @param speed a value between 1.0 and -1.0, indication intake or spit
   */
  public void setIntakeSpeed(double speed) {
    mIntakeMotor.set(speed);
  }

  public void runIntake() {
    intakeStartTime = System.currentTimeMillis();
    setIntakeSpeed(1.0);
  }

  public void stopIntake() {
    // Delay the stopping of the intake to be 0.5 sec after being on. This is because the intake
    // is automated to be on when the drive is going in a specific direction
    if (System.currentTimeMillis() > (intakeStartTime + Constants.Properties.INTAKE_TURN_OFF_TIME_DELAY)){
      setIntakeSpeed(0.0);
    }
  }

  public void logToDashboard() {

  }

  @Override
  public synchronized void reset() {
    setIntakeSpeed(0.0);
  }
}
