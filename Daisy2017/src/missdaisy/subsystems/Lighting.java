package missdaisy.subsystems;

//import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Solenoid;
import missdaisy.Constants;

/**
 * This subsystem controls the lights of the robot
 *
 * @author AJN
 */
public final class Lighting extends DaisySubsystem {

  private static Lighting lightInstance = null;

  private DigitalOutput hoodLight;
  private DigitalOutput gearLight;
  private DigitalOutput rainbowLights;
  private Solenoid mAllianceRedLight;
  private Solenoid mAllianceBlueLight;
  
  private boolean lightshowEnabled = false;

  /**
   * Gets the instance of the Lighting. Used in order to never have more than
   * one Lighting object, ever.
   *
   * @return The one and only instance of the Lighting
   */
  public static Lighting getInstance() {
    if (lightInstance == null) {
      lightInstance = new Lighting();
    }
    return lightInstance;
  }

  private Lighting() {
    hoodLight = new DigitalOutput(Constants.DigitalOutputs.SHOOTER_ON_TARGET_SIGNAL);
    gearLight = new DigitalOutput(Constants.DigitalOutputs.GEAR_ON_TARGET_SIGNAL);
    rainbowLights = new DigitalOutput(Constants.DigitalOutputs.RUN_LIGHTSHOW_SIGNAL);
    
    mAllianceRedLight = new Solenoid(Constants.Solenoids.ALLIANCE_RED_LED);
    mAllianceBlueLight = new Solenoid(Constants.Solenoids.ALLIANCE_BLUE_LED);
  }
  
  /**
   * Turns the underglow LEDs red, blue or purple if enabled.
   *
   * @param on True for on, false for off
   */
  public void setAllianceLightRed() {
    mAllianceRedLight.set(true);
    mAllianceBlueLight.set(false);
  }

  public void setAllianceLightBlue() {
    mAllianceRedLight.set(false);
    mAllianceBlueLight.set(true);
  }
  
  public void setAllianceLightPurple() {
    mAllianceRedLight.set(true);
    mAllianceBlueLight.set(true);
  }
  
  public void setAllianceLightOff() {
    mAllianceRedLight.set(false);
    mAllianceBlueLight.set(false);
  }

  public void startLightShow() {
    if (!lightshowEnabled) {
      lightshowEnabled = true;
      //mRunLightShow.set(true);
    }
  }

  public void endLightShow() {
    if (lightshowEnabled) {
      lightshowEnabled = false;
      //mRunLightShow.set(false);
    }
  }
  
  /**
   * If this is set, rainbow lights will go!
   * @param on
   */
  public void setRainbowMode(boolean on) {
	rainbowLights.set(on);
	hoodLight.set(false);
	gearLight.set(false);
	//mRunLightShow.set(on);
  }
  
  /**
   *  If this is called, rainbow lights will turn off and the hood will be green
   *  for true or off for false
   */
  public void setHoodLightOn(boolean on) {
	rainbowLights.set(false);
	hoodLight.set(on);
	//mShooterOnTarget.set(on);
  }
  
  public void setGearLightOn(boolean on) {
	rainbowLights.set(false);
	gearLight.set(on);
  }
    
  @Override
  public synchronized void reset() {
    
  }

  public void logToDashboard() {

  }
}
