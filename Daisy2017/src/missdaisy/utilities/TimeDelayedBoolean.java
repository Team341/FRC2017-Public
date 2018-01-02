package missdaisy.utilities;

import edu.wpi.first.wpilibj.Timer;

/**
 * Returns true if called before the designated timeout, and if called after.
 * 
 * @author Joshua Sizer
 *
 */
public class TimeDelayedBoolean implements Filter {

  private double mStartTime;
  private double mTimeout;
  
  
  /**
   * Initializes this filter's start time to be when this object is created. Call 
   * reset() to reset the start time.
   * 
   * @param timeout The timeout, in seconds
   * @param type, which clock we are using
   */
  public TimeDelayedBoolean(double timeout) {
    mTimeout = timeout;
  }
  
  /**
   * Set a new timeout for this object. 
   * @param timeout The timeout in seconds
   */
  public void setTimeout(double timeout) {
    mTimeout = timeout;
  }
  
  /**
   * Returns true if called before the specified timeout, and false thereon after.
   * 
   * @param currentTime
   * @return
   */
  public boolean get() {
    if (Timer.getFPGATimestamp() - mStartTime > mTimeout) {
      return false;
    } 
    return true;
  }

  /**
   * Set's the start time of the filter to be when this function is called
   */
  @Override
  public void reset() {
    mStartTime = Timer.getFPGATimestamp();
  }

}
