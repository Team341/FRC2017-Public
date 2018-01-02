package missdaisy.autonomous;

/**
 * Represents a state that should stop running after a certain amount of time. The subclass should
 * call super.enter() if they override the enter() method
 *
 * @author Josh Sizer
 */
public class TimeoutState extends State {

  private long mStartTime;
  private long mTimeout;

  /**
   * @param aName The name of the state
   * @param timeout The number of milliseconds to wait until this state is done
   */
  public TimeoutState(String aName, int timeout) {
    super(aName);
    mTimeout = timeout;
  }

  /**
   * Sets the start time of the timeout action
   */
  @Override
  public void enter() {
    mStartTime = System.currentTimeMillis();
  }

  @Override
  public void running() {}

  /**
   * Returns true if it has been longer than the timeout
   */
  @Override
  public boolean isDone() {
    return System.currentTimeMillis() > mTimeout + mStartTime;
  }

  public int getTimeout() {
    return (int) mTimeout;
  }
}
