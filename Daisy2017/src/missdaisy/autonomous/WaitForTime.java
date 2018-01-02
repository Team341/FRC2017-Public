package missdaisy.autonomous;

/**
 * State to wait for a specified number of milliseconds.
 *
 * @author Jared341
 */
public class WaitForTime extends TimeoutState {

  long mTimeout;

  public WaitForTime(int aMilliseconds) {
    super("WaitForTime", aMilliseconds);
  }

  @Override
  public void enter() {
    super.enter();
  }

  @Override
  public void running() {}
}
