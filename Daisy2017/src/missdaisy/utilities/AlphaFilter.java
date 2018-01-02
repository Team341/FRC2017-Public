package missdaisy.utilities;

/**
 * A filter to achieve smooth rate of change to a desired input speed through the equation: <br>
 * </br>
 * output = (alpha*input) + ((1-alpha)*oldspeed) <br>
 * </br>
 * Where alpha is a tunable coefficient.
 *
 * @author Adam N.
 */
public class AlphaFilter implements Filter {

  private double mAlpha;
  private double mFilteredSpeed;

  /**
   * @param The desired alpha coefficient
   */
  public AlphaFilter(double alpha) {
    mAlpha = alpha;
  }

  /**
   * Calculates the correct output based on a desired value.
   *
   * @param input The desired value to reach
   * @return The calculated output based on previous inputs.
   */
  public double calculate(double input) {
    mFilteredSpeed = (mAlpha * input) + ((1 - mAlpha) * mFilteredSpeed);

    return mFilteredSpeed;
  }

  /**
   * set the alpha gain.
   *
   * @param alpha The alpha gain
   */
  public void setAlpha(double alpha) {
    mAlpha = alpha;
  }

  @Override
  public void reset() {
    mFilteredSpeed = 0.0;
  }
}
