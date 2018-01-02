package missdaisy.utilities;

/**
 * Filters a quantity by limiting the rate of change.
 *
 * This is a nonlinear low pass filter that smooths out sharp changes by limiting the first
 * derivative (rate of change) to a specified maximum.
 *
 * @author Jared341
 */
public class RateLimitFilter implements Filter {

  private double maxRate = 1.0; // Per LOOP
  private double lastVal = 0.0;
  private double desired = 0.0;

  public RateLimitFilter(double maxRate) {
    this.maxRate = maxRate;
  }

  public void setMaxRate(double rate) {
    maxRate = rate;
  }

  public double getMaxRate() {
    return maxRate;
  }

  @Override
  public void reset() {
    lastVal = 0.0;
    desired = 0.0;
  }

  public void setDesired(double desired) {
    this.desired = desired;
  }

  public double run() {
    double difference = desired - lastVal;
    if (difference > maxRate) {
      lastVal += maxRate;
    } else if (difference < -maxRate) {
      lastVal -= maxRate;
    } else {
      lastVal = desired;
    }
    return lastVal;
  }

}
