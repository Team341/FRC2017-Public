package missdaisy.utilities;

/**
 * A moving average filter.
 *
 * This is a low pass filter that smooths out a noisy signal by averaging the last N measurements.
 *
 * @author Jared341
 */
public class MovingAverageFilter implements Filter {

  private final double[] vals;
  private int ptr = 0;
  private double average = 0.0;

  public MovingAverageFilter(int nSamples) {
    vals = new double[nSamples];
  }

  @Override
  public void reset() {
    for (int i = 0; i < vals.length; i++) {
      vals[i] = 0.0;
    }
    ptr = 0;
    average = 0.0;
  }

  public void setInput(double val) {
    vals[ptr] = val;
    ++ptr;

    if (ptr >= vals.length) {
      ptr = 0;
    }
  }

  public double getAverage() {
    return average;
  }

  public double run() {
    average = 0.0;
    for (int i = 0; i < vals.length; i++) {
      average += vals[i];
    }
    average /= vals.length;
    return average;
  }

}
