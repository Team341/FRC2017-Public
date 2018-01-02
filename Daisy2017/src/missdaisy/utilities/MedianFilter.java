package missdaisy.utilities;

import java.util.Arrays;

/**
 * A median filter. See: http://en.wikipedia.org/wiki/Median_filter
 *
 * This is a nonlinear low pass filter that smooths out a noisy measurement by taking the median of
 * the last N measurements.
 *
 * @author Jared341
 */
public class MedianFilter implements Filter {

  private final double[] vals;
  private int ptr = 0;
  private double median = 0.0;

  public MedianFilter(int nSamples) {
    vals = new double[nSamples];
  }

  @Override
  public void reset() {
    for (int i = 0; i < vals.length; i++) {
      vals[i] = 0.0;
    }
    ptr = 0;
    median = 0.0;
  }

  public void setInput(double val) {
    vals[ptr] = val;
    ++ptr;

    if (ptr >= vals.length) {
      ptr = 0;
    }
  }

  public double getMedian() {
    return median;
  }

  public double run() {
    median = 0.0;

    double[] sorted = Arrays.copyOf(vals, vals.length);
    Arrays.sort(sorted);

    median = sorted[vals.length / 2];

    return median;
  }

}
