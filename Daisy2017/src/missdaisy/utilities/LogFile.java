package missdaisy.utilities;

import java.io.File;
import java.util.Arrays;

public class LogFile extends File implements Comparable<File> {
  public int logNumber;

  public LogFile(File file, int logNumber) {
    super(file.getAbsolutePath());
    this.logNumber = logNumber;
  }

  public int getLogNumber() {
    return this.logNumber;
  }

  public static void sortArray(LogFile[] arr) {
    Arrays.sort(arr);
  }

  public int compareTo(File o) {
    LogFile input = (LogFile) o;
    if (this.getLogNumber() < input.getLogNumber()) {
      return -1;
    } else if (this.getLogNumber() > input.getLogNumber()) {
      return 1;
    } else {
      return 0;
    }
  }
}
