package missdaisy.utilities;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

/**
 * A logger which can store an undefined amount of rows and a defined number of columns. All data
 * should be Numbers.
 * 
 * Write does one dump of the data into a file specified by the Logger.Config. To add rows, use the
 * addRow method.
 * 
 * This Logger will not overwrite log files until the number of log files has reached the number
 * specified in the Config.
 * 
 * You can use the same folder for different types of logs, but it may be beneficial to store
 * different logs in different folders for organizational purposes
 * 
 * @author Joshua Sizer
 *
 */
public class Logger {
  public static final String EXTENSION = ".csv";
  private ArrayList<Number>[] data;
  private Config config;
  private int numLogsToKeep;

  public static class Config {
    public String[] columnNames;
    public int numColumns;
    public File filePath;
    public String fileName;

    /**
     * @param columnNames The titles of the columns, as a String array
     * @param numColumns The number of columns. This must be the same as the size of the columnNames
     *        array
     * @param filePath The directory where the logs will be stored
     * @param fileName A name for the log files. This will be the prefix of all the log files. This
     *        should be unique to the data you are writing (such as DriveTrajectoryFollower)
     */
    public Config(String[] columnNames, int numColumns, File filePath, String fileName) {
      this.columnNames = columnNames;
      this.numColumns = numColumns;
      this.filePath = filePath;
      this.fileName = fileName;
    }
  }

  public Logger(Config config, int numLogsToKeep) {
    this.config = config;
    this.numLogsToKeep = numLogsToKeep;
    checkConfig();
    init();
  }
  
  @SuppressWarnings("unchecked")
  private void init() {
    data = new ArrayList[config.numColumns];
    for (int i = 0; i < data.length; i++) {
      data[i] = new ArrayList<Number>();
    }
  }
  
  /**
   * Throws any exceptions based on the input configuration object
   */
  public void checkConfig() {
    if (!this.config.filePath.exists()) {
      throw new RuntimeException("The file path does not exist! Please create it to continue.");
    }
    if (this.numLogsToKeep < 1) {
      throw new RuntimeException("The number of logs to keep must be at least 1.");
    }
  }

  /**
   * Add a row of numbers.
   * 
   * @param row
   */
  public void addRow(Number[] row) {
    for (int i = 0; i < data.length; i++) {
      Number toAdd = 0;
      if (i < row.length) {
        toAdd = row[i];
      }
      data[i].add(toAdd);
    }
  }
  
  public void clear() {
   init(); 
  }

  public void write() throws IOException {
    String finalPath = "";

    LogFile[] logFiles = getLogsInDir();
    LogFile.sortArray(logFiles);

    // Determines if we must over write an existing log file.
    if (logFiles.length >= numLogsToKeep) {
      logFiles[logFiles.length - 1].delete();
    }
    
    int logNum = logFiles.length + 1;

    for (int i = logFiles.length - 1; i > -1; i--) {
      logFiles[i].renameTo(new File(generatePath(logNum--)));
    }

    finalPath = generatePath(1);

    toFile(this.toString(), finalPath);
    System.out.println("Log written to: " + finalPath);
  }

  /**
   * 
   * @return The number of logs in the directory
   */
  private int numLogsInDir() {
    int numLogs = 0;
    File[] oldFiles = this.config.filePath.listFiles();
    // goes through each file in the directory, and compares
    // its beginning file name to the desired logName
    for (File oldFile : oldFiles) {
      if (fileIsLog(oldFile)) {
        numLogs++;
      }
    }

    return numLogs;
  }

  /**
   * Creates a log file path based on the input config and the log number argument
   * 
   * @param logNumber
   * @return an absolute path of a log file
   */
  private String generatePath(int logNumber) {
    String path = this.config.filePath.getAbsolutePath();
    path = path.concat(File.separator + this.config.fileName + logNumber + Logger.EXTENSION);
    return path;
  }

  /**
   * @return All the log files in the directory
   */
  private LogFile[] getLogsInDir() {
    int numLogsInDir = numLogsInDir();
    System.out.println("Found " + numLogsInDir + " similar logs in this directory.");
    LogFile[] logs = new LogFile[numLogsInDir];
    int index = 0;

    for (File cur : this.config.filePath.listFiles()) {
      if (fileIsLog(cur)) {
        LogFile toAdd = convertToLogFile(cur);
        if (toAdd != null) {
          logs[index++] = toAdd;
        }
      }
    }

    return logs;
  }

  /**
   * Returns true if a file has the format of "fileName#.EXTENSION", else returns false
   * 
   * @param file
   * @return
   */
  private boolean fileIsLog(File file) {
    String fileName = file.getName();

    return fileName.startsWith(this.config.fileName) && fileName.endsWith(Logger.EXTENSION);
  }

  /**
   * Creates a LogFile object from a File object. This associates a File with a log number.
   * 
   * @param file The file to convert
   * @return Null if the file path is not in the correct format, and a LogFile with the log number
   *         correctly assigned.
   */
  private LogFile convertToLogFile(File file) {
    int logNumber = -1;
    String name = file.getName();

    String s_num =
        name.substring(this.config.fileName.length(), name.length() - Logger.EXTENSION.length());
    try {
      logNumber = Integer.parseInt(s_num);
      return new LogFile(file, logNumber);
    } catch (IllegalArgumentException e) {
      return null;
    }
  }

  public String toString() {
    StringBuilder ret = new StringBuilder("");

    for (int i = 0; i < data.length; i++) {
      ret.append(this.config.columnNames[i] += ",");
    }

    ret.append("\r\n");

    for (int i = 0; i < data[0].size(); i++) {
      for (int k = 0; k < data.length; k++) {
        ret.append(data[k].get(i) + ",");
      }
      ret.append("\r\n");
    }

    return ret.toString();
  }

  /**
   * 
   * @param data The text to write to the file
   * @param path The path to write the text to
   * @throws IOException
   */
  private void toFile(String data, String path) throws IOException {
    BufferedWriter fileWriter = new BufferedWriter(new FileWriter(path));
    fileWriter.write(data);
    fileWriter.close();
  }

  /**
   * For testing purposed
   * 
   * @param args
   */
  public static void main(String[] args) {
    File dir = new File(System.getProperty("user.home") + "\\Desktop\\LoggerTest");
    String[] headers = {"Time", "Pos", "Vel", "Acc"};
    String fileName = "DriveTrajectoryLog";
    Logger.Config loggerConfig = new Logger.Config(headers, headers.length, dir, fileName);

    Logger logger = new Logger(loggerConfig, 10);

    try {
      Random rand = new Random();
      for (int i = 0; i < 1000; i++) {
        logger.addRow(new Number[] {i, rand.nextInt(2000), rand.nextInt(2000), rand.nextInt(2000)});
      }
      logger.write();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}