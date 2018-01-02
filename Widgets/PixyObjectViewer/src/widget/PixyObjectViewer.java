package widget;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import edu.wpi.first.smartdashboard.gui.StaticWidget;
import edu.wpi.first.smartdashboard.properties.Property;
import edu.wpi.first.smartdashboard.properties.StringProperty;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class PixyObjectViewer extends StaticWidget {
  private static final long serialVersionUID = 1617984877783278744L;
  public static final String NAME = "Pixy Object Viewer";
  NetworkTable piTable;
  BGThread bgThread;
  Dimension size;
  StringProperty ntKey;
  StringProperty xKey;
  StringProperty yKey;
  StringProperty widthKey;
  StringProperty heightKey;
  StringProperty IP;
  private int width;
  private int height;
  private int x = 0;
  private int y = 0;
  final String errorMessage = "Not reading Pixy!";
  
  public PixyObjectViewer() {
	NetworkTable.initialize();
    NetworkTable.setClientMode();
    size = new Dimension(320, 200);
    super.setPreferredSize(size);
    super.setResizable(false);
    bgThread = new BGThread("Pixy Camera Object Viewer Thread");
    ntKey = new StringProperty(this, "Network Table", "RaspberryPi");
    xKey = new StringProperty(this, "X Key Name", "x");
    yKey = new StringProperty(this, "Y Key Name", "y");
    widthKey = new StringProperty(this, "Width Key Name", "width");
    heightKey = new StringProperty(this, "Height Key Name", "height");
    IP = new StringProperty(this, "Server IP", "raspberrypi.local");
    NetworkTable.setIPAddress(IP.getValue());
  }
  
  private void getValues() {
    width = (int) piTable.getNumber(widthKey.getValue(), -1);
    height = (int) piTable.getNumber(heightKey.getValue(), -1);
    x = (int) piTable.getNumber(xKey.getValue(), -1);
    y = (int) piTable.getNumber(yKey.getValue(), -1);
  }
  
  @Override
  public void propertyChanged(Property prop) {
    System.out.println(NAME + ": Property \"" + prop.getName() + "\" changed value to \"" + prop.getValue() + "\"");
    if (prop == ntKey) {
      piTable = NetworkTable.getTable(ntKey.getValue());
      System.out.println("Opened NetworkTable \"" + ntKey.getValue() + "\"");
    } else if (prop == IP) {
    	NetworkTable.setIPAddress(IP.getValue());
    	System.out.println("Changed IP address to \"" + IP.getValue() + "\"");
    }
  }

  @Override
  public void init() {
    piTable = NetworkTable.getTable(ntKey.getValue());
    bgThread.start();
  }
  
  @Override
  public void paintComponent(Graphics g) {
    getValues();
    if (g != null) {
    	if (x != -1) {
		  g.setColor(Color.BLACK);
		  g.fillRect(0, 0, size.width, size.height);
		  g.setColor(Color.WHITE);
		  g.fillRect(x, y, height, width);
    	} else {
    	  g.setColor(Color.BLACK);
    	  g.fillRect(0, 0, size.width, size.height);
    	  g.setColor(Color.RED);
    	  g.drawString(errorMessage, 10, 25);
    	}
    } else {
      System.out.println("Graphics object is null");
    }
  }
  
  private class BGThread extends Thread {
    public BGThread(String name) {
      super(name);
    }
    
    @Override
    public void run() {
      while (!interrupted()) {
        getValues();
        revalidate();
        repaint();
        try {
          Thread.sleep(15);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
  }
}
