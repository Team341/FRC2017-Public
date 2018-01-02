/*
 * The following code was found at
 * http://www.jfree.org/phpBB2/viewtopic.php?f=3&t=24525
 * 
 * AJN - modified it to handle inverted y axis, need to work on zooming out issues
 */


package tools.patheditor;

import java.awt.AlphaComposite;
import java.awt.Composite;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;

import org.jfree.chart.axis.ValueAxis;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.Range;
import org.jfree.data.xy.XYDataset;
import org.jfree.ui.Align;

public class XYPlotWithZoomableBackgroundImage extends XYPlot {

  private boolean zoomableBackgroundImage;

  public XYPlotWithZoomableBackgroundImage(XYDataset dataset, ValueAxis domainAxis,
      ValueAxis rangeAxis, XYItemRenderer renderer, boolean zoomableBackgroundImage) {
    super(dataset, domainAxis, rangeAxis, renderer);
    this.zoomableBackgroundImage = zoomableBackgroundImage;
  }

  /**
   * Draws the background image (if there is one) aligned within the specified area.
   *
   * @param g2 the graphics device.
   * @param area the area.
   *
   * @see #getBackgroundImage()
   * @see #getBackgroundImageAlignment()
   * @see #getBackgroundImageAlpha()
   */
  public void drawBackgroundImage(Graphics2D g2, Rectangle2D area) {
    System.out.print("using me to draw background: ");
    
    if (!zoomableBackgroundImage) {
      // if background image is not zoomable call Plot.drawBackgroundImage(g2,area);
      super.drawBackgroundImage(g2, area);
      System.out.println("default");
    } else {

      // get the background image
      Image backgroundImage = this.getBackgroundImage();

      // make sure we have a background image
      if (backgroundImage != null) {

        // get background image alignment
        int backgroundImageAlignment = this.getBackgroundImageAlignment();

        // get background image alpha
        float backgroundImageAlpha = this.getBackgroundImageAlpha();

        // get X Axis
        ValueAxis xAxis = getDomainAxis();

        // get Y Axis
        ValueAxis yAxis = getRangeAxis();

        // get full X Range
        Range xRange = getDataRange(xAxis);

        // get full Y Range
        Range yRange = getDataRange(yAxis);

        // get full x range value
        double xDataUpperBound = xRange.getUpperBound();
        double xDataLowerBound = xRange.getLowerBound();
        double xRangeValue = xDataUpperBound - xDataLowerBound;

        // get full y range value
        double yDataUpperBound = yRange.getUpperBound();
        double yDataLowerBound = yRange.getLowerBound();
        double yRangeValue = yDataUpperBound - yDataLowerBound;


        // get current min X
        double xmin = xAxis.getLowerBound();

        // get current max X
        double xmax = xAxis.getUpperBound();

        // get current min Y
        double ymin = yAxis.getLowerBound();

        // get current max Y
        double ymax = yAxis.getUpperBound();

        if (yRangeValue < Math.abs(ymax - ymin)) {
          yRangeValue = Math.abs(ymax - ymin);
        }

        if (xRangeValue < Math.abs(xmax - xmin)) {
          xRangeValue = Math.abs(xmax - xmin);
        }

        // get original image width
        int originalImageWidth = backgroundImage.getWidth(null);

        // get original image height
        int originalImageHeight = backgroundImage.getHeight(null);

        // we are starting at top left for image and bottom left for drawing graph

        // move to point 0,0
        double xmin2 = xmin - xDataLowerBound;
        double xmax2 = xmax - xDataLowerBound;
        double ymin2 = ymin - yDataLowerBound;
        double ymax2 = ymax - yDataLowerBound;

        // flip the y axis
        double xmin3 = xmin2;
        double xmax3 = xmax2;
        double ymin3 = yRangeValue - ymax2;
        double ymax3 = yRangeValue - ymin2;
        if (yAxis.isInverted()) {
          // Fix for when Y axis is inverted
          ymin3 = ymin2;
          ymax3 = ymax2;
        } 

        System.out.println("ymax " + ymax);
        System.out.println("ymin " + ymin);
        System.out.println("ymax2 " + ymax2);
        System.out.println("ymin2 " + ymin2);

        System.out.println("xmin3 " + xmin3);
        System.out.println("xmax3 " + xmax3);
        System.out.println("ymin3 " + ymin3);
        System.out.println("ymax3 " + ymax3);


        // ARGB to support transparency if in original image
        BufferedImage bi =
            new BufferedImage(originalImageWidth, originalImageHeight, BufferedImage.TYPE_INT_ARGB); 

        Graphics2D g = bi.createGraphics();
        g.drawImage(backgroundImage, 0, 0, null);
        g.dispose();


        double newXMin = ((xmin3 / xRangeValue) * originalImageWidth);
        double newXMax = ((xmax3 / xRangeValue) * originalImageWidth);
        double newYMin = ((ymin3 / yRangeValue) * originalImageHeight);
        double newYMax = ((ymax3 / yRangeValue) * originalImageHeight);


        if (newXMin < 0) {
          newXMin = 0;
        }
        if (newYMin < 0) {
          newYMin = 0;
        }

        System.out.println("newXMin " + newXMin);
        System.out.println("newYMin " + newYMin);
        System.out.println("(newXMax - newXMin) " + (newXMax - newXMin));
        System.out.println("(newYMax - newYMin) " + (newYMax - newYMin));

        double subImageWidth = (newXMax - newXMin);
        double subImageHeight = (newYMax - newYMin);


        if ((newYMin + subImageHeight) > (bi.getHeight())) {
          subImageHeight = bi.getHeight() - newYMin;
        }

        if ((newXMin + subImageWidth) > bi.getWidth()) {
          subImageWidth = bi.getWidth() - newXMin;
        }



        BufferedImage bi2 = bi.getSubimage((int) (newXMin), (int) (newYMin), (int) (subImageWidth),
            (int) (subImageHeight));

        Composite originalComposite = g2.getComposite();
        g2.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, backgroundImageAlpha));
        Rectangle2D dest =
            new Rectangle2D.Double(0.0, 0.0, bi2.getWidth(null), bi2.getHeight(null));
        Align.align(dest, area, backgroundImageAlignment);
        g2.drawImage(bi2, (int) dest.getX(), (int) dest.getY(), (int) dest.getWidth() + 1,
            (int) dest.getHeight() + 1, null);


        g2.setComposite(originalComposite);
        System.out.println("custom");
      }
    }
    
  }


}
