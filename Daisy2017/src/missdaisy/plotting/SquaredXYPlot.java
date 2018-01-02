package missdaisy.plotting;

/**
*class SquaredXYPlot.java
*by Erica Liszewski
*December 20, 2005
*
*An extension to the XYPlot provided in JFreeChart, this class draws
*XY plots where the spacing between two given values is the same on
*both the X and Y axis.  This class over-rides draw() in XYPlot to add it's
*functionality.
*/


import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

import org.jfree.chart.axis.AxisSpace;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.chart.plot.PlotRenderingInfo;
import org.jfree.chart.plot.PlotState;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.Range;
import org.jfree.data.xy.XYDataset;
import org.jfree.ui.RectangleEdge;
import org.jfree.ui.RectangleInsets;

public class SquaredXYPlot extends XYPlot {
    
    /**
   * 
   */
  private static final long serialVersionUID = 1L;
    protected boolean squareToRange;
    
    /*public SquaredXYPlot()
     *default constructor
     */
    public SquaredXYPlot(){
        super();
        squareToRange = true;
    }
    
    /*public SquaredXYPlot(XYDataset dataset,
            ValueAxis domainAxis,
            ValueAxis rangeAxis,
            XYItemRenderer renderer)
     */
    public SquaredXYPlot(XYDataset dataset,
            ValueAxis domainAxis,
            ValueAxis rangeAxis,
            XYItemRenderer renderer) {
        super(dataset, domainAxis, rangeAxis, renderer);
        squareToRange = true;
    }
    
    /*public void setSquaredToRange(boolean squareToRange)
     *determines which axis is changed so the axis end up squared
     *
     *squareToRange is true if the y-axis is left alone and the
     *     x-axis is adjusted to match
     *squareToRange is false if the x-axis is left alone and the
     *    y-axis is adjusted
     */
    public void setSquaredToRange(boolean squareToRange){
        this.squareToRange = squareToRange;
    }
    
    /*public boolean getSquaredToRange()
     *returns the squareToRange
     *see setSquaredToRange()
     */
    public boolean getSquaredToRange(){
        return this.squareToRange;
    }
    
    /*public void draw(Graphics2D g2,
            Rectangle2D area,
            Point2D anchor,
            PlotState parentState,
            PlotRenderingInfo info)
     *over-ridden draw method, so we can calculate the correct axis
     */
    @Override
    public void draw(Graphics2D g2,
            Rectangle2D area,
            Point2D anchor,
            PlotState parentState,
            PlotRenderingInfo info) {
        
        /**this stuff from original draw, to get the dataArea**/
        // if the plot area is too small, just return...
        boolean b1 = (area.getWidth() <= MINIMUM_WIDTH_TO_DRAW);
        boolean b2 = (area.getHeight() <= MINIMUM_HEIGHT_TO_DRAW);
        if (b1 || b2) {
            return;
        }
        
        // record the plot area...
        if (info != null) {
            info.setPlotArea(area);
        }
        
        // adjust the drawing area for the plot insets (if any)...
        RectangleInsets insets = getInsets();
        insets.trim(area);
        
        AxisSpace space = calculateAxisSpace(g2, area);
        Rectangle2D dataArea = space.shrink(area, null);
        this.getAxisOffset().trim(dataArea);
        /** end stuff from original draw() **/
        
        //if the range is being adjusted
        if( !this.squareToRange){
            
            ValueAxis adjaxis = getRangeAxis();
            ValueAxis stataxis = getDomainAxis();

            //calculate the Java2D coord on the x-axis that is the same distance from
            //0 as the maximim coord on the y-axis
            double xcoord = dataArea.getMinX() + (dataArea.getMaxY()-dataArea.getMinY());
                
            //get the axis value of the coord
            double scale = stataxis.java2DToValue(xcoord, dataArea, RectangleEdge.BOTTOM);
            //System.out.println("new range is "+scale);
            //set the value of the coord as the maximum range
            adjaxis.setRange(new Range(adjaxis.getRange().getLowerBound(), scale));
                
        }else {//if the domain is being adjusted
            ValueAxis adjaxis = getDomainAxis();
            ValueAxis stataxis = getRangeAxis();

            //calculate the Java2D coord on the y-axis that is the same distance from
            //0 as the maximim coord on the x-axis
            double ycoord = dataArea.getMaxY() - (dataArea.getMaxX()-dataArea.getMinX());
            //get the axis value of the coord
            double scale = stataxis.java2DToValue(ycoord, dataArea, RectangleEdge.LEFT);
            //System.out.println("new domain is "+scale);
            adjaxis.setRange(new Range(adjaxis.getRange().getLowerBound(), scale));
            
        }
        //call the original draw method to handle things from here
        super.draw(g2, area, anchor, parentState, info);
        
    }
    
}