package modeling;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

import modeling.Constants.*;
import sim.engine.*;
import sim.util.*;

/**
 * Class to describe road objects
 * 
 * @author hh940
 *
 */

public class Road extends Line2D.Double implements Steppable 
{
	protected final int ID;
	protected final int type;	
	protected boolean isSchedulable;
	public static final double roadWidth = 6.0;
	private double roadLength;
	private double direction; // HH 2.10.14 This can range from 0 (incl) to 180 (excl) where 0 is equivalent to N/S and 90 to E/W
	
	public Road(int idNo, int typeNo, Double2D start, Double2D end)
	{
		ID = idNo;
		type = typeNo;
		this.setLine(start.x, start.y, end.x, end.y);
		
		// calculate length
		if (start.x == end.x) {
			// Vertical line
			roadLength = Math.abs(start.y-end.y);
			setDirection(0); // HH 2.10.14 Replaced isNS = true
			
		} else if (start.y == end.y) {
			// Horizontal line
			roadLength = Math.abs(start.x-end.x);
			setDirection(90); // HH 2.10.14 Replaced isNS = false
		} else {
			// TO DO - Raise an error here
		}
	}	
		
	/**
	 * @return the direction
	 */
	public double getDirection() {
		return direction;
	}

	/**
	 * @param direction the direction to set
	 */
	public void setDirection(double direction) {
		this.direction = direction;
	}

	@Override
	public void step(SimState state) {
		// TODO Auto-generated method stub
		
	}
	
	public boolean isSchedulable() {
		return isSchedulable;
	}
	
	public int getType() {return type;}
	public int getID() {return ID;}
	public double getLength() {return roadLength;}
	
	/* 
	 * HH 2.10.14 - Updated to use direction field
	 * 
	 * TO DO: It may not be appropriate to call this method when we have reverted to non-grid road layout
	 */
	public boolean getIsNS() 
	{
		if (direction == 0)
		{
			return true;
		} else {
			return false;
		}
	}

	public String toString() 
	{
		return "" + getID();
	}
	
	/**
	 * Method returns true if coord intersects with the road object (when extended by required width), false otherwise
	 */
	public boolean inShape(Double2D coord)
	{
		// Construct a rectangle that is the size of the whole road area (rather than just the centre line)
		Rectangle2D.Double roadSurface = getSurface();
		
		return roadSurface.contains(coord.x, coord.y);
	}	
	
	/**
	 * Method returns true if rectangle intersects with the road object (when extended by required width), false otherwise
	 */
	public boolean inShape(Rectangle2D.Double inRectangle)
	{
		// Construct a rectangle that is the size of the whole road area (rather than just the centre line)
		Rectangle2D.Double roadSurface = getSurface();
		
		return roadSurface.intersects(inRectangle);
	}	
	
	/**
	 *  Method returns a Rectangle2D which represents the whole road area (rather than just the centre line)
	 */
	public Rectangle2D.Double getSurface()
	{
		double tlx;
		double tly;
		
		Rectangle2D.Double roadSurface = new Rectangle2D.Double();
		
		// Calculate the coordinates of the upper left corner of the rectangle
		if (x1 == x2) {
			// Vertical line
			tlx = x1 - roadWidth/2;
			tly = Math.min(y1, y2);
			roadLength = Math.abs(y1-y2);
			roadSurface = new Rectangle2D.Double(tlx, tly, roadWidth, roadLength);
		} else if (y1 == y2) {
			// Horizontal line
			tly = y1 - roadWidth/2;
			tlx = Math.min(x1, x2);
			roadLength = Math.abs(x1-x2);
			roadSurface = new Rectangle2D.Double(tlx, tly, roadLength, roadWidth);
		} else {
			// TO DO - Raise an error here
		}
		
		return roadSurface;
		
// TO DO - This code was written to support non x/y aligned rectangles, but since these are not supported by 
// Rectangle2D, this code may need to be rewritten using polygons or another class at a later point.
//		// Calculate the coordinates of the upper left corner of the rectangle
//		double angle = Math.atan((y2-y1)/(x2-x1));
//		
//		double dX = Math.sin(angle)*(roadWidth/2);
//		double dY = Math.cos(angle)*(roadWidth/2);
//		
//		double length = Math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
//		
//		// NOTE: in order to apply these displacements to calculate the perpendicular offsets
//		// we actually need to swap the x and y displacements over, and change the sign of each
//		// result.  If we don't swap the x and y, the displacements are either back along the road
//		// or a continuation of it.  Swapping the sign changes whether the displacement is to the
//		// left or the right.  TO DO - may need to check this works okay, as will depend on how 
//		// important it is to return the top left of the rectangle
//		
//		Rectangle2D.Double roadSurface = new Rectangle2D.Double(x1-(dY), y1-(-dX), roadWidth, length);
//		return roadSurface;
	}
	
	/**
	 * Returns the distance from the closest part of the obstacle to the coord provided.
	 * 
	 * @param coord the coordinate the distance to be checked for
	 */
	public double obstacleToPoint(Double2D coord)
	{
		return java.lang.Double.MAX_VALUE; // TO DO - Implement
	}

	/**
	 *  HH 16/6/14 Method returns a Rectangle2D which represents the painted lines on the specified location on the
	 *  road.  Assumes a line thickness of 10cm at the sides, and 10cm in the centre.  For now, assume continuous single
	 *  line in both cases (this should be adjusted in time to allow for broken lines at junctions, and broken lines in 
	 *  the centre (warning/standard) - see workbook for details (or Traffic Signs Manual, Ch 5, 2003).
	 *  
	 *  argument: nearside / centre / offside
	 */
	public Rectangle2D.Double getLine(LineType inLineType)
	{
		double tlx;
		double tly;
		
		Rectangle2D.Double paintedLine = new Rectangle2D.Double();
		
		switch (inLineType) {
			case NWSIDE : {
		
				// Calculate the coordinates of the upper left corner of the rectangle
				if (x1 == x2) {
					// Vertical line
					tlx = x1 - roadWidth/2 + Constants.ROADEDGEOFFSET;
					tly = Math.min(y1, y2);
					roadLength = Math.abs(y1-y2);
					paintedLine = new Rectangle2D.Double(tlx, tly, Constants.ROADEDGINGWIDTH, roadLength);
				} else if (y1 == y2) {
					// Horizontal line
					tly = y1 - roadWidth/2 + Constants.ROADEDGEOFFSET;
					tlx = Math.min(x1, x2);
					roadLength = Math.abs(x1-x2);
					paintedLine = new Rectangle2D.Double(tlx, tly, roadLength, Constants.ROADEDGINGWIDTH);
				} else {
					// TO DO - Raise an error here
				}
				break;
			}
			case SESIDE : {
				// Calculate the coordinates of the upper left corner of the rectangle
				if (x1 == x2) {
					// Vertical line
					tlx = x1 + roadWidth/2 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH;
					tly = Math.min(y1, y2);
					roadLength = Math.abs(y1-y2);
					paintedLine = new Rectangle2D.Double(tlx, tly, Constants.ROADEDGINGWIDTH, roadLength);
				} else if (y1 == y2) {
					// Horizontal line
					tly = y1 + roadWidth/2 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH;
					tlx = Math.min(x1, x2);
					roadLength = Math.abs(x1-x2);
					paintedLine = new Rectangle2D.Double(tlx, tly, roadLength, Constants.ROADEDGINGWIDTH);
				} else {
					// TO DO - Raise an error here
				}				
				break;
			}
			case CENTRE : {
				// Calculate the coordinates of the upper left corner of the rectangle
				if (x1 == x2) {
					// Vertical line
					tlx = x1 - Constants.ROADEDGINGWIDTH/2;
					tly = Math.min(y1, y2);
					roadLength = Math.abs(y1-y2);
					paintedLine = new Rectangle2D.Double(tlx, tly, Constants.ROADEDGINGWIDTH, roadLength);
				} else if (y1 == y2) {
					// Horizontal line
					tly = y1 - Constants.ROADEDGINGWIDTH/2;
					tlx = Math.min(x1, x2);
					roadLength = Math.abs(x1-x2);
					paintedLine = new Rectangle2D.Double(tlx, tly, roadLength, Constants.ROADEDGINGWIDTH);
				} else {
					// TO DO - Raise an error here
				}				
				break;
			}
		}
		
		return paintedLine;
		
// TO DO - This code was written to support non x/y aligned rectangles, but since these are not supported by 
// Rectangle2D, this code may need to be rewritten using polygons or another class at a later point.
//		// Calculate the coordinates of the upper left corner of the rectangle
//		double angle = Math.atan((y2-y1)/(x2-x1));
//		
//		double dX = Math.sin(angle)*(roadWidth/2);
//		double dY = Math.cos(angle)*(roadWidth/2);
//		
//		double length = Math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
//		
//		// NOTE: in order to apply these displacements to calculate the perpendicular offsets
//		// we actually need to swap the x and y displacements over, and change the sign of each
//		// result.  If we don't swap the x and y, the displacements are either back along the road
//		// or a continuation of it.  Swapping the sign changes whether the displacement is to the
//		// left or the right.  TO DO - may need to check this works okay, as will depend on how 
//		// important it is to return the top left of the rectangle
//		
//		Rectangle2D.Double roadSurface = new Rectangle2D.Double(x1-(dY), y1-(-dX), roadWidth, length);
//		return roadSurface;
	}
	
	/**
	 *  HH 18/6/14 Method returns a Line2D.Double which represents the innermost edge of the painted lines on the specified location on the
	 *  road.  Assumes a line thickness of 10cm at the sides.  For now, assume continuous single
	 *  line in both cases (this should be adjusted in time to allow for broken lines at junctions, and broken lines in 
	 *  the centre (warning/standard) - see workbook for details (or Traffic Signs Manual, Ch 5, 2003).
	 *  
	 *  argument: nearside / offside
	 */
	public Line2D.Double getThinLine(LineType inLineType)
	{
		double tlx;
		double tly;
		
		Line2D.Double paintedLine = new Line2D.Double();
		
		switch (inLineType) {
			case SESIDE : {
		
				// Calculate the coordinates of the upper left corner of the rectangle
				if (x1 == x2) {
					// Vertical line
					tlx = x1 - roadWidth/2 + Constants.ROADEDGEOFFSET + Constants.ROADEDGINGWIDTH;
					tly = Math.min(y1, y2);
					roadLength = Math.abs(y1-y2);
					paintedLine = new Line2D.Double(tlx, tly, tlx, tly + roadLength);
				} else if (y1 == y2) {
					// Horizontal line
					tly = y1 - roadWidth/2 + Constants.ROADEDGEOFFSET + Constants.ROADEDGINGWIDTH;
					tlx = Math.min(x1, x2);
					roadLength = Math.abs(x1-x2);
					paintedLine = new Line2D.Double(tlx, tly, tlx + roadLength, tly);
				} else {
					// TO DO - Raise an error here
				}
				break;
			}
			case NWSIDE : {
				// Calculate the coordinates of the upper left corner of the rectangle
				if (x1 == x2) {
					// Vertical line
					tlx = x1 + roadWidth/2 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH;
					tly = Math.min(y1, y2);
					roadLength = Math.abs(y1-y2);
					paintedLine = new Line2D.Double(tlx, tly, tlx, tly + roadLength);
				} else if (y1 == y2) {
					// Horizontal line
					tly = y1 + roadWidth/2 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH;
					tlx = Math.min(x1, x2);
					roadLength = Math.abs(x1-x2);
					paintedLine = new Line2D.Double(tlx, tly, tlx + roadLength, tly);
				} else {
					// TO DO - Raise an error here
				}				
				break;
			}
			
			//[TODO] - May want to error if supplied with CENTRE
		}
		
		return paintedLine;
		
	}
	
	/*
	 * HH 4.9.14 getLane(coord) - return 1 for N or E; 2 for S or W
	 */
	public int getLane(Double2D coord)
	{
		int retVal = 0;
		
		if (getIsNS() == true) 
		{
			if (coord.x >= x1)
			{
				retVal = 2;
			} else {
				retVal = 1;
			}	
		} else {
			if (coord.y >= y1)
			{
				retVal = 2;
			} else {
				retVal = 1;
			}				
		}
				
		return retVal;
	}
}
