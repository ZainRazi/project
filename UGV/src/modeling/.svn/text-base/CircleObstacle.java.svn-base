package modeling;
import sim.util.Double2D;

/**
 *
 * @author Robert Lee
 */
public class CircleObstacle extends Obstacle
{
	private double radius;
	
	public CircleObstacle(int idNo, double r)
	{
		super(idNo, TCIROBSTACLE);
		radius = r;
	}
	
	/**
	 * Returns true or false if a provided coordinate is in the shape, used to
	 * detect collisions.
	 * 
	 * @param coord the coordinate to be tested if it's in the object
	 * @return a value based on if the coordinate is in the shape (true) or not (false)
	 */
	@Override
	public boolean inShape(Double2D coord)
	{
		if (location.distance(coord) <= radius)
		{
			return true;
		} else {
			return false;
		}
	}	

	/**
	 * Returns the distance from the provided coordinate to the closest part of
	 * the obstacle to that point.
	 * 
	 * @param coord
	 * @return 
	 */
	@Override
	public double obstacleToPoint(Double2D coord)
	{
		//as the shape of this obstacle is a circle the closest point is the edge
		//of the circle to the point.
		double val = (location.distance(coord) - radius);
		
		if (val < 0)
		{
			return 0;
		} else {
			return val;
		}
	}
	
	//accessor methods
	public double getRadius() {return radius;}
}
