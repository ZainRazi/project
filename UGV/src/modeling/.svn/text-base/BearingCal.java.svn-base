package modeling;
import sim.util.Double2D;


public class BearingCal {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		double out = calculateAngle(new Double2D(0,-1));
		System.out.println(out);
		
		// TODO Auto-generated method stub

	}
	
	private static double calculateAngle(Double2D point1)
	{
		Double2D vector = point1;
		double angle;
		if(vector.y != 0)
		{
			angle = Math.toDegrees(Math.atan(vector.x / vector.y));
			
			if(vector.x >0)
			{
				if (vector.y <0) 
				{	
					angle +=180;
					
				} 
				
			}
			else
			{
				if (vector.y <0) 
				{	
					angle +=180;
					
				}
				else
				{
					angle +=360;
				}
			}
			
			
		
		} else {
			//the car is either in line with the target horizontally or vertically
			if (vector.x >0)
			{
			    angle = 90;			    
			}
			else
			{
				angle = 270;
			}
		}
		
		return angle;
      }
}
