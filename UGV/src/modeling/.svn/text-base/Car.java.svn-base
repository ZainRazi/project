package modeling;
//MASON imports
import sim.engine.*;
import sim.field.continuous.*;
import sim.util.*;

/**
 *
 * @author Robert Lee
 */
public class Car extends Entity
{
	//parameters for car movement
	private double direction = 0; //will be a value between 0(inc) and 360(exc)
	private double speed = 0; //the speed the vehicle is travelling at
	private CarPerformance performance;//the set performance for the car;
	public boolean isActive= true;
	//private static int noActiveCars = COModel.noCars;
	
	//parameters for navigation
	private int targetID;
	
	//parameters for sensors
	private double viewingRange = 10; //how many units in front of the car it can see obstacles
	private double viewingAngle = 90; //this is the angle for the viewing in front of the car, viewingAngle / 2 in both directions from right in front of the car
	private final double sensitivityForCollisions = 0.5; //this is used to see if the car will collide with obstacles on it's current heading
	
	//parameters for recording information about sim
	private double distanceToDanger = Double.MAX_VALUE; //records the closest distance to danger experienced by the car

	private COModel sim;
	


	public Car(int idNo, int idTarget, CarPerformance performance)
	{
		super(idNo, TCAR);
		this.targetID = idTarget;
		this.performance = performance;
	}
	
	
	/**
	 * Method which calculated the stopping distance of the car at a particular speed
	 * 
	 * @param speed Value of speed which the stopping distance is calculated for
	 * @return the stopping distance of the vehicle at that speed
	 */
//	private double stoppingDistance(double speed)
//	{
//		double dist = 0;
//		double s = speed;
//		
//		s -= performace.getCurrentMaxDecel();
//		
//		while (s > 0)
//		{
//			//loop until stopped
//			dist += s;
//			s -= performace.getCurrentMaxDecel();
//		}
//		
//		return dist;
//	}
	

	@Override
	public void step(SimState state)
	{
		if(this.isActive == true)
		{
			sim = (COModel) state;
			Continuous2D environment = sim.environment;
			
			Double2D me = environment.getObjectLocation(this);
			MutableDouble2D sumForces = new MutableDouble2D(); //used to record the changes to be made to the location of the car

			Double2D targetCoor= me;
			double moveV; //verticle component of the cars movement
			double moveH; //horizontal component of the cars movement
			
			//get location of target
			Bag everything = environment.getAllObjects(); //this will get all of the objects in the world, then start filtering :)
			Bag obstacles = new Bag();
			
			this.performance = new CarPerformance(sim.getCarMaxSpeed(),sim.getCarMaxAcceleration(), sim.getCarMaxDecceleration(), sim.getCarMaxTurning());
			//System.out.println("Car.step is called, car"+this.getID()+"'s coordinate: ("+ me.x+" , "+me.y+")");
	        
			Entity e;
			
			Entity eTarget = new Entity(-1, TOTHER); //this id for the target is illegal, to get ids one should use COModel.getNewID()
					
			for(int i = 0; i < everything.size(); i++)
			{
				e = (Entity) everything.get(i);			
				if (e.getID() == targetID)
				{
					eTarget =  e;
					targetCoor = eTarget.getLocation();
				} else if (e.getType() == TCIROBSTACLE) {
					obstacles.add(e);
				}
			}
			
			dealWithTerrain();
					
			//see if on course to target
			if (direction == calculateAngle(me, targetCoor))
			{
				if (checkCourse(obstacles, direction)) 
				{
					//only check to see if the car is to hit something if it onto it's
					//target course, in the case that it isn't on course then it may turn out of
					//the way of things in it's current path
					int wpID =sim.getNewID();
					alterCourse(obstacles, me, wpID);						
				}
			}
			
			if (me.distance(targetCoor) > 3)
			{
				changeSpeed(ACCELERATE);
				setDirection(me, targetCoor);
			} else {
				if (eTarget.getType() == TTARGET)
				{
					changeSpeed(DECELERATE);
				}
			} 
			
			if (me.distance(targetCoor) < 1) {
				
				if (eTarget.getID() == -1)
				{
					//flag an error as -1 is an illegal id so at this point it can only be that there isn't
					//an existing target for the car
					//System.out.println("Car"+this.getID()+"arrived at fake destination, die!");
					//System.out.println("Ending Sim after failure to read from target");
					//sim.schedule.clear();
					this.isActive = false;
					
				} else {
					if (eTarget.getType() == TTARGET)
					{
						//System.out.println("Car"+this.getID()+"arrived at destination, the closest to danger the car got was " + Double.toString(distanceToDanger));
						//System.out.println("Ending Sim at destination the closest to danger the car got was " + Double.toString(distanceToDanger));
						//sim.schedule.clear();
						this.isActive = false;
						
					} else if (eTarget.getType() == TWAYPOINT) {
						//get rid of wp and get new target ID
						//System.out.println("Car"+this.getID()+"gets a new target and removing waypoint");
						targetID = ((Waypoint) eTarget).getNextPoint();
						environment.remove(eTarget);
					}
				}			
			}		
			
			//call the operations to calculate how much the car moves in the x
			//and y directions.
			moveV = yMovement(direction, speed);
			moveH = xMovement(direction, speed);
			
			sumForces.addIn(new Double2D(moveH, moveV));	
	        sumForces.addIn(me);
			sim.environment.setObjectLocation(this, new Double2D(sumForces));
			this.setLocation( new Double2D(sumForces));
			
//			if(checkWall() == true)
//			{
//				System.out.println("Car"+this.getID()+"clashes with the wall!");
//				//sim.schedule.clear();
//				this.isActive = false;
//				
//			}
			
			location = new Double2D(sumForces);
//			if (detectCollision(obstacles))
//			{
//				System.out.println("Car"+this.getID()+"has clashed with one of the obstacles!");
//				//sim.schedule.clear();
//				this.isActive = false;
//				
//			} else {
				proximityToDanger(obstacles, location);
//			}
			
			
		}
		if(sim!=null)
		{
			sim.dealWithTermination();
		}
    }
	
	

	
    
	


	//**************************************************************************
	//methods for setting the speed and direction of the car and for moving the car
	
	/**
	 * A method which moves the car in the direction of the target point.
	 * 
	 * @param loc the location of the car
	 * @param targ the target location for the car
	 */
	private void setDirection(Double2D loc, Double2D targ)
	{
		double idealDirection = calculateAngle(loc, targ);
		
		//first the ideal bearing for the car to get to it's target must be calculated
		//System.out.println("x: " + Double.toString(loc.x) + " y: " + Double.toString(loc.y));

		//now based on the ideal bearing for the car to get to it's position it
		//must be determined if the car needs to be changed from the bearing it's
		//on at all
		if (idealDirection != direction)
		{
			//then the course that the car is on needs correcting
			//check if it would be quicker to turn left or right
			double delta = idealDirection - direction;
			if(delta>0)
			{
				if(delta <= 180)
				{
					turnLeft(delta);
				}

				else if (delta >180 )
				{
					turnRight(360 - delta);
				}
				
			}
			else
			{
				if (delta >= -180)
				{
					turnRight(-delta);
				}
				else
				{
					turnLeft(360+delta);
				}
			}
			
		}		
	}


	/**
	 * Calculates the bearing the vehicle should be travelling on to move directly
	 * from a location to another.
	 * 
	 * @param point1
	 * @param point2
	 * @return 
	 */
	private double calculateAngle(Double2D point1, Double2D point2)
	{
		Double2D vector = point2.subtract(point1);
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
	
	
	/**
	 * A method which turns the car to the left towards a given bearing.
	 * 
	 * @param bearing the bearing the car is turning onto
	 */
	private void turnLeft(double theta)
	{
		if(theta <= this.performance.getCurrentMaxTurning())
		{
			direction += theta;
		}
		else
		{
			direction += this.performance.getCurrentMaxTurning();
		}
		this.direction = correctAngle(direction);
	}
	
	
	/**
	 * A method which turns the car to the right towards a given bearing.
	 * 
	 * @param bearing the bearing the car is turning onto
	 */
	
	private void turnRight(double theta)
	{
		if(theta <= this.performance.getCurrentMaxTurning())
		{
			direction -= theta;
		}
		else
		{
			direction -= this.performance.getCurrentMaxTurning();
		}
		
		this.direction = correctAngle(direction);
	}
	
	
//	private void turnRight(double bearing)
//	{
//		if (direction > bearing) {bearing += 360;} //correct cases wrapping around 360
//		
//		if (bearing > (direction + performace.getCurrentMaxTurning()))
//		{
//			//the target bearing is more right than can be turned to in one step
//			direction += performace.getCurrentMaxTurning();
//		} else {
//			direction = bearing;
//		}
//		
//		direction = correctBearing(direction);
//	}
	
	
	/** 
	 * A method which changes a bearing to be in the range of 0 (inclusive) to 360 (exclusive)
	 * 
	 * @param b the bearing to be corrected
	 * @return a bearing equivalent to b which has been converted to be in the correct range
	 */
	private double correctAngle(double b)
	{
		if (b >= 360)
		{
			return (b - 360);
		}
		
		if (b < 0)
		{
			return (b + 360);
		}
		
		return b;
	}
	
	
    /**
     * A function which based on the direction the car is facing and the speed it
	 * is travelling at 
	 * it returns a value for how much the x position should change in one step.
	 * 
	 * @param speed the speed
     * @return the change in x coordinate of the car in the world
     */
	private double xMovement(double angle, double speed)
	{
		double xChange;
		
		if (angle <= 90) 
		{
			xChange = (speed * Math.sin(Math.toRadians(angle)));
		} else if (angle <= 180) {
			xChange = (speed * Math.sin(Math.toRadians(180 - angle)));
		} else if (angle <= 270) {
			xChange = (-1 * speed * Math.cos(Math.toRadians(270 - angle)));
		} else {
			xChange = (-1 * speed * Math.sin(Math.toRadians(360 - angle)));
		}	
		return xChange;
    }
	
    
	/**
	 * The y axis equivalent of the xMovement method
	 * 
	 * @return the change in y coordinate of the car in the world
	 */
	private double yMovement(double angle, double speed)
	{
		double yChange;
		if (angle <= 90) 
		{
			yChange = (speed * Math.cos(Math.toRadians(angle)));
		} else if (angle <= 180) {
			yChange = (-1 * speed * Math.cos(Math.toRadians(180 - angle)));
		} else if (angle <= 270) {
			yChange = (-1 * speed * Math.sin(Math.toRadians(270 - angle)));
		} else {
			yChange = (speed * Math.cos(Math.toRadians(360 - angle)));
		}	
		return yChange;
    }
	
	
	/**
	 * A method which increases the speed of the vehicle as much as possible until
	 * it reaches a defined maximum speed.
	 */
	private void changeSpeed(boolean accelerate)
	{
		if (accelerate == true)
		{
			//the car is accelerating
			if (speed <= performance.getCurrentMaxSpeed())
			{
				//then continue to speed up
				if ((speed + performance.getCurrentMaxAccel()) < performance.getCurrentMaxSpeed())
				{
					speed += performance.getCurrentMaxAccel();
				} else {
					speed = performance.getCurrentMaxSpeed();
				}
			} else if (speed > performance.getCurrentMaxSpeed()) {
				//prevent car travelling over maximum speed - useful if car moves into terrain which lowers max speed
				changeSpeed(DECELERATE);
			}
		} else {
			//then the car is to decelerate
			speed -= performance.getCurrentMaxDecel();
		
			if (speed < 0)
			{
				//stop the car moving at a minus speed when it's trying to slow down
				//reverse will have to be implemented separately
				speed = 0;
			}
		}
	}
	
	
	//**************************************************************************
	//methods for viewing and then dodging other Entities
	
	/**
	 * Detects if the car has hit any of the obstacles in the Bag passed to it
	 * 
	 * @param obstacles a bag containing all of the obstacles in the environment
	 */
	private boolean detectCollision(Bag obstacles)
	{
		return sim.obstacleAtPoint(location, obstacles);
	}
	
	
	/**
	 * A method which measures how far away the closest obstacle to the car is
	 * 
	 * @param obstacles 
	 */
	private void proximityToDanger(Bag obstacles, Double2D coord)
	{
		double check;
		
		for (int i = 0; i < obstacles.size(); i++)
		{
			check = ((Obstacle) obstacles.get(i)).obstacleToPoint(coord);
			if (check < distanceToDanger)
			{
				distanceToDanger = check;
			}
		}
		
	}
	
	
	/**
	 * Checks to see if the car from it's current position and heading can see the
	 * Entity it is passed.
	 * 
	 * @param e
	 * @return value which reflects if the car can see the entity or not.
	 */
//	private boolean canSee(Entity e)
//	{
//		Double2D eLocation = e.getLocation();
//		double lowerLimit;
//		double upperLimit;
//		double bearingToObstacle; //this only needs calculating if the target is within range of the car
//
//		if (myLocation.distance(eLocation) < viewingRange)
//		{
//			//the entity is within the range of things that can be seen by the car
//			
//			//must calculate bearing to obstacle
//			bearingToObstacle = calculateBearing(myLocation, eLocation);
//			lowerLimit = direction - (viewingAngle / 2);
//			upperLimit = direction + (viewingAngle / 2);
//			
//			//must work out if the edges of the viewing angle will cross the 0/360 line
//			if ((upperLimit > 360) && (((lowerLimit < bearingToObstacle) && (bearingToObstacle < 360)) || 
//					((0 < bearingToObstacle) && (bearingToObstacle < (upperLimit - 360)))))
//			{
//				return true;
//			} else if ((lowerLimit < 0) && (((0 < bearingToObstacle) && (bearingToObstacle < upperLimit)) || 
//					(((lowerLimit + 360) < bearingToObstacle) && (bearingToObstacle < 360)))) {
//				return true;
//			} else if ((lowerLimit < bearingToObstacle) && (bearingToObstacle < upperLimit)) {
//				return true;
//			}			
//			//if within the limits then the target can be seen by the car
//		}
//		
//		return false;
//	}
	
	
	/**
	 * Method which adds a Waypoint for the vehicle to travel via on it's path to
	 * prevent it from hitting an obstacle in it's way
	 * 
	 * @param obstacles All of the obstacles in the environment
	 * @param me location of the vehicle
	 * @param wpID id for the Waypoint
	 */
	public void alterCourse(Bag obstacles, Double2D me, int wpID)
	{
		double resolution = 0.5;
		MutableDouble2D coord = new MutableDouble2D(me);
		Waypoint wp;
		double xComponent;
		double yComponent;
	
		
		for(double i = 0; i < (performance.getCurrentMaxTurning() - 5); i += resolution)
		{
			if (checkCourse(obstacles, correctAngle(direction - i)) == false)
			{
				//then moving right gives a clear path
				//set wp and return
				
				//first must find out where to put wp
				xComponent = xMovement(correctAngle(direction - (i+5)), (viewingRange / 1));
				yComponent = yMovement(correctAngle(direction - (i+5)), (viewingRange / 1));
				coord.addIn(xComponent, yComponent);
				wp = new Waypoint(wpID, targetID);
				wp.setLocation(new Double2D(coord));
				targetID = wpID;
				sim.environment.setObjectLocation(wp, new Double2D(coord));
				return;
				
			} else if (checkCourse(obstacles, correctAngle(direction + i)) == false) {
				//then moving left gives a clear path
				//set wp and return
				
				xComponent = xMovement(correctAngle(direction + (i+5)), (viewingRange / 1));
				yComponent = yMovement(correctAngle(direction + (i+5)), (viewingRange / 1));
				coord.addIn(xComponent, yComponent);
				wp = new Waypoint(wpID, targetID);
				wp.setLocation(new Double2D(coord));
				targetID = wpID;
				sim.environment.setObjectLocation(wp, new Double2D(coord));
				return;
			}
		}
		
		//no path that can be immediately turned onto is clear
		//therefore see if it is possible for the car to see a clear path even
		//if it can't be immediately turned onto
		for(double i = (performance.getCurrentMaxTurning()-5); i < (viewingAngle / 2); i += resolution)
		{
			if (checkCourse(obstacles, correctAngle(direction - i)) == false)
			{
				//then moving right gives a clear path
				//set wp and return
				
				//first must find out where to put wp
				
				xComponent = xMovement(correctAngle(direction - (performance.getCurrentMaxTurning()+5)), (viewingRange / 1));
				yComponent = yMovement(correctAngle(direction - (performance.getCurrentMaxTurning()+5)), (viewingRange / 1));
				coord.addIn(xComponent, yComponent);
				wp = new Waypoint(wpID, targetID);
				wp.setLocation(new Double2D(coord));
				targetID = wpID;
				sim.environment.setObjectLocation(wp, new Double2D(coord));
				return;
				
			} else if (checkCourse(obstacles, correctAngle(direction + i)) == false) {
				//then moving left gives a clear path
				//set wp and return
				
				xComponent = xMovement(correctAngle(direction + (performance.getCurrentMaxTurning()+5)), (viewingRange / 1));
				yComponent = yMovement(correctAngle(direction + (performance.getCurrentMaxTurning()+5)), (viewingRange / 1));
				coord.addIn(xComponent, yComponent);
				wp = new Waypoint(wpID, targetID);
				wp.setLocation(new Double2D(coord));
				targetID = wpID;
				sim.environment.setObjectLocation(wp, new Double2D(coord));
				return;
			}
		}
	}


	//this method tests a course to see if any of the obstacles in the bag will be hit
	//by the car if it moves from it's position on the bearing provided
	/**
	 * 
	 * @param obstacles
	 * @param bearing
	 * @return true if going to hit something in obstacles, false if not
	 */
	private boolean checkCourse(Bag obstacles, double bearing)
	{
		for(int i = 0; i < obstacles.size(); i++)
		{
			if (onCourse((Obstacle) obstacles.get(i), bearing))
			{
				return true;
			}
		}
		
		return false;
	}
	
	
	/** this method will analyse an obstacle and will see if the car will hit it
	 *  on the course specified as bearing
	 * 
	 * @param o
	 * @param bearing
	 * @return true if going to hit o on provided course (as far as it can see) false if not
	 */
	private boolean onCourse(Obstacle o, double bearing)
	{
		//simple and dirty method which checks the coordinates between 0 and 
		//the viewing range away from the target in certain increments and see 
		//if they're in the obstacle
		MutableDouble2D testCoord = new MutableDouble2D();
		Double2D amountAdd = new Double2D(xMovement(bearing, sensitivityForCollisions), yMovement(bearing, sensitivityForCollisions));
		testCoord.addIn(location);
		
		for(double i = 0; i < viewingRange; i += sensitivityForCollisions)
		{
			//keep adding the amountAdd on and seeing if the coordinate is in the obstacle o
			//going to need to change obstacles to be a subset of entities now so that one 
			//can use the inShape with all of them
			if (o.inShape(new Double2D(testCoord)))
			{
				return true; //the testing doesn't need to continue if it would hit the obstacle at one point
			}
			testCoord.addIn(amountAdd);
			
		}
		
		return false; //the car does not hit the obstacle at any point it can see on it's current course
	}
	
	
	/**
	 * A method which checks the terrain that the car is currently in and then
	 * changes it's properties as required based on the terrain it is currently
	 * in
	 */
	private void dealWithTerrain()
	{
		int type = sim.terrainAtPoint(location);
		switch (type)
		{
			case NORMAL:
				//normal terrain
				performance.reset();
				break;
			case GRAVEL:
				//driving on gravel
				performance.reset();
				performance.setCurrentMaxSpeed(performance.getCurrentMaxSpeed() / 2);
				break;
			case ICE:
				//driving on ice
				performance.reset();
				performance.setCurrentMaxAcceleration(performance.getCurrentMaxAccel() / 2);
				performance.setCurrentMaxDeceleration(performance.getCurrentMaxDecel() / 2);
				performance.setCurrentMaxTurning(performance.getCurrentMaxTurning() / 2);				
				break;
			default:
				performance.reset();
				break;
		}
	}
	
	private boolean checkWall()
	{
		Double2D me = sim.environment.getObjectLocation(this);
		
		if(me.x <= 0.2 )
		{
			//System.out.println("clash with the left wall!");
			return true;
		}
		else if(100 - me.x <= 0.2)
		{
			//System.out.println("clash with the right wall!");
			return true;
		}
		
		if (me.y <= 0.2)
		{
			//System.out.println("clash with the upper wall!");
			return true;
		}
		else if(100- me.y <= 0.2)
		{
			//System.out.println("clash with the lower wall!");
			return true;
		}
		
		return false;
	}


	public CarPerformance getStats() {
		return performance;
	}


	public void setStats(CarPerformance stats) {
		this.performance = stats;
	}
}
