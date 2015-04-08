package modeling;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.io.FileOutputStream;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import sim.util.*;
/**
 *
 * @author Robert Lee
 * This class is used to build/initiate the simulation.
 * There is a "main" method for running the simulation without GUI
 * Called for by simulationWithUI.class
 */
public class COModelBuilder
{
	private static String simLength = "1000";
	
	public  COModel sim;
	
	private static double worldXVal = 100;
	private static double worldYVal = 100;
	//private int noCars=3;
	
		
	public COModelBuilder()
	{
		//System.out.println("SimBuilder1 is being called!!!!!!!!!!");
		setUpSim((Calendar.SECOND * 1000)+ Calendar.MILLISECOND);
		
		
	
	}
	
	
	public COModelBuilder(COModel s)
	{
		//System.out.println("SimBuilder2 is being called!!!!!!!!!! ");
		sim = s;
		
		
	}
	
	
	private void setUpSim(long initSeed)
	{
		System.out.println("COModelBuilder.setUpSim is called!!!!!!!! ");
		sim = new COModel(initSeed, worldXVal, worldYVal, false);
		
	}
	
	

	
//	public static void testSim()
//	{
//		int tID = sim.getNewID();
//		Target t = new Target(tID);
//		entAdder(t, 0, 0, false);
//		
//		CircleObstacle ob = new CircleObstacle(sim.getNewID(), 10);
//		entAdder(ob, 50, 50, false);		
//		
//		Car AV = new Car(sim.getNewID(), tID);
//		entAdder(AV, 99, 99, true);
//	}
//	
//	
//	public static void testSim2()
//	{
//		int tID = sim.getNewID();
//		double xVal = sim.random.nextDouble() * 100;
//		double yVal = sim.random.nextDouble() * 100;
//		
//		Target t = new Target(tID);
//		entAdder(t, xVal, yVal, false);
//		
//		CircleObstacle ob = new CircleObstacle(sim.getNewID(), 10);
//		entAdder(ob, 50, 50, false);		
//		
//		if (xVal < 50)
//		{
//			xVal += 50;
//		} else if (xVal > 50) {
//			xVal -= 50;
//		}
//		
//		if (yVal < 50)
//		{
//			yVal += 50;
//		} else if (xVal > 50) {
//			yVal -= 50;
//		}
//		
//		Car AV = new Car(sim.getNewID(), tID);
//		entAdder(AV, xVal, yVal, true);		
//	}
	
	
	public  void generateSimulation(int noObstacles, int noCars)
	{		
		Bag obstacles = sim.obstacles;
		double x;
		double y;		
		
		for (int i = 0; i < noObstacles; i++)
		{
			x = sim.random.nextDouble() * worldXVal;
			y = sim.random.nextDouble() * worldYVal;
			CircleObstacle ob = new CircleObstacle(sim.getNewID(), sim.random.nextInt(9) + 1);
			ob.setLocation(new Double2D(x,y));
			ob.isSchedulable = false;
			sim.allEntities.add(ob);
			obstacles.add(ob);
			//System.out.println("obstacle " + Integer.toString(i) + " is at (" + Double.toString(x) + ", " + Double.toString(y) + "), ID is"+ ob.ID);
		}
		
		
		int tID = sim.getNewID();
		Target t = new Target(tID);
		do
		{
			x = sim.random.nextDouble() * worldXVal;
			y = sim.random.nextDouble() * worldYVal;
		}  while (sim.obstacleAtPoint(new Double2D(x,y), obstacles));
		//entAdder(t, x, y, false);
		t.setLocation(new Double2D(x,y));
		t.isSchedulable = false;
		sim.allEntities.add(t);
		//System.out.println("target is at (" + Double.toString(x) + ", " + Double.toString(y) + ") ID is"+ t.ID);
		
		//Bag cars=new Bag();
		for(int i=1; i<=noCars; i++)
		{
			do
			{
				x = sim.random.nextDouble() * worldXVal;
				y = sim.random.nextDouble() * worldYVal;
			}  while (sim.obstacleAtPoint(new Double2D(x,y), obstacles));
			Car car = new Car(sim.getNewID(), tID, sim.carStats);	
			sim.cars.add(car);
			//entAdder(car, x, y, true);	
			car.setLocation(new Double2D(x,y));
			car.isSchedulable = true;
			sim.allEntities.add(car);
			sim.toSchedule.add(car);
			//System.out.println("car is at (" + Double.toString(x) + ", " + Double.toString(y) + ")ID is"+ car.ID);
			//System.out.println("COModelBuilder.genereteSimulation is called, car's max speed is: " + sim.carStats.getMaxSpeed());
		}
		
		
//		System.out.println("Simultaion stepping begins!");
//		System.out.println("==============================================================");
//		
		
		
	}
	
	
	/**
	 * A method which can be used with constructing a random simulation, this method
	 * will add a specified number of waypoints to a simulation which will give the 
	 * car a route to follow.
	 * 
	 * @param target the id of the eventual target of the car
	 * @param noWaypoints the number of waypoints to randomly add
	 * @return the id number of the first waypoint for the car to reach
	 */
//	private int addWaypoints(int target, int noWaypoints)
//	{
//		int newTarget = target;
//		double x;
//		double y;
//		
//		for (int i = 0; i < noWaypoints; i++)
//		{
//			x = sim.random.nextDouble() * worldXVal;
//			y = sim.random.nextDouble() * worldYVal;
//			newTarget = sim.getNewID();
//			Waypoint wp = new Waypoint(newTarget, target);
//			entAdder(wp, x, y, false);
//		}
//		
//		return newTarget;
//	}
	
	
	/**
	 * A method which contructs a Double2D coordinate from two values
	 * 
	 * @param x the x component of the coordinate
	 * @param y the y component of the coordinate
	 * @return the contructed Double2D coordinate
	 */
//	private static Double2D coordBuilder(double x, double y) {return new Double2D(x, y);}
	
	
	/**
	 * A method which adds an entity to the list of entities to be added to the
	 * environment when the simulation begins.
	 * 
	 * @param e the entity to be added
	 * @param x the x component of it's location
	 * @param y the y component of it's location
	 * @param schedulable a true if the entity needs to be added to the schedule, false if it doesn't
	 */
//	private void entAdder(Entity e, double x, double y, boolean schedulable)
//	{
//		Double2D loc = coordBuilder(x, y);
//		sim.addEntity(e, loc, schedulable);
//	}
//	

	
	
	/**
	 * 
	 * @param args
	 * @return 
	 */
	public static String[] addEndTime(String[] args)
	{
		String[] x = new String[args.length + 2];
		
		if (args.length != 0)
		{
			x = args;
		}
		
		x[args.length] = "-for";
		x[args.length + 1] = simLength;
		
		return x;
	}
	
	public COModel getSim() {return sim;}
	

	
}
