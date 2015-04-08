package modeling;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import sim.util.*;
import sim.field.continuous.*;
import sim.engine.*;
import sim.field.grid.IntGrid2D;

public class COModel extends SimState
{
	public Bag toSchedule = new Bag();
	public Bag allEntities = new Bag();
	public Bag cars = new Bag();
	public Bag obstacles= new Bag();
	
	public int noObstacles=18;
	public int noCars=4;
	
	
	private boolean runningWithUI = false; 
	
    private int newID = 0;	
    
	private double xDouble;
	private double yDouble;	
	
    public Continuous2D environment;
	
	public IntGrid2D obstacleMap;
	private int obstacleMapResolution = 3; //multiplier used to change resolution of obstacles image

	public IntGrid2D terrainMap;
	private int terrainMapResolution = 3;
	
	public IntGrid2D wallMap;
	private int wallMapResolution =3;
	
	private double carMaxSpeed=1.0;
	private double carMaxAcceleration = 0.05;
	private double carMaxDecceleration = 0.05;
	private double carMaxTurning = 15;
	
	public CarPerformance carStats;
	
	public AccidentDetector aDetector= new AccidentDetector();
	
	/**
	 * Constructor method used in the simState.doLoop process for running the simulation
	 * 
	 * @param seed for random number generator
	 */
//	public COModel(long seed)//, int noCars, int noObstacles)
//	{
//		super(seed);
//		environment = new Continuous2D(1.0, xDouble, yDouble);
//		obstacleMap = new IntGrid2D((int) (xDouble * obstacleMapResolution), (int) (yDouble * obstacleMapResolution), 0);
//		terrainMap = new IntGrid2D((int) (xDouble * terrainMapResolution), (int) (yDouble * terrainMapResolution), 0);
//		wallMap = new IntGrid2D((int) (xDouble * wallMapResolution), (int) (yDouble * wallMapResolution), 0);
//		carStats = new CarPerformance(carMaxSpeed, carMaxAcceleration, carMaxDecceleration, carMaxTurning);	
////		this.noCars= noCars;
////		this.noObstacles= noObstacles;
//		System.out.println("Simulation1 is being called!!!!!!!!!!!");
//	}
	
	/**
	 * Constructor used for setting up a simulation from the COModelBuilder object.
	 * 
	 * @param seed for random number generator
	 * @param x the width of the simulation environment
	 * @param y the height of the simulation environment
	 * @param UI pass true if the simulation is being ran with a UI false if it is not.
	 */
    public COModel(long seed, double x, double y, boolean UI)//,int noCars, int noObstacles)
    {
		super(seed);
		environment = new Continuous2D(1.0, x, y);
		xDouble = x;
		yDouble = y;
		obstacleMap = new IntGrid2D((int) (xDouble * obstacleMapResolution), (int) (yDouble * obstacleMapResolution), 0);
		terrainMap = new IntGrid2D((int) (xDouble * terrainMapResolution), (int) (yDouble * terrainMapResolution), 0);
		wallMap = new IntGrid2D((int) (xDouble * wallMapResolution), (int) (yDouble * wallMapResolution), 0);
		runningWithUI = UI;
		carStats = new CarPerformance(carMaxSpeed, carMaxAcceleration, carMaxDecceleration, carMaxTurning);
//		this.noCars= noCars;
//		this.noObstacles= noObstacles;
//		System.out.println("Simulation2 is being called!!!!!!!!!!! the model is: "+ this.toString());
	}
    
	
	
	
	public  double getCarMaxSpeed() {
		return carMaxSpeed;
	}


	public  void setCarMaxSpeed(double MaxSpeed) {
		carMaxSpeed = MaxSpeed;
	}

	
	public  double getCarMaxAcceleration() {
		return carMaxAcceleration;
	}


	public  void setCarMaxAcceleration(double MaxAcceleration) {
		carMaxAcceleration = MaxAcceleration;
	}


	public  double getCarMaxDecceleration() {
		return carMaxDecceleration;
	}


	public  void setCarMaxDecceleration(double MaxDecceleration) {
		carMaxDecceleration = MaxDecceleration;
	}


	public  double getCarMaxTurning() {
		return carMaxTurning;
	}


	public  void setCarMaxTurning(double MaxTurning) {
		carMaxTurning = MaxTurning;
	}
	
	public int getNoObstacles() {
		return this.noObstacles;
	}

	public void setNoObstacles(int noObstacles) {
		this.noObstacles = noObstacles;
	}

	public int getNoCars() {
		return this.noCars;
	}

	public void setNoCars(int noCars) {
		this.noCars = noCars;
	}


	
	
	public void start()
	{
		super.start();	
		environment.clear();
		
		loadEntities();
		scheduleEntities();
		//if (runningWithUI)
		{
			buildObstacleMap(); //draw the map of integers for obstacle representation
			buildTerrainMap();
			buildWallMap();
		}
	}
	
	
//	/**
//	 * A method which constructs an example simulation, this exists for testing
//	 * purposes and can be removed without affecting the COModel class.
//	 */
//	private void exampleSim()
//	{
//		Double2D targ = new Double2D(0,0);
//
//		//Add target to the environment at a random location
//		int targetID = getNewID();
//		Target t = new Target(targetID);
//		addEntity(t, targ, false);
//
//		CircleObstacle b = new CircleObstacle(getNewID(), 4);
//		addEntity(b, new Double2D(50, 50), false);
//
//		Car AV = new Car(getNewID(), targetID);
//		addEntity(AV, new Double2D(100, 100), true);
//	}
//     
//	
	/**
	 * A method which starts the simulation
	 * 
	 * @param args the arguments for the doLoop method
	 */
    public void beginSimulation(String[] args)
    {
   		doLoop(COModel.class, args);

    } 
		
	
	/**
	 * A method which provides a different number each time it is called, this is
	 * used to ensure that different entities are given different IDs
	 * 
	 * @return a unique ID number
	 */
	public int getNewID()
	{
		int t = newID;
		newID++;
		return t;
	}
	
	
	/**
	 * A method which adds all of the entities to the simulations environment.
	 */
	public void loadEntities()
	{
		for(int i = 0; i < allEntities.size(); i++)
		{
			environment.setObjectLocation((Entity) allEntities.get(i), ((Entity) allEntities.get(i)).getLocation());
			//getNewID(); //need to count off the ids inputted to make sure they aren't reused.
		}
		//allEntities.clear();
	}
	
	
	/**
	 * A method which adds an entity to the simulation which is to be ran
	 * 
	 * @param e the entity to be added
	 * @param location the location to add the entity
	 * @param schedule does the entity require being added to the simulations schedule
	 */
//	public void addEntity(Entity e, Double2D location, boolean schedule)
//	{
//		e.setLocation(location);
//		//environment.setObjectLocation(e, location);
//		allEntities.add(e);
//
//		if (schedule == true)
//		{
//			toSchedule.add(e);
//		}
//		
//
//	}
	
	
	/**
	 * A method which adds all the entities marked as requiring scheduling to the
	 * schedule for the simulation
	 */
	public void scheduleEntities()
	{
		//loop across all items in toSchedule and add them all to the schedule
		for(int i = 0; i < toSchedule.size(); i++)
		{
			schedule.scheduleRepeating((Entity) toSchedule.get(i));
		}
		//toSchedule.clear();
		
		try{
			Bag trackedCars = (Bag)this.cars.clone();
			aDetector.setTrackedCars(trackedCars);
		}
		
		catch(CloneNotSupportedException e)
		{
			System.out.print("Clone not supported!");
			return;
		}
		
		
		schedule.scheduleRepeating(aDetector);
	}
	
	
	
	

	
	
	/**
	 * A method which resets the variables for the COModel and also clears
	 * the schedule and environment of any entities, to be called between simulations.
	 * 
	 * This method resets the newID counter so should NOT be called during a run.
	 */
	public void reset()
	{
		newID = 0;
		obstacles.clear();
		cars.clear();
		toSchedule.clear();
		allEntities.clear();
		environment.clear(); //clear the environment
		terrainMap.setTo(Constants.NORMAL);
	}
	
	
	/**
	 * A method which creates a discrete map of where obstacles are in the environment
	 * which can be used for outputting a visual representation of the obstacles
	 * when the simulation is ran with a UI
	 */
	private void buildObstacleMap()
	{
		//[TODO] This will need to be called more than once if the simulations can have moving obstacles
		Bag all = environment.allObjects;
		Bag obstacles = new Bag();
		Double2D coordinate;
		int xInt = (int) (xDouble * obstacleMapResolution);
		int yInt = (int) (yDouble * obstacleMapResolution);
		obstacleMap = new IntGrid2D(xInt, yInt, 0);
		
		for (int i = 0; i < all.size(); i++)
		{
			if (((Entity) (all.get(i))).getType() == Constants.TCIROBSTACLE)
			{
				obstacles.add(all.get(i));
			}
			
		}
		
		for (int i = 0; i < xInt; i++) {
			for (int j = 0; j < yInt; j++)
			{
				coordinate = new Double2D(((double) i) / obstacleMapResolution, ((double) j) / obstacleMapResolution);
				//loop over all coordinates and check if there is an obstacle there
				if (obstacleAtPoint(coordinate, obstacles))
				{
					//System.out.println("setting the point at x:" + Integer.toString(i) + " y:" + Integer.toString(j) + " to 1");
					obstacleMap.field[i][j] = 1;
				}				
			}
		}
	}
	
	
	/**
	 * A method which draws two circles onto the terrain map for different terrain types.
	 */
	public void buildWallMap()
	{
		int xInt = (int) (xDouble * terrainMapResolution);
		int yInt = (int) (yDouble * terrainMapResolution);
		
		for (int i = 0; i < xInt; i++) 
		{
			
			wallMap.field[0][i]=1;
			wallMap.field[yInt-1][i]=1;			
		}
		
		for (int j = 0; j < yInt; j++) 
		{
			
			wallMap.field[j][0]=1;
			wallMap.field[j][xInt-1]=1;			
		}
		
	}
	
	
	public void buildTerrainMap()
	{

		int xInt = (int) (xDouble * wallMapResolution);
		int yInt = (int) (yDouble * wallMapResolution);
		
		for (int i = 0; i < xInt; i++) {
			for (int j = 0; j < yInt; j++)
			{
				if ((((i - (50 * terrainMapResolution)) * (i - (50 * terrainMapResolution))) + ((j - (50 * terrainMapResolution)) * (j - (50 * terrainMapResolution)))) < ((20 * terrainMapResolution) * (20 * terrainMapResolution)))
				{
					terrainMap.field[i][j] = Constants.GRAVEL;
				} else if ((((i - (50 * terrainMapResolution)) * (i - (50 * terrainMapResolution))) + ((j - (50 * terrainMapResolution)) * (j - (50 * terrainMapResolution)))) < ((40 * terrainMapResolution) * (40 * terrainMapResolution))){
					terrainMap.field[i][j] = Constants.ICE;
				} else {
					terrainMap.field[i][j] = Constants.NORMAL;
				}
			}
		}
		
	}
	
	/**
	 * A method which returns a true or false value depending on if an obstacle is
	 * at the coordinate provided
	 * 
	 * @param coord the coordinate to check
	 * @param obstacles the obstacles to be checked
	 * @return 
	 */
	public boolean obstacleAtPoint(Double2D coord, Bag obstacles)
	{
		for (int i = 0; i < obstacles.size(); i++)
		{
			//for all of the obstacles check if the provided point is in it
			if (((Obstacle) (obstacles.get(i))).inShape(coord)) //[TODO] this might not work it depends on if whatever the object is will override inShape
			{
				return true;
			}
		}
		//at this point no cross over has been detected so false should be returned
		return false;
	}
	
	
	/**
	 * A method which checks the terrain at a given location
	 * 
	 * @param coord coordinate to check the terrain at
	 * @return 
	 */
	public int terrainAtPoint(Double2D coord)
	{
		int x = (int)coord.x * terrainMapResolution;
		int y = (int)coord.y * terrainMapResolution;
		int tTerrain =0;
		try
		{
			tTerrain = terrainMap.get(x, y);
		}
		catch(ArrayIndexOutOfBoundsException e)
		{
			System.out.println("too fast to detect the clash with wall accident, go on!");
		}
		return tTerrain;
	}
	
    public void dealWithTermination()
	{
    	int noActiveAgents =0;
    	//System.out.println(COModel.toSchedule.size());
    	for(Object o: this.toSchedule)
    	{
    		if(((Car)o).isActive)
    		{
    			noActiveAgents++;
    		}
    		
    	}
    	//System.out.println("NO. of active cars is: "+ noActiveAgents);
    	
		if(noActiveAgents < 1)
		{
			this.schedule.clear();
			//System.out.println("NO. of active cars is: "+ noActiveAgents+". Game Over!");
			//System.out.println(this.schedule.scheduleComplete());
			this.kill();
		}
	 }
	

//	public static void main(String[] args) throws FileNotFoundException
//	{
//		//setting up the logger
//		///*
//		DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd-HHmmss");
//		Date date = new Date();
//		
//		File file = new File("Run" + dateFormat.format(date) + ".txt");
//		PrintStream logger = new PrintStream(new FileOutputStream(file));
//				
//		//System.setOut(logger);
//		//*/
//		
//		//setting up the simulation
//		long time = System.nanoTime();
//		COModel state= new COModel(time, 100, 100, true);
//		System.out.println("Seed: " + Long.toString(time));
//		state.setNoObstacles(18);
//		state.setNoCars(4);
//				
//		COModelBuilder sBuilder = new COModelBuilder(state);
//		sBuilder.generateSimulation(state.getNoObstacles(),state.getNoCars());
//		state.start();
//		
//		//simB.setUpSim(l);
//				
//		//String[] withEndTime = COModelBuilder.addEndTime(args); //add a max number of steps to the arguments passed to the simulation
//        //int endingSteps = Integer.parseInt(args[1]);
////		System.out.println("Initial seed value: " + Long.toString(l));
////		System.out.println("testSim:");
//		do
//		{
//			if (!state.schedule.step(state))
//			{
//				break;
//			}
//		} while(state.schedule.getSteps()< 1000);
//		
//		state.finish();
//		System.out.println("finished :)");
//		//state.beginSimulation(withEndTime);
//		
//	}
}
