package modeling;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;

import sim.engine.SimState;
import sim.engine.Steppable;
import sim.util.Bag;
import sim.util.Double2D;


/**
 * 
 */

/**
 * @author xueyi
 *
 */
public class AccidentDetector implements Constants,Steppable {

	/**
	 * 
	 */
	private File accidentLog = new File("AccidentLog.txt");
	private COModel sim;
	private PrintStream ps;
	private Bag trackedCars=new Bag();
	private int noAccidents=0;
	
	
	public int getNoAccidents() {
		return noAccidents;
	}

	public void setNoAccidents(int noAccidents) {
		this.noAccidents = noAccidents;
	}

	public Bag getTrackedCars() {
		return this.trackedCars;
	}

	public void setTrackedCars(Bag trackedCars) {
		this.trackedCars = trackedCars;
	}

	public AccidentDetector(){
		// TODO Auto-generated constructor stub
		
		
		try{
			ps= new PrintStream(new FileOutputStream(accidentLog));
		}
		catch(FileNotFoundException e)
		{
			System.out.print("File not found!");
			return;
		}
				
	}

	/* (non-Javadoc)
	 * @see sim.engine.Steppable#step(sim.engine.SimState)
	 */
	@Override
	public void step(SimState state) {
		// TODO Auto-generated method stub
		sim = (COModel)state;
		Obstacle obstacle;
		//Bag cars = COModel.cars;
		Car car1;
		Car car2;
		for (int i=0; i<trackedCars.size(); i++)
		{
			car1= (Car)trackedCars.get(i);
			for(int j=0; j<sim.obstacles.size(); j++)
			{
				obstacle=(Obstacle)sim.obstacles.get(j);
				//System.out.println("Obstacle"+obstacle.getID());
				if(detectCollisionWithObstacle(car1, obstacle))
				{
					addLog(AccidentType.CLASHWITHOBSTACLE, car1.getID(), sim.schedule.getSteps(), car1.getLocation(), "with octacle id = "+ obstacle.getID() );
					noAccidents++;
					car1.isActive=false;
					trackedCars.remove(car1);
				}
			}
			
			
			if(detectCollisionWithWall(car1))
			{
				addLog(AccidentType.CLASHWITHWALL, car1.getID(), sim.schedule.getSteps(), car1.getLocation(), null);
				noAccidents++;
				car1.isActive=false;
				trackedCars.remove(car1);
			}
			
//			for (int j=0; j<sim.cars.size(); j++)
//			{
//				
//				car2= (Car)sim.cars.get(j);
//				if(car2 == car1)
//				{
//					continue;
//				}
//				else if (detectCollisionWithOtherCar(car1, car2))
//				{
//					addLog(AccidentType.CLASHWITHOTHERCAR, car1.getID(), sim.schedule.getSteps(), car1.getLocation(), null );
//					noAccidents++;
//					car1.isActive=false;
//					car2.isActive=false;
//					trackedCars.remove(car1);
//					trackedCars.remove(car2);
//				}
//			}
			
		}
		
		
		

	}
	
	public void addLog(AccidentType t, int carID, long step, Double2D coor, String str)
	{
		ps.println(t.toString() +":car"+carID + "; time:"+step+"steps; location: ("+coor.x+" , "+coor.y+")" + str);
	}
	
	private boolean detectCollisionWithObstacle(Car car, Obstacle obstacle)
	{
		return obstacle.inShape(car.getLocation());
	}
	
	private boolean detectCollisionWithWall(Car car)
	{
        Double2D me = car.getLocation();
		
		if(me.x <= 0.5 )
		{
			//System.out.println("clash with the left wall!");
			return true;
		}
		else if(100 - me.x <= 0.5)
		{
			//System.out.println("clash with the right wall!");
			return true;
		}
		
		if (me.y <= 0.5)
		{
			//System.out.println("clash with the upper wall!");
			return true;
		}
		else if(100- me.y <= 0.5)
		{
			//System.out.println("clash with the lower wall!");
			return true;
		}
		
		return false;
	}
	
	private boolean detectCollisionWithOtherCar(Car car1, Car car2)
	{
		Double2D location1=car1.getLocation();
		Double2D location2=car2.getLocation();
		if(location1.distance(location2) < 1)
		{
			return true;
		}
		else
		{
			return false;
		}
		
		
	}

}
