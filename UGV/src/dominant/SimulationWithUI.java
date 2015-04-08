/**
 * 
 */
package dominant;

import modeling.COModelWithUI;
import sim.display.Console;

import modeling.EvolvedCarReader;

/**
 * @author xueyi
 *
 */
public class SimulationWithUI {

	/**
	 * @param args
	 */
	
    public static void main(String[] args)
    {
    	//System.out.println("Before we do anything: Free =" + Runtime.getRuntime().freeMemory());
    	//System.out.println("Before we do anything: Max =" + Runtime.getRuntime().maxMemory());
    	//System.out.println("Before we do anything: Total =" + Runtime.getRuntime().totalMemory());

        //works with COModelWithoutUI, need to find what passed in params do to set up evo run

        EvolvedCarReader reader = EvolvedCarReader.getInstance();
        reader.setName("");

    	COModelWithUI vid = new COModelWithUI();
    	Console c = new Console(vid);
		c.setVisible(true);

    }

}
