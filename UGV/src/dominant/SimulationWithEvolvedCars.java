package dominant;

        import modeling.COModelWithUI;
        import modeling.COModelWithoutUI;
        import sim.display.Console;
        import modeling.EvolvedCarReader;

        import java.io.File;
        import java.io.FileNotFoundException;
        import java.io.FileOutputStream;
        import java.io.PrintStream;
        import java.security.SecureRandom;


public class SimulationWithEvolvedCars {

    /**
     * @param args
     */

    public static void main(String[] args)
    {
        //works with COModelWithoutUI, need to find what passed in params do to set up evo run

        String evolvedCarName = args[0];

        EvolvedCarReader.getInstance().setName(evolvedCarName);

        if (evolvedCarName.equals("test")) {
            COModelWithUI vid = new COModelWithUI();
            Console c = new Console(vid);
            c.setVisible(true);
        }else {

            COModelWithoutUI vid = new COModelWithoutUI(1, 0);
            long ExternalSeed = Math.round(new SecureRandom().nextInt());
            vid.runBatch(1, ExternalSeed, 0);


        }


    }

}