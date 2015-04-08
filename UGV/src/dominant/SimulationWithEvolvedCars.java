package dominant;

        import modeling.COModelWithoutUI;
        import sim.display.Console;
        import modeling.EvolvedCarReader;

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
        COModelWithoutUI vid = new COModelWithoutUI(1,1);
        long ExternalSeed = Math.round(new SecureRandom().nextInt());
        vid.runBatch(1, ExternalSeed, 1);
    }

}