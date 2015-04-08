package modeling;

import javax.tools.JavaCompiler;
import javax.tools.ToolProvider;
import java.io.File;
import java.lang.reflect.Constructor;
import java.net.URL;
import java.net.URLClassLoader;

/**
 * Created by zain on 23/03/15.
 */
public class EvolvedCarReader {

    String evolvedCarName;
    DumbCar evolvedCar;
    Class<?> temp;


    private static EvolvedCarReader instance = null;
    protected EvolvedCarReader() {

    }

    public static EvolvedCarReader getInstance() {
        if(instance == null) {
            instance = new EvolvedCarReader();
        }
        return instance;
    }

    public void setName(String name) {


        this.evolvedCarName = name;
    }

    public Class<?> getTemp(){
        return temp;
    }

    public DumbCar readCar(int idNo, CarPerformance performance,
                           double initialBearing) {

        //File root = new File(System.getProperty("user.dir"));
        //File sourceFile = new File(root, name+"/DumbCarImpl.java");


        File root = new File(System.getProperty("user.dir")+"/UGV/evolved/"+this.evolvedCarName+"/");
        File sourceFile = new File(root, "DumbCarImpl.java");

      //  File root = new File("user.home/UGV/evolved/"+this.evolvedCarName+"/");
      //  String path = System.getProperty("user.home")+"/UGV/evolved/"+this.evolvedCarName+"/DumbCarImpl.java";

       // File sourceFile = new File(path);

        //some weird stuff with reflection comment properly later...

        JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
        compiler.run(null, null, null, sourceFile.getPath());
        DumbCar theCar = null;

        File root2 = new File(System.getProperty("user.dir"));

        try {
     //       URLClassLoader classLoader = URLClassLoader.newInstance(new URL[] { root.toURI().toURL() });
            URLClassLoader classLoader = URLClassLoader.newInstance(new URL[] { root2.toURI().toURL() });
            Class<?> cls = Class.forName("DumbCarImpl", true, classLoader);

            temp = cls;

            //define what cls's constructors arguements should be
            Constructor clsCon = cls.getDeclaredConstructor(int.class, CarPerformance.class, double.class);

            //create instance using new constructor
            Object instance = clsCon.newInstance(idNo, performance, initialBearing);


            theCar = (DumbCar)instance;
        }catch (Exception e)

        {
            System.out.print("reader has crashed");
            System.exit(0);
        }

        return theCar;
    }
}
