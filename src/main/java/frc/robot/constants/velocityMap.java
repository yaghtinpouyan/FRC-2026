package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class velocityMap {
    private static velocityMap vMap = null;
    public InterpolatingDoubleTreeMap mainMap = new InterpolatingDoubleTreeMap();

    public velocityMap(){
        // added 59.69cm offset to center of hub
        mainMap.put(1.6, 1300.0);
        mainMap.put(1.8, 1400.0);
        mainMap.put(2.1,1480.0);
        //2.3
        mainMap.put(2.5,1500.0);
        //2.7
        mainMap.put(2.95,1600.0);
        mainMap.put(3.2,1650.0);
        //3.4
        mainMap.put(3.6,1750.0);
        mainMap.put(3.8,1900.0);
        //4
    }

    public static velocityMap getInstance(){
        if (vMap == null){
            vMap = new velocityMap();
        }
        return vMap;
    }
}
