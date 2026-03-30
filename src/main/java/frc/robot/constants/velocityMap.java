package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class velocityMap {
    private static velocityMap vMap = null;
    public InterpolatingDoubleTreeMap mainMap = new InterpolatingDoubleTreeMap();

    public velocityMap(){
        // added 59.69cm offset to center of hub
        mainMap.put(1.6, 1450.0);
        mainMap.put(1.8, 1550.0);
        mainMap.put(2.1,1630.0);
        //2.3
        mainMap.put(2.5,1650.0);
        //2.7
        mainMap.put(2.95,1750.0);
        mainMap.put(3.2,1800.0);
        //3.4
        mainMap.put(3.6,1900.0);
        mainMap.put(3.8,2050.0);
        //4
    }

    public static velocityMap getInstance(){
        if (vMap == null){
            vMap = new velocityMap();
        }
        return vMap;
    }
}
