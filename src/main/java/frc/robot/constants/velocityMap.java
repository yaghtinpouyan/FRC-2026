package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class velocityMap {
    private static velocityMap vMap = null;
    public InterpolatingDoubleTreeMap mainMap = new InterpolatingDoubleTreeMap();

    public velocityMap(){
        // added 59.69cm offset to center of hub
        mainMap.put(1.0,1200.0);
        mainMap.put(1.5, 1350.0);
        mainMap.put(2.0, 1550.0);
        mainMap.put(2.5,1700.0);
        mainMap.put(3.0,1850.0);
    }

    public static velocityMap getInstance(){
        if (vMap == null){
            vMap = new velocityMap();
        }
        return vMap;
    }
}
