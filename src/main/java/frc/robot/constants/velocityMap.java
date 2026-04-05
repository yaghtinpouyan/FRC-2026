package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class velocityMap {
    private static velocityMap vMap = null;
    public InterpolatingDoubleTreeMap mainMap = new InterpolatingDoubleTreeMap();

    public velocityMap(){
        mainMap.put(1.45,1415.0);
        mainMap.put(1.76,1435.0);
        mainMap.put(2.07, 1445.0);
        mainMap.put(2.38, 1465.0);
        mainMap.put(2.69,1485.0);
        mainMap.put(3.0,1500.0);
        mainMap.put(3.31, 1530.0);
        mainMap.put(4.0, 1650.0);
    }

    public static velocityMap getInstance(){
        if (vMap == null){
            vMap = new velocityMap();
        }
        return vMap;
    }
}
