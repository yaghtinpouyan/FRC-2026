package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class velocityMap {
    private static velocityMap vMap = null;
    public InterpolatingDoubleTreeMap mainMap = new InterpolatingDoubleTreeMap();

    public velocityMap(){
        mainMap.put(1.45,1415.0);
        mainMap.put(1.76,1435.0);
        mainMap.put(2.07, 1445.0);
        mainMap.put(2.38, 1505.0);
        mainMap.put(2.45, 1510.0);
        mainMap.put(2.69,1515.0);
        mainMap.put(3.0,1540.0);
        mainMap.put(3.31, 1580.0);
        mainMap.put(4.0, 1660.0);
    }

    public static velocityMap getInstance(){
        if (vMap == null){
            vMap = new velocityMap();
        }
        return vMap;
    }
}
