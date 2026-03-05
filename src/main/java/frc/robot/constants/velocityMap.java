package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class velocityMap {
    private static velocityMap vMap = null;
    public InterpolatingDoubleTreeMap mainMap = new InterpolatingDoubleTreeMap();

    public velocityMap(){
        mainMap.put(1.0,4.0);
        mainMap.put(2.0,4.0);
        mainMap.put(3.0,4.0);
        mainMap.put(4.0,4.0);
        mainMap.put(5.0,4.0);
        mainMap.put(6.0,4.0);
        mainMap.put(7.0,4.0);
        mainMap.put(8.0,4.0);
        mainMap.put(9.0,4.0);
        mainMap.put(10.0,4.0);
        mainMap.put(11.0,4.0);
        mainMap.put(12.0,4.0);
        mainMap.put(13.0,4.0);
        mainMap.put(14.0,4.0);
        mainMap.put(15.0,4.0);
        mainMap.put(16.0,4.0);
        mainMap.put(17.0,4.0);
    }

    public static velocityMap getInstance(){
        if (vMap == null){
            vMap = new velocityMap();
        }
        return vMap;
    }
}
