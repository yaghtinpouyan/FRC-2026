package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class velocityMap {
    private static velocityMap vMap = null;
    public InterpolatingDoubleTreeMap mainMap = new InterpolatingDoubleTreeMap();

    public velocityMap(){
        mainMap.put(0.64135,3.2);
        mainMap.put(1.49225,3.35);
        mainMap.put(1.6,3.5);   //Inaccurate entry
        mainMap.put(2.8702,4.1);
        mainMap.put(2.57175,4.1);
        mainMap.put(3.00355,4.25);
    }

    public static velocityMap getInstance(){
        if (vMap == null){
            vMap = new velocityMap();
        }
        return vMap;
    }
}
