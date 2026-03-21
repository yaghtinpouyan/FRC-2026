package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class velocityMap {
    private static velocityMap vMap = null;
    public InterpolatingDoubleTreeMap mainMap = new InterpolatingDoubleTreeMap();

    public velocityMap(){
        mainMap.put(0.64135,1600.0);
        mainMap.put(1.49225,1675.0);
        mainMap.put(1.6,1750.0);   //Inaccurate entry
        mainMap.put(2.8702,2050.0);
        mainMap.put(2.57175,2050.0);
        mainMap.put(3.00355,2125.0);
        mainMap.put(0.0254, 1500.0);
        mainMap.put(2.52095, 1700.0);
    }

    public static velocityMap getInstance(){
        if (vMap == null){
            vMap = new velocityMap();
        }
        return vMap;
    }
}
