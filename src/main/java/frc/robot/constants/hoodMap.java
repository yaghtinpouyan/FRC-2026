package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class hoodMap {
    private static hoodMap vMap = null;
    public InterpolatingDoubleTreeMap mainMap = new InterpolatingDoubleTreeMap();

    public hoodMap(){
        // added 59.69cm offset to center of hub
        mainMap.put(1.45,20.0);
        mainMap.put(1.76,22.0);
        mainMap.put(2.07, 24.0);
        mainMap.put(2.38, 26.0);
        mainMap.put(2.69,28.0);
        mainMap.put(3.0,30.0);
    }

    public static hoodMap getInstance(){
        if (vMap == null){
            vMap = new hoodMap();
        }
        return vMap;
    }
}