package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class hoodMap {
    private static hoodMap vMap = null;
    public InterpolatingDoubleTreeMap mainMap = new InterpolatingDoubleTreeMap();

    public hoodMap(){
        mainMap.put(1.0,20.0);
        mainMap.put(1.45,20.0);
        mainMap.put(1.76,21.0);
        mainMap.put(2.07, 22.0);
        mainMap.put(2.38, 23.0);
        mainMap.put(2.69,24.0);
        mainMap.put(3.0,25.0);
        mainMap.put(3.31, 26.0);
        mainMap.put(4.0, 30.0);
    }

    public static hoodMap getInstance(){
        if (vMap == null){
            vMap = new hoodMap();
        }
        return vMap;
    }
}