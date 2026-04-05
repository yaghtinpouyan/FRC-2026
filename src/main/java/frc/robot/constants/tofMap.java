package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class tofMap {
     private static tofMap tMap = null;
    public InterpolatingDoubleTreeMap mainMap = new InterpolatingDoubleTreeMap();

    public tofMap(){
        //Calculated placeholder values
        mainMap.put(1.45,0.663);
        mainMap.put(1.76,0.711);
        mainMap.put(2.07, 0.758);
        mainMap.put(2.38, 0.81);
        mainMap.put(2.69,0.862);
        mainMap.put(3.0,0.913);
        mainMap.put(3.31, 0.928);
        mainMap.put(4.0, 0.987);
    }

    public static tofMap getInstance(){
        if (tMap == null){
            tMap = new tofMap();
        }
        return tMap;
    }
}
