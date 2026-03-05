package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class angleMap {
    public static InterpolatingDoubleTreeMap mainMap;
    
    public angleMap(){
        mainMap = new InterpolatingDoubleTreeMap();
        mainMap.put(4.0,4.0);
;    }
}
