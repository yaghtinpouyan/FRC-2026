package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
    //Camera 1 position and rotation relative to the robot center (in meters and degrees)
    public static final double camPosX = 0.2096;
    public static final double camPosY = 0.028653;
    public static final double camPosZ = 0.506743;
    public static final double camRotPitch = 15;


     //Camera 2 position and rotation relative to the robot center (in meters and degrees)
    public static final double cam2PosX = -0.209550;
    public static final double cam2PosY = -0.10808;
    public static final double cam2PosZ = 0.505474;
    public static final double cam2RotPitch = 15;

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
}
