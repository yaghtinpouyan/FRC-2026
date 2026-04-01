package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
    //Camera 1 position and rotation relative to the robot center (in meters and degrees)
    public static final double frontCamPosX = -0.213;
    public static final double frontCamPosY = 0.027;
    public static final double frontCamPosZ = 0.509;
    public static final double frontCamRotPitch = (Math.PI/12);
    public static final double frontCamRotYaw = Math.PI;

    //Camera 2 position and rotation relative to the robot center (in meters and degrees)
    public static final double backCamPosX = 0.213;
    public static final double backCamPosY = 0.110;
    public static final double backCamPosZ = 0.508;
    public static final double backCamRotPitch = (Math.PI/12);
    public static final double backCamRotYaw = 0;

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
}
