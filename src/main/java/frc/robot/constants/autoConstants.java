package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class autoConstants {
    //IMPORTANT Note I stands for intermediate pose for pathplanner

    //Hub Pose
    public static final Pose2d HubB = new Pose2d(4.6, 4, Rotation2d.fromDegrees(0)); //blue
    public static final Pose2d HubR = new Pose2d(11.9, 4, Rotation2d.fromDegrees(0)); //red

    //Tower Poses
    public static final Pose2d TowerB = new Pose2d(1, 3.7, Rotation2d.fromDegrees(0)); //blue
    public static final Pose2d TowerBI = new Pose2d(2, 3.7, Rotation2d.fromDegrees(0));

    public static final Pose2d TowerR = new Pose2d(15.5, 3.7, Rotation2d.fromDegrees(0)); //red
    public static final Pose2d TowerRI = new Pose2d(14.5, 3.7, Rotation2d.fromDegrees(0));

    //Trench Poses
    public static final Pose2d RedTrenchRight = new Pose2d(11.9, 7.4, Rotation2d.fromDegrees(0));
    public static final Pose2d RedTrenchRightI = new Pose2d(12.4, 7.4, Rotation2d.fromDegrees(0));
    public static final Pose2d RedTrenchLeft = new Pose2d(11.9, 0.7, Rotation2d.fromDegrees(0));
    public static final Pose2d RedTrenchLeftI = new Pose2d(12.4, 0.7, Rotation2d.fromDegrees(0));

    //point1: blue trench right is x = 4.6, y = 7.4
    // point2: blue trench right x = 4.6, y = 0.6
    public static final Pose2d BlueTrenchRight = new Pose2d(4.6, 7.4, Rotation2d.fromDegrees(0));
    public static final Pose2d BlueTrenchRightI = new Pose2d(5.1, 7.4, Rotation2d.fromDegrees(0));
    public static final Pose2d BlueTrenchLeft = new Pose2d(4.6, 0.6, Rotation2d.fromDegrees(0));
    public static final Pose2d BlueTrenchLeftI = new Pose2d(5.1, 0.6, Rotation2d.fromDegrees(0));

    //Blue april tags (Outpost = O, Climbing Area = C, Trench = T, Hub = H, B = blue)
    public static final int tagBH1 = 18;
    public static final int tagBH2 = 19;
    public static final int tagBH3 = 20;
    public static final int tagBH4 = 21;
    public static final int tagBH5 = 24;
    public static final int tagBH6 = 25;
    public static final int tagBH7 = 26;
    public static final int tagBH8 = 27;

    public static final int tagBT1 = 17;
    public static final int tagBT2 = 22;
    public static final int tagBT3 = 23;
    public static final int tagBT4 = 28;

    public static final int tagBC1 = 31;
    public static final int tagBC2 = 32;
    
    public static final int tagBO1 = 29;
    public static final int tagBO2 = 30;

    //Red april tags (Outpost = O, Climbing Area = C, Trench = T, Hub = H, R = red)
    public static final int tagRH1 = 2;
    public static final int tagRH2 = 3;
    public static final int tagRH3 = 4;
    public static final int tagRH4 = 5;
    public static final int tagRH5 = 8;
    public static final int tagRH6 = 9;
    public static final int tagRH7 = 10;
    public static final int tagRH8 = 11;

    public static final int tagRT1 = 1;
    public static final int tagRT2 = 6;
    public static final int tagRT3 = 7;
    public static final int tagRT4 = 12;

    public static final int tagRC1 = 15;
    public static final int tagRC2 = 16;
    
    public static final int tagRO1 = 13;
    public static final int tagRO2 = 14;
}