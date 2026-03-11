package frc.robot.test;
import frc.robot.subsystems.Climb;
import frc.robot.constants.Constants;

public class ClimbTest {
    private static ClimbTest climbTest = null;
    private Climb climber = Climb.getInstance();

    public void runClimbSysID(){
        climber.sysId(Constants.climbSysIdMaxVoltage, Constants.climbSysIdStep, Constants.climbSysIdDuration);
    }

    public static ClimbTest getInstance(){
        if (climbTest == null){
            climbTest = new ClimbTest();
        }
        return climbTest;
    }
}
