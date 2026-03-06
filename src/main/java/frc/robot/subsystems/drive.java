package frc.robot.subsystems;

//WPILIB Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.Constants;
import frc.robot.subsystems.automations.Vision;
import frc.robot.subsystems.automations.autoAlign;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

//YAGSL Imports
import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.SwerveModule;

//Pathplanner imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


public class drive extends SubsystemBase {
    private static drive swerve = null;
    
    //YAGSL
    private File directory = new File(Filesystem.getDeployDirectory(),"swerve2");
    public SwerveDrive swerveDrive;
    private SwerveInputStream angularVel;
    private SwerveInputStream driveVel;

    //Advantage Scope
    private StructPublisher<Pose3d> publisher3d;
    private StructPublisher<ChassisSpeeds> publisherSpeed;
    private Pose2d currentPose2d;
    private Pose3d currentPose3d;

    //Pathplanner
    private RobotConfig config;

    //Select module
    public int selectModule;

    //sysid
    SysIdRoutine driveSysID;
    SysIdRoutine angleSysID;
    Direction currentDir = Direction.kForward;
    private SwerveModule[] modules;

    //Driving modes
    private boolean driveLock = false;
    intake ballIntake = intake.getInstance();

    private drive(){
        try
        {
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxDriveSpeed, Constants.startPose);
        } catch (Exception e)
        {
        throw new RuntimeException(e);
        }

        //Advantage Scope
        publisherSpeed = NetworkTableInstance.getDefault().getStructTopic("MyChassisSpeed", ChassisSpeeds.struct).publish();
        publisher3d = NetworkTableInstance.getDefault().getStructTopic("/AdvantageScope/Robot/Pose", Pose3d.struct).publish();
        modules = swerveDrive.getModules();

        //sysID
        driveSysID = new SysIdRoutine(new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(
                (voltage) -> {modules[0].getDriveMotor().setVoltage(voltage.in(Volts));}, 
                (log) -> {
                    log.motor("drive1").voltage(Volts.of(modules[0].getDriveMotor().getVoltage()))
                        //Potential encoder return error, the ttb and integrated encoders could be returning non meter related units
                       .linearVelocity(MetersPerSecond.of(modules[0].getDriveMotor().getVelocity()/(Constants.secondsPerMinute*Constants.driveCircumferenceMeters)))
                       .linearPosition(Meters.of(modules[0].getDriveMotor().getPosition()*Constants.driveCircumferenceMeters));
                }, 
                this
            )
        );

        angleSysID = new SysIdRoutine(new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(
                (voltage) -> {modules[0].getAngleMotor().setVoltage(voltage.in(Volts));}, 
                (log) -> {
                    log.motor("angle1").voltage(Volts.of(modules[0].getAngleMotor().getVoltage()))
                    //Changed to use internal motor encoder, (watch out for meters instead of deg)
                    .angularVelocity(RotationsPerSecond.of(modules[0].getAngleMotor().getVelocity()/Constants.secondsPerMinute))
                    .angularPosition(Rotations.of(modules[0].getAngleMotor().getPosition()));
                }, 
                this
            )
        );

        /*
         * 
         */

        //Pathplanner
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Configure AutoBuilder last
            AutoBuilder.configure(
            this::getRobotPose, 
            this::resetRobotPose, 
            this::getRobotSpeed, 
            (speeds, feedforwards) -> drive(speeds), 
            new PPHolonomicDriveController( 
                    new PIDConstants(5.0, 0.0, 0.0), // Movement PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            drive.this// Reference to this subsystem to set requirements
        );

    }

    //Main driving code
    public void switchModes(boolean mode1, boolean mode2){
        if(mode1) driveLock = false;
        if(mode2) driveLock = true;
    }

    public void driveInputHandler(double x, double y, double theta, boolean toggle1, boolean toggle2){
        switchModes(toggle1, toggle2);
        if(driveLock){
            rotateToYaw(x, y);
            ballIntake.stateIntaking = false;
        }
        else{
            swerveSupplier(x,y,theta);
        }
    }
    
    public void swerveSupplier(double x, double y, double theta){
        angularVel = SwerveInputStream.of(
            returnSwerveDrive(), 
            () -> x, 
            () -> y
        ).withControllerRotationAxis(() -> theta).scaleRotation(0.8).deadband(Constants.deadband).allianceRelativeControl(true);   

        driveVel = angularVel.copy().withControllerHeadingAxis(() -> x, () ->y);
        driveField(driveVel.get());

        publisherSpeed.set(swerveDrive.getFieldVelocity());
    }

    private void rotateToYaw(double x, double y){
        autoAlign alignBot = autoAlign.getInstance();
       Rotation2d targetAngle = alignBot.getHubYaw();
        //could bug out with diff alliance
        swerveDrive.driveFieldOriented(
            swerveDrive.swerveController.getTargetSpeeds(
                x, 
                y, 
                targetAngle.getRadians(), 
                swerveDrive.getYaw().getRadians(), 
                swerveDrive.getMaximumChassisVelocity()
            )
        );

        driveVel = angularVel.copy().withControllerHeadingAxis(() -> x, () ->y);
    }

    private void drive(ChassisSpeeds vel){
        swerveDrive.drive(vel);
    }

    private void driveField(ChassisSpeeds vel){
        swerveDrive.driveFieldOriented(vel);
    }

    //Helper methods
    private SwerveDrive returnSwerveDrive(){
        return swerveDrive;
    }

    public void autoZero(boolean input1){
        SwerveModule[] modules = swerveDrive.getModules();
        if(input1){
            modules[0].setAngle(0);
            modules[1].setAngle(0);
            modules[2].setAngle(0);
            modules[3].setAngle(0);
        }
    }

    public void runDriveSysID(boolean toggleF, boolean toggleR, boolean quat, boolean dynamic){
        if(toggleF) currentDir = Direction.kForward;
        if(toggleR) currentDir = Direction.kReverse;
        if(quat) driveSysID.quasistatic(currentDir).schedule();
        if(dynamic) driveSysID.dynamic(currentDir).schedule();
    }

    public void runAngleSysID(boolean toggleF, boolean toggleR, boolean quat, boolean dynamic){
        if(toggleF) currentDir = Direction.kForward;
        if(toggleR) currentDir = Direction.kReverse;
        if(quat) angleSysID.quasistatic(currentDir).schedule();
        if(dynamic) angleSysID.dynamic(currentDir).schedule();
    }

    public ChassisSpeeds getRobotSpeed(){
        return swerveDrive.getRobotVelocity();
    }

    public Pose2d getRobotPose(){
        return swerveDrive.getPose();
    }

    public void resetRobotPose(Pose2d pose){
        swerveDrive.resetOdometry(pose);
    }

    public void updatePoseEstimator(){
        Vision.getInstance().updateVision();
        swerveDrive.updateOdometry();
    }

    //Update advantage scope periodically
    @Override
    public void periodic(){
        //Get current pose 3d for advantage scope
        Rotation2d heading = swerveDrive.getYaw();
        currentPose2d = swerveDrive.getPose();
        currentPose3d = new Pose3d(currentPose2d.getX(), currentPose2d.getY(), 0, new Rotation3d(heading));
        //Send 3D data to advantage scope
        publisher3d.set(currentPose3d);
    }

    public static drive getInstance(){
        if (swerve == null){
            swerve = new drive();
        }
        return swerve;
    }
}
