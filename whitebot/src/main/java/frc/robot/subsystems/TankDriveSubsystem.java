package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TankDriveCmd;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.examples.ramsetecommand.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;


public class TankDriveSubsystem extends SubsystemBase{
    //private final TankDriveCmd m_tankDriveCmd = new TankDriveCmd();
    private WPI_TalonSRX leftTank1 = new WPI_TalonSRX(DriveConstants.kLeft1Port);
    private WPI_TalonSRX leftTank2 = new WPI_TalonSRX(DriveConstants.kLeft2Port);
    private WPI_TalonSRX rightTank1 = new WPI_TalonSRX(DriveConstants.kRight1Port);
    private WPI_TalonSRX rightTank2 = new WPI_TalonSRX(DriveConstants.kRight2Port);
    private Encoder encoderR = new Encoder(0, 1);
    private Encoder encoderL = new Encoder(2, 3);
    public MotorControllerGroup m_leftMotors = new MotorControllerGroup(leftTank1, leftTank2);
    public MotorControllerGroup m_rightMotors = new MotorControllerGroup(rightTank1, rightTank2);
    private DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    //Gyro sensor
    private final Gyro m_gyro = new ADXRS450_Gyro();

    //Odometry for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;


    //new DriveSubsystem
    public TankDriveSubsystem()
    {
        //inverts one wheel to make robot move straight
        m_rightMotors.setInverted(true);

        //sets distance per pulse for encoders
        encoderL.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        encoderR.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        //resets encoders then sets odometry
        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), encoderL.getDistance(), encoderR.getDistance());

    }

    //public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        
    //}
    
    @SuppressWarnings("ParameterName")
    public void tankDriveCommand(double speedRight, double speedLeft){
        m_drive.arcadeDrive(speedRight * DriveConstants.kNormSpeedMult, speedLeft * DriveConstants.kNormSpeedMult);
    }

    @Override
    public void periodic()
    {
        //updates the odometry for robot pose
        m_odometry.update(m_gyro.getRotation2d(), encoderL.getDistance(), encoderR.getDistance());
    }


    //returns the pose of the robot
    public Pose2d getPose()
    {
        return m_odometry.getPoseMeters();
    }

    //returns the speed of the wheels
    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(encoderL.getRate(), encoderR.getRate());
    }

    //resets the odometrys pose
    public void resetOdometry(Pose2d pose)
    {
        resetEncoders();
        m_odometry.resetPosition(m_gyro.getRotation2d(), encoderL.getDistance(), encoderR.getDistance(), pose);
    }

    //Drives robot with aracde (NOT THE SAME ARACDE DRIVE AT LINE 61)
    public void arcadeDrive(double fwd, double rot)
    {
        //fwd is the foward movement
        //rot is the rotation of the robot
        m_drive.arcadeDrive(fwd, rot);
    }


    //Drives the robots left and right side VIA volts
    public void tankDriveVolts(double leftVolts, double rightVolts)
    {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(rightVolts);
        m_drive.feed();
    }

    //resets the encoders of the robot
    public void resetEncoders()
    {
        encoderL.reset();
        encoderR.reset();
    }

    //Gets the average distance between the 2 encoders
    public double getAverageEncoderDistance()
    {
        return (encoderL.getDistance() + encoderR.getDistance()) / 2.0;
    }

    //returns the left encoder
    public Encoder getLeftEncoder()
    {
        return encoderL;
    }

    //returns the right encoder
    public Encoder getRightEncoder()
    {
        return encoderR;
    }

    //Sets the max output of the drive
    //Constraints the volts
    public void setMaxOutput(double maxOutput)
    {
        m_drive.setMaxOutput(maxOutput);
    }

    //Zeroes the heading of the robot
    public void zeroHeading()
    {
        m_gyro.reset();
    }

    //returns the heading of the robot
    public double getHeading()
    {
        return m_gyro.getRotation2d().getDegrees();
    }

    //returns the turn rate of the robot
    public double getTurnRate()
    {
        return -m_gyro.getRate();
    }

}
