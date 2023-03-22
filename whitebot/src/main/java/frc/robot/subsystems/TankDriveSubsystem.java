package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TankDriveCmd;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
    //public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        
    //}
    
    @SuppressWarnings("ParameterName")
    public void tankDriveCommand(double speedRight, double speedLeft){
        m_drive.arcadeDrive(speedRight * DriveConstants.kNormSpeedMult, speedLeft * DriveConstants.kNormSpeedMult);
    }
}
