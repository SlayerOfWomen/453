
package frc.robot.commands;

import frc.robot.subsystems.TankDriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
public class TankDriveCmd extends CommandBase {
    private final TankDriveSubsystem m_tankDriveSubsystem;
    private final Joystick driver = new Joystick(Constants.IOConstants.kDriverControllerPort);

    public TankDriveCmd(TankDriveSubsystem subsystem) {
        this.m_tankDriveSubsystem = subsystem;
        addRequirements(m_tankDriveSubsystem);
    }
    @Override
    public void execute() {
        m_tankDriveSubsystem.tankDriveCommand(driver.getRawAxis(2), driver.getRawAxis(1));
    }
    @Override
    public void initialize() {

    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
