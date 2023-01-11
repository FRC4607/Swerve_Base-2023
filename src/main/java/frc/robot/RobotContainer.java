package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.commands.FollowPath;
import frc.robot.commands.RestHeading;
import frc.robot.commands.SetModuleHomes;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Main Class of the robot.
 */
public class RobotContainer {

    // Controllers
    private final XboxController m_driver = new XboxController(
            Constants.Hardware.DRIVER_CONTROLLER_PORT);

    // Subsystems
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

    // Autonomous Commands and Chooser
    private final SendableChooser<Command> m_autonomousSelector = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        m_drivetrainSubsystem.setDefaultCommand(new Drive(m_driver, m_drivetrainSubsystem));

        JoystickButton driverStart = new JoystickButton(m_driver, XboxController.Button.kStart.value);
        driverStart.onTrue(new RestHeading(m_drivetrainSubsystem));

        SmartDashboard.putData(new SetModuleHomes(m_drivetrainSubsystem));
        SmartDashboard.putData(new SetModuleHomes(m_drivetrainSubsystem));

        SmartDashboard.putData(m_autonomousSelector);

        m_autonomousSelector.addOption("Forward", new FollowPath(Paths.FORWARD, m_drivetrainSubsystem));
        m_autonomousSelector.addOption("Other Auto", new FollowPath(Paths.OTHER_AUTO, m_drivetrainSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_autonomousSelector.getSelected();
    }
}
