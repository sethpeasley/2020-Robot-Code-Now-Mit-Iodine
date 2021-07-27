/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the Robot periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 * @author LukeCrum, Nicholas Blackburn
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final Command m_autoCommand;

    private NetworkTableEntry kp, kd, kv, ka;

    public static Joystick m_driverController = new Joystick(Constants.HIDConstants.k_DriverPort);
    public static Joystick m_operatorController = new Joystick(Constants.HIDConstants.k_OperatorPort);

    public static JoystickButton climberUp, climberDown, intakeIn, intakeOut, intakeFlip, rotationControl, positionControl, shoot, target, indexerOverride, turretOverride, setAutomatic;
    //TODO: Manual indexer, shooter, override toggle

    private final Indexer m_indexer = new Indexer(m_driverController);
    private final Drivetrain m_drivetrain; // = new Drivetrain();
    private final ControlPanel m_controlpanel = new ControlPanel();
    private final Climb m_climb = new Climb();
    private final Turret m_turret = new Turret();
    private final Intake m_intake = new Intake();
    private final DashBoard m_dash; // = new DashBoard(m_drivetrain, m_indexer, m_turret, m_controlpanel, m_intake);

    public RobotContainer()
    {
        WPI_TalonSRX m_leftFrontDriveMotor = new WPI_TalonSRX(Constants.CAN_Addresses.leftDriveACAN);
        WPI_TalonSRX m_rightFrontDriveMotor = new WPI_TalonSRX(Constants.CAN_Addresses.rightDriveACAN);
        WPI_TalonSRX m_leftRearDriveMotor = new WPI_TalonSRX(Constants.CAN_Addresses.leftDriveBCAN);
        WPI_TalonSRX m_rightRearDriveMotor = new WPI_TalonSRX(Constants.CAN_Addresses.rightDriveBCAN);

        ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    
        m_drivetrain = new Drivetrain(m_leftFrontDriveMotor, m_rightFrontDriveMotor, 
                                      m_leftRearDriveMotor, m_rightRearDriveMotor,
                                      m_gyro);

        m_dash = new DashBoard(m_drivetrain, m_indexer, m_turret, m_controlpanel, m_intake);

        configureButtonBindings();

        //m_autoCommand = new Autotestpath(m_drivetrain, m_intake, m_turret, m_indexer);
        m_autoCommand = new TestPathCommand(m_drivetrain);

        m_drivetrain.setDefaultCommand(new RunCommand(() -> m_drivetrain.deadbandedArcadeDrive(), m_drivetrain));
        
        m_indexer.setDefaultCommand(new AutomatedIndexer(m_indexer));
        // m_dash.setDefaultCommand(new RunCommand(() -> m_dash.dashboardData(), m_dash));
        m_intake.setDefaultCommand(new RaiseIntake(m_intake));
    }

    private void configureButtonBindings()
    {
        shoot = new JoystickButton(m_driverController, Constants.HIDConstants.k_A);
        target = new JoystickButton(m_driverController, Constants.HIDConstants.k_RB);
        climberUp = new JoystickButton(m_driverController, Constants.HIDConstants.k_Start);
        climberDown = new JoystickButton(m_driverController, Constants.HIDConstants.k_Select);

        intakeIn = new JoystickButton(m_operatorController, Constants.HIDConstants.k_RB);
        intakeOut = new JoystickButton(m_operatorController, Constants.HIDConstants.k_LB);
        intakeFlip = new JoystickButton(m_operatorController, Constants.HIDConstants.k_A);

        rotationControl = new JoystickButton(m_operatorController, Constants.HIDConstants.k_Start);
        positionControl = new JoystickButton(m_operatorController, Constants.HIDConstants.k_Select);

        indexerOverride = new JoystickButton(m_operatorController, Constants.HIDConstants.k_LeftStickPress);
        turretOverride = new JoystickButton(m_operatorController, Constants.HIDConstants.k_RightStickPress);
        setAutomatic = new JoystickButton(m_operatorController, Constants.HIDConstants.k_X);

        shoot.whileHeld(new Shoot(m_turret, m_indexer));
        target.whileHeld(new Target(m_turret));
        climberUp.whenPressed(new RaiseClimber(m_climb));
        climberDown.whileHeld(new RunClimb(m_climb));

        intakeIn.whileHeld(new RunIntake(m_intake));
        intakeOut.whileHeld(new RunReverseIntake(m_intake));
        intakeFlip.toggleWhenPressed(new RaiseIntake(m_intake));

        indexerOverride.toggleWhenPressed(new ManualIndexer(m_operatorController, m_indexer));
        turretOverride.toggleWhenPressed(new ManualTurret(m_turret, m_operatorController));
        //setAutomatic.whenPressed(new InstantCommand(() -> setAutomatic(), m_turret, m_indexer));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return m_autoCommand;
    }

    public void manualOverrideIndexer()
    {
        m_indexer.setDefaultCommand(new ManualIndexer(m_operatorController, m_indexer));
    }

    public void manualOverrideTurret()
    {
        target.toggleWhenPressed(new VoidCommand());
        shoot.toggleWhenPressed(new RunTurret(m_turret));
        m_turret.setDefaultCommand(new ManualTurret(m_turret, m_operatorController));
    }

    public void setAutomatic()
    {
        m_indexer.setDefaultCommand(new AutomatedIndexer(m_indexer));
        target.toggleWhenPressed(new Target(m_turret));
        shoot.toggleWhenPressed(new Shoot(m_turret, m_indexer));
        m_turret.setDefaultCommand(new VoidCommand());
    }
}
