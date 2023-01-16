// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.Constants.AutonomousCommandConstants.StartPositions;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  // The robot's subsystems
  public final Kinematics m_kinematics = new Kinematics(new Position2D(0.0, 0.0, 0.0));
  public final Drivetrain m_drivetrain = new Drivetrain(m_kinematics);
  public final Targeting m_targeting = new Targeting();

  // Joysticks
  public final XboxController m_xboxController = new XboxController(0);

  //Commands
  public Command m_AutoCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous Command",
        new SequentialAutoCommand(m_drivetrain, m_kinematics, StartPositions.LEFT, m_targeting));
    SmartDashboard.putData("ArcadeDrive", new ArcadeDrive(m_drivetrain, m_xboxController));
    SmartDashboard.putData("Reset Kinematics",
        new ResetKinematics(new Position2D(0, 0, 0), m_drivetrain, m_kinematics));

    // Configure the button bindings
    configureButtonBindings();

    // Setting default command for drivetrain as VelocityDrive
    m_drivetrain.setDefaultCommand(new ArcadeDrive(m_drivetrain, m_xboxController));

    if(DriverStation.getLocation() == 1){
      m_AutoCommand = new SequentialAutoCommand(m_drivetrain, m_kinematics, StartPositions.LEFT, m_targeting);
    }
    else if(DriverStation.getLocation() == 2){
      m_AutoCommand = new SequentialAutoCommand(m_drivetrain, m_kinematics, StartPositions.MIDDLE, m_targeting);
    }
    else if(DriverStation.getLocation() == 3){
      m_AutoCommand = new SequentialAutoCommand(m_drivetrain, m_kinematics, StartPositions.RIGHT, m_targeting);
    }
    else{
      System.out.println("Field location error");
    }
  }
  
  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
    // Create some buttons
    final JoystickButton xboxFireBtn = new JoystickButton(m_xboxController, XboxController.Button.kLeftBumper.value);     
    xboxFireBtn.whileHeld(new LocateTarget(m_drivetrain, m_targeting));

    final JoystickButton aBtn = new JoystickButton(m_xboxController, XboxController.Button.kA.value);
    aBtn.whileHeld(new AutoTarget(m_targeting, m_drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    Command autoCommand = new SequentialAutoCommand(m_drivetrain, m_kinematics, StartPositions.LEFT, m_targeting);
    
    if(DriverStation.getLocation() == 1){
      autoCommand = new SequentialAutoCommand(m_drivetrain, m_kinematics, StartPositions.LEFT, m_targeting);
    }
    else if(DriverStation.getLocation() == 2){
      autoCommand = new SequentialAutoCommand(m_drivetrain, m_kinematics, StartPositions.MIDDLE, m_targeting);
    }
    else if(DriverStation.getLocation() == 3){
      autoCommand = new SequentialAutoCommand(m_drivetrain, m_kinematics, StartPositions.RIGHT, m_targeting);
    }
    else{
      System.out.println("Field location error");
    }
    return autoCommand;
  }

  public Command getTeleopCommand() {
    return new ArcadeDrive(m_drivetrain, m_xboxController);
  }
}
