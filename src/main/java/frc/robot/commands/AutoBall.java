package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.classes.SPIKE293Utils;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonVision;
import static frc.robot.Constants.DrivetrainConstants.*;

public class AutoBall extends CommandBase {
  private final PhotonVision m_photonVision;
  private final JoystickButton m_aButton;
  private final Drivetrain m_drive;
  private final XboxController m_xboxcontroller;
  private Double m_yaw; 
  private double m_turningOutput = 0.0d;
  private double m_error = 0.0d;
  private double m_lastError = 0.0d;
  private double m_change = 0.0d;
  private double m_errorIntegral = 0.0d;
  // start (gives throttle) (may make it overshoot if too high)
  private final double m_P = 0.0015d;
  // finicky (depends on situation) (within 5 to 3 degress of error)
  private final double m_I = 0.00d;
  // good rule of thumb for d: m_d = m_p * 10
  private final double m_D = m_P * 10;
  public AutoBall(
      PhotonVision photonVision,
      JoystickButton aButton,
      XboxController controller,
      Drivetrain drive) {
    m_photonVision = photonVision;
    m_aButton = aButton;
    m_drive = drive;
    m_xboxcontroller = controller;
    addRequirements(photonVision, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (false == m_photonVision.hasTarget()) return;
    m_yaw = m_photonVision.getYaw();

    SmartDashboard.putNumber("Yaw", m_yaw);
    
    m_error = m_yaw;
    
    m_change = m_error - m_lastError;
    
    if(Math.abs(m_errorIntegral) < 5) // this is an integral limit to keep from excessive I commanded movement 
    {
        //Accumulate the error into the integral
        m_errorIntegral += m_error;
    }
    
    m_turningOutput = (m_P * m_error) + (m_I * m_errorIntegral) + (m_D * m_change);

    double triggerRight = m_xboxcontroller.getRightTriggerAxis();
    triggerRight = SPIKE293Utils.applyDeadband(triggerRight, DEFAULT_FORZA_DEADBAND);
    double triggerLeft = m_xboxcontroller.getLeftTriggerAxis();
    triggerLeft = SPIKE293Utils.applyDeadband(triggerLeft, DEFAULT_FORZA_DEADBAND);
    double speed = 0;
    if (triggerRight >= triggerLeft) {
        // Use right trigger for forward speed!
        speed = triggerRight;
    } else {
        // Going in reverse! Right trigger was zero, set speed to left trigger
        speed = -triggerLeft;
    }
    
    speed = speed * 0.65;
    m_turningOutput *= 0.5;

    arcadeDrive(speed, m_turningOutput);
  }

  private void arcadeDrive(double velocity, double turning) {
    // Convert turning and speed to left right encoder velocity
    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(velocity), Math.abs(turning)), velocity);
    if (velocity >= 0.0) {
        // First quadrant, else second quadrant
        if (turning >= 0.0) {
            leftMotorOutput = maxInput;
            rightMotorOutput = velocity - turning;
        } else {
            leftMotorOutput = velocity + turning;
            rightMotorOutput = maxInput;
        }
    } else {
        // Third quadrant, else fourth quadrant
        if (turning >= 0.0) {
            leftMotorOutput = velocity + turning;
            rightMotorOutput = maxInput;
        } else {
            leftMotorOutput = maxInput;
            rightMotorOutput = velocity - turning;
        }
    }

    // Convert to encoder velocity
    // double leftMotorSpeed =
    // SPIKE293Utils.percentageToControllerVelocity(leftMotorOutput);
    // Right needs to be inverted
    // double rightMotorSpeed =
    // SPIKE293Utils.percentageToControllerVelocity(rightMotorOutput *-1.0d);

    // Send to motors
    m_drive.percentDrive(leftMotorOutput, rightMotorOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_aButton.whenInactive(this::cancel);
    return false;
  }
}