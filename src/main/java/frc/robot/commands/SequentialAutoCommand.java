package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Targeting;
import frc.robot.Constants.AutonomousCommandConstants.StartPositions;

public class SequentialAutoCommand extends SequentialCommandGroup {
    private StartPositions m_startPosition;
    private Drivetrain m_drivetrain;
    private Kinematics m_kinematics;
    private AutoTarget m_autoBall;
    private Targeting m_targeting;

    public SequentialAutoCommand(Drivetrain drivetrain, Kinematics kinematics, StartPositions startPosition, Targeting targeting) {

        m_drivetrain = drivetrain;
        m_kinematics = kinematics;
        m_startPosition = startPosition;
        m_targeting = targeting;
        SmartDashboard.putBoolean("AutoDone", false);

        switch (m_startPosition) {
            case LEFT:
                addCommands(
                        new ResetKinematics(new Position2D(0, 0, Math.toRadians(0)), m_drivetrain, m_kinematics),
                        new DriveTo(new Position2D(4, 0, Math.toRadians(90)),2.0d, false, m_kinematics, m_drivetrain),
                        new AutoTarget(m_targeting, m_drivetrain)
                
                        
                        //new DriveTo(new Position2D(7, 14, Math.toRadians(0)),2.0d, false, m_kinematics, m_drivetrain)
                        // new DriveTo(new Position2D(16, 0, Math.toRadians(270)),2.0d, false, m_kinematics, m_drivetrain),
                        // new DriveTo(new Position2D(0, 0, Math.toRadians(180)),2.0d, false, m_kinematics, m_drivetrain)
                        );

                break;
            case MIDDLE:

                break;
            case RIGHT:

                break;
            default:
                break;
        }
    }
}
