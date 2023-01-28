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
    private Position2D m_Tag2 = new Position2D(21.5168,-9.48,0);

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
                        new DriveToAT(m_Tag2,2.0d, false, m_kinematics, m_drivetrain, m_targeting)
                
                        
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
