package frc.robot.autos;
import frc.robot.subsystems.Intake;
import frc.robot.commands.IntakeOut;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class justScore extends SequentialCommandGroup {
    public justScore(/*Pivot m_Pivot,*/ Intake m_Intake) {
        addCommands(
            //new GoHome(m_Pivot),
            new IntakeOut(m_Intake)
        );
    }
}
