package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Nothing extends SequentialCommandGroup {
    public Nothing(
        // does nothing btw
    ) {
        addCommands(Commands.none());
    }
}