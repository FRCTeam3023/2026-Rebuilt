package frc.robot.Commands;

import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.ControlPanel;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Pivot;

public class UniversalCommandFactory {
    public static final Command reefCycle(Drivetrain drivetrain, Elevator elevator, Pivot pivot, EndEffector endEffector) {
        return new RepeatCommand(
            new SequentialCommandGroup(
                new InstantCommand(() -> ControlPanel.ReefCycle.setTravelState(endEffector.coralPresent())),
                new ParallelCommandGroup(
                    ControlPanel.ReefCycle.mutatingPathCommand(),
                    
                    new SequentialCommandGroup(
                        new ParallelRaceGroup(
                            new WaitUntilCommand(() -> endEffector.coralPresent()),
                            new SequentialCommandGroup(
                                new WaitCommand(0.2),
                                new ParallelCommandGroup(
                                    elevator.moveCommand(Constants.ElevatorConstants.MIN_ELEVATOR_EXTENSION),
                                    pivot.setAngleCommand(Constants.PivotConstants.TRAVEL_POSITION)
                                )
                            )
                        ),
                        new SequentialCommandGroup(
                            new WaitUntilCommand(() -> drivetrain.getDistanceToCurrentPath() < Constants.NavigationConstants.PRIME_SCORE_RADIUS),
                            new ParallelCommandGroup(
                                new DeferredCommand(() -> pivot.setAngleCommand(ControlPanel.ReefCycle.getTargetAngle()), Set.of(pivot)),
                                new DeferredCommand(() -> elevator.moveCommand(ControlPanel.ReefCycle.getTargetHeight()), Set.of(elevator))
                            )
                        )
                    )
                ),
                new WaitUntilCommand(() -> endEffector.coralPresent() != ControlPanel.ReefCycle.getTravelState()),
                new WaitCommand(0.5)
            )
        );
    }
}