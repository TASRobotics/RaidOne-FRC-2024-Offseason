package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DrivePath;

public class ForwardPath extends CommandBase {

    private final Trajectory path1 = PathPlanner.loadPath("TestPath1", AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    
    public ForwardPath() {};

    @Override
    public void initialize() {
        SequentialCommandGroup command = new SequentialCommandGroup(
            new DrivePath(RobotContainer.getSwerve(), path1, true)
        );

        command.schedule();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }

}
