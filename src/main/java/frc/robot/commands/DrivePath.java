package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DrivePath extends CommandBase {

    private Swerve swerve;
    private Trajectory path;
    private boolean isFirstPath;

    private PathPlannerState state;

    public DrivePath(Swerve iSwerve, Trajectory iPath, boolean iFirstPath) {
        swerve = iSwerve;
        path = iPath;
        isFirstPath = iFirstPath;

        addRequirements(iSwerve);
    }

    public DrivePath(Swerve iSwerve, Trajectory iPath) {
        this(iSwerve, iPath, false);
    }

    @Override
    public void initialize() {        
        if (isFirstPath) {
            swerve.resetOdometry(path.getInitialPose());
        }
    }

    @Override
    public void execute() {
        driveTrajectory(swerve, path);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        state = (PathPlannerState) path.getStates().get(path.getStates().size() - 1);
        if (Math.abs(swerve.getPose().getX() - state.poseMeters.getX()) < 0.1 && Math.abs(swerve.getPose().getY() - state.poseMeters.getY()) < 0.1) {
            return true;
        } else {
            return false;
        }
    }

    public void driveTrajectory(Swerve iSwerve, Trajectory iTrajectory) {
        var thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints
        );

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            iTrajectory,
            iSwerve::getPose, // Functional interface to feed supplier
            Constants.DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            iSwerve::setModuleStates,
            iSwerve
        );

        // Reset odometry to the starting pose of the trajectory.
        iSwerve.resetOdometry(iTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        swerveControllerCommand.andThen(() -> iSwerve.drive(0, 0, 0, false, false));
        }
    
}
