package frc.robot.auto.actions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * similar to the command interface from WPIlib
 * all actions should utilize this interface
 */

public class DrivePath implements Action {

    private Swerve swerve;

    private Trajectory trajectory;
    private boolean isFirstPath;
    private Timer timer = new Timer();

    /**
     * Constructs a DrivePath action that follows a path.
     * 
     * @param path the path to follow
     */
    public DrivePath(Swerve iSwerve, Trajectory iPath) {
        this(iSwerve, iPath, false);
    }

    /**
     * Constructs a DrivePath action that follows a path.
     * 
     * @param path the path to follow
     * @param isFirstPath whether this is the first path in an autonomous sequence
     */
    public DrivePath(Swerve iSwerve, Trajectory iTrajectory, boolean iIsFirstPath) {
        swerve = iSwerve;
        iTrajectory = trajectory;
        isFirstPath = iIsFirstPath;
    }

    /**
     * returns whether the action is finished or not
     * 
     * @return if the action is finished
     */
    public boolean isDone() {
        return timer.advanceIfElapsed(trajectory.getTotalTimeSeconds());
    }

    /**
     * runs periodicly every 20? ms until isDone returns true
     */
    public void update() {

    }

    /**
     * runs once after action finishes
     */
    public void done() {
        swerve.setX();
    }

    /**
     * runs once when the action first starts
     */
    public void initialize() {
        if (isFirstPath) {
            /**
             * Sets the odometry pose to the start of the trajectory.
             *! Note: Should only do this on the first trajectory.
             */ 
            swerve.resetEncoders();
            swerve.resetOdometry(trajectory.getInitialPose());
        }

        swerve.driveTrajectory(trajectory);
    }

}
