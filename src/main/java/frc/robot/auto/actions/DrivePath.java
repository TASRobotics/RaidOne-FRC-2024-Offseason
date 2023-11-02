package frc.robot.auto.actions;

/**
 * similar to the command interface from WPIlib
 * all actions should utilize this interface
 */

public class DrivePath implements Action {

    /**
     * returns whether the action is finished or not
     * 
     * @return if the action is finished
     */
    public boolean isDone() {
        return true;
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

    }

    /**
     * runs once when the action first starts
     */
    public void initialize() {

    }

}
