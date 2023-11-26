package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.*;

public class AutoChooser {

    private SendableChooser<CommandBase> chooser;
    private CommandBase selectedCommand;

    // List of commands for autonomous
    private CommandBase[] commands = {
        new EmptyAuto()
    };

    /**
     * Creates tab in Shuffleboard with list of specified commands ({@link AutoChooser#commands})
     */
    public AutoChooser() {
        chooser = new SendableChooser<>();
        chooser.setDefaultOption("Empty sequence", commands[0]);
        for(CommandBase cmd : commands){
            chooser.addOption(cmd.getName(), cmd);
        }
        Shuffleboard.getTab("Main")
                .add("Auton Selection", chooser)
                .withSize(3, 1)
                .withPosition(2, 1);
    }

    /**
     * Reads the selected autonomous command from the SendableChooser.
     */
    public void readCommand() {
        selectCommand(chooser.getSelected());
    }

    /**
     * Selects an autonomous command to run.
     * 
     * @param iComand The autonomous command to run
     */
    public void selectCommand(CommandBase iCommand) {
        selectedCommand = iCommand;
    }

    /**
     * <ul>
     * <li>Returns the currently selected autonomous command.</li>
     * <li>Defaults to {@link EmptyAuto} if nothing is selected</li>
     * </ul>
     * 
     * @return Currently selected autonomous command
     */
    public CommandBase getSelectedCommand() {
        return selectedCommand;
    }
    
}
