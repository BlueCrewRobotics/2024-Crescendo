package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This Command will vibrate the controller according to how far the joysticks are pushed
 */
public class RumbleControllerWhenDriving extends Command {
    private CommandXboxController driver;

    /**
     * @param driver The Driver's Xbox Controller
     */
    public RumbleControllerWhenDriving(CommandXboxController driver) {
        this.driver = driver;
    }

    @Override
    public void execute() {
        if(Math.abs(driver.getLeftX()) > 0.1 || Math.abs(driver.getLeftY()) > 0.1 || Math.abs(driver.getRightX()) > 0.1) {
            driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, Math.max(Math.max(Math.abs(driver.getLeftX()), Math.abs(driver.getLeftY())), Math.abs(driver.getRightX())));
        } else {
            driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }
}
