package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommand extends Command {
    private CommandXboxController xboxController; // creates a new Command based Xbox Controller for controlling the
                                                  // Robot
    private SwerveDriveTrain driveTrain; // Creates the SwerveDriveTrain for controlling swerve

    public DriveCommand(SwerveDriveTrain driveTrain, CommandXboxController xboxController) {
        addRequirements(driveTrain); // add the requriments of the subsystem, will prevent the susbsytem from running
                                     // duplicated objects and actions
        this.driveTrain = driveTrain;
        this.xboxController = xboxController;
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = driveTrain.getDriveSpeeds(xboxController); // Gets the nessesary Rotations, Speeds, and
                                                                          // Translations for moving the Chassis based
                                                                          // on the Controller
        driveTrain.setSetpoint(speeds); // Sets the Chassis to move in the desired mechanism from the controller
    }

    public void executeAutoAlign(ChassisSpeeds speeds) { // Made for autoaligning to Note, Speaker or Amp, but is the
                                                         // same principle as execute()
        driveTrain.setSetpoint(speeds);
    }

    @Override
    public boolean isFinished() {
        return false; // SWERVE DRIVE TRAIN NEVER STOPPSSS RAHHHHHHHH 🚄🚄🚄🚄
    }

}
