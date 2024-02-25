package frc.robot.io;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class PilotingControls {
    public PilotingControls(CommandXboxController controller) {

        // controller.y().onTrue(new InstantCommand(() -> {
        //     SteelTalonsLocalization.getInstance().resetPose(
        //         MiscUtil.isBlue() ? MiscUtil.resetPose()[0] : MiscUtil.resetPose()[1]
        //     );
        //     SwerveDrivetrain.getInstance().resetGyro(new Rotation2d());
        // });

        

        // controller.rightTrigger(0.1).onTrue(new Command() {
        //     @Override
        //     public void initialize() {
        //         addRequirements(Climber.getInstance());
        //         Climber.getInstance().setSetpoint(ClimberConstants.CLIMB_POSITION);
        //     }
        // });

        // controller.leftTrigger(0.1 ).onTrue(Climber.getInstance().getClimbCommand());

    }
}