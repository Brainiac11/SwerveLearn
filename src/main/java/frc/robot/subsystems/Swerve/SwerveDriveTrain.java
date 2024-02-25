package frc.robot.subsystems.Swerve;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.SteelTalonsLogger;

public class SwerveDriveTrain extends SubsystemBase {

    public static SwerveDriveTrain instance; // Creates a swerve drive train instance
    public static Pigeon2 gyro; // creates gyroscope
    public static List<SwerveModule> modules; // makes a list of SwerveModules that contains all 4 modules present in
                                              // the bot
    private ChassisSpeeds setPoint = new ChassisSpeeds(); // X: m/s - Y: m/s - Theta: rad/s
    // used to create the setPoints needed for the chassis to move towards

    private ChassisSpeeds adjustment = new ChassisSpeeds(); // the adjustment the chassis has to make to reach the
                                                            // desired setPoint

    public SwerveDriveTrain() {
        instance = this;
        gyro = new Pigeon2(10); // creates a new Pigeon object with CanID of 10
        gyro.reset(); // resets the gyro
        DriveTrainConstants.configureMotors(); // configures all the motors on the drive train

        modules = List.of(
                new SwerveModule(
                        DriveTrainConstants.FRONT_LEFT_DRIVE_ID,
                        DriveTrainConstants.FRONT_LEFT_STEER_ID,
                        DriveTrainConstants.FRONT_LEFT_DRIVE,
                        DriveTrainConstants.FRONT_LEFT_STEER,
                        DriveTrainConstants.FRONT_LEFT_CANCODER_ID,
                        DriveTrainConstants.FRONT_LEFT_OFFSET), // FRONT LEFT SWERVE MODULE
                new SwerveModule(
                        DriveTrainConstants.FRONT_RIGHT_DRIVE_ID,
                        DriveTrainConstants.FRONT_RIGHT_STEER_ID,
                        DriveTrainConstants.FRONT_RIGHT_DRIVE,
                        DriveTrainConstants.FRONT_RIGHT_STEER,
                        DriveTrainConstants.FRONT_RIGHT_CANCODER_ID,
                        DriveTrainConstants.FRONT_RIGHT_OFFSET), // FRONT RIGHT SWERVE MODULE
                new SwerveModule(
                        DriveTrainConstants.BACK_LEFT_DRIVE_ID,
                        DriveTrainConstants.BACK_LEFT_STEER_ID,
                        DriveTrainConstants.BACK_LEFT_DRIVE,
                        DriveTrainConstants.BACK_LEFT_STEER,
                        DriveTrainConstants.BACK_LEFT_CANCODER_ID,
                        DriveTrainConstants.BACK_LEFT_OFFSET), // BACK LEFT SWERVE MODULE
                new SwerveModule(
                        DriveTrainConstants.BACK_RIGHT_DRIVE_ID,
                        DriveTrainConstants.BACK_RIGHT_STEER_ID,
                        DriveTrainConstants.BACK_RIGHT_DRIVE,
                        DriveTrainConstants.BACK_RIGHT_STEER,
                        DriveTrainConstants.BACK_RIGHT_CANCODER_ID,
                        DriveTrainConstants.BACK_RIGHT_OFFSET) // BACK RIGHT SWERVE MODULE
        ); // This creates all 4 swerve modules and combines them into an easy to use List
    }

    public static SwerveDriveTrain getInstance() {
        return instance;
    }

    public Rotation2d geRotation() {
        return gyro.getRotation2d(); // returns the gyroscope's Rotation2d measurment of the chassis
    }

    public void setGyro(Rotation2d rot) {
        gyro.setYaw(rot.getDegrees()); // sets the Gyroscope to the given value (usually for resetting it)
    }

    public SwerveDriveWheelPositions getWheelPositions() {
        return new SwerveDriveWheelPositions(new SwerveModulePosition[] {
                modules.get(0).getModulePosition(),
                modules.get(1).getModulePosition(),
                modules.get(2).getModulePosition(),
                modules.get(3).getModulePosition()
        }); // returns a Custom List Containing the positions of all the modules
    }

    public void setSetpoint(ChassisSpeeds setPoint) {
        this.setPoint = setPoint; // Sets the Chassis to go to a the set position
    }

    public void adjustSpeeds(ChassisSpeeds adj) {
        this.adjustment = adj; // adjusts the speeds of the chassis
    }

    @Override
    public void periodic() {
        if (DriverStation.isTeleop()) {
            SwerveModuleState[] states = DriveTrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                    ChassisSpeeds.discretize(setPoint.plus(adjustment), Units.millisecondsToSeconds(20)));
            // HUGE LINE
            // basically it is calculating the states needed for moving the chassis by
            // utilizing the setpoints, current speeds, and adjustments
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveTrainConstants.MAX_PHYSICAL_SPEED_M_S);
            // if some of the motor speeds are higher than the maximum possible speeds, then
            // it will normalize them to match accordingly whiel maintaining the ratios

            for (int i = 0; i < modules.size(); i++) {
                modules.get(i).setModuleState(states[i]); // sets the existing states of each Module to the new states
            }
        }

        if (DriverStation.isDisabled()) { // If disabled, then reset every motor so it no longer moves
            for (int i = 0; i < modules.size(); i++) {
                modules.get(i).resetController();
            }
        }
        log();
    }

    public void setStatesAuton(SwerveModuleState[] states) { // same thing as in teleop, but in auton
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveTrainConstants.MAX_PHYSICAL_SPEED_M_S);
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).setModuleState(states[i]);
        }
    }

    public ChassisSpeeds getVelocityVector() {
        SwerveModuleState[] states = new SwerveModuleState[4]; // creates a new swervemodulestates list

        for (int i = 0; i < states.length; i++) {
            states[i] = modules.get(i).getModuleState(); // assigns the current states to the new list
        }

        ChassisSpeeds speeds = DriveTrainConstants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(states); // calculates the
                                                                                                    // states of the
                                                                                                    // modules currently
                                                                                                    // into a chassis
                                                                                                    // speed
        return speeds;
    }

    public ChassisSpeeds getDriveSpeeds(CommandXboxController controller) {
        double[] cv = {
                -controller.getRightY() * DriveTrainConstants.MAX_TRANSLATION_SPEED_M_S_TELEOP, // right Joystick will
                                                                                                // be for trasnalting
                                                                                                // the robot
                -controller.getRightX() * DriveTrainConstants.MAX_TRANSLATION_SPEED_M_S_TELEOP,
                Math.copySign(Math.abs(controller.getLeftX()), 2.0),
                -controller.getLeftX() * DriveTrainConstants.MAX_ROTATION_SPEED_RAD_S_TELEOP,
                // Left joystick is for rotating (spinning) the robot
                // Calculates the rotational speed the robot will be moving at

        };
        return ChassisSpeeds.fromFieldRelativeSpeeds(cv[0], cv[1], cv[2], this.geRotation()); // Returns the chassis
                                                                                              // speed for the xbox
                                                                                              // controllers inputs
    }

    public void log() {
        SteelTalonsLogger.post("Drivetrain Setpoint X", setPoint.vxMetersPerSecond);
        SteelTalonsLogger.post("Drivetrain Setpoint Y", setPoint.vyMetersPerSecond);
        SteelTalonsLogger.post("Drivetrain Setpoint Theta", setPoint.omegaRadiansPerSecond);
        SteelTalonsLogger.post("x speed", getVelocityVector().vxMetersPerSecond);
        SteelTalonsLogger.post("y speed", getVelocityVector().vyMetersPerSecond);
        // modules.get(0).log("front left");
        // modules.get(1).log("front right");
        // modules.get(2).log("back left");
        // modules.get(3).log("back right");
    }
    
    public Command getDriveCommand(CommandXboxController joy) {
        return new DriveCommand(getInstance(), joy);
    }

}
