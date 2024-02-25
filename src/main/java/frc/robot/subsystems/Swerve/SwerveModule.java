package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.SteelTalonsLogger;
// import frc.robot.util.SmaxProfiles.SteelTalonsSparkMaxSimpleServo;

public class SwerveModule {
    // private static final DriveTrainConstants drivetrainConstants = new DriveTrainConstants();
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANcoder canCoder;

    public SwerveModule(int driveTalonID, int steerTalonID, TalonFXConfiguration driveConfig,
            TalonFXConfiguration steerConfig, int canCoderID, double offset) {

        driveMotor = new TalonFX(driveTalonID); // Applies talonID to the driveMotor
        driveMotor.getConfigurator().apply(driveConfig); // applies to config to the drive motor
        DriveTrainConstants.configureDriveTalons(driveMotor); // applies the constants in DriveTrainConstants to the
                                                              // motor

        steerMotor = new TalonFX(steerTalonID); // Applies talonID to the steerMotor
        steerMotor.getConfigurator().apply(steerConfig); // applies to config to the steer motor
        DriveTrainConstants.configureSteerTalons(steerMotor); // applies the constants in DriveTrainConstants to the
                                                              // motor

        driveMotor.setPosition(0); // sets the drive motor position to 0
        steerMotor.setPosition(canCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI); // sets the steer motor
                                                                                                 // position to the
                                                                                                 // value of the encoder
                                                                                                 // converted from a
                                                                                                 // semi circle to a
                                                                                                 // full circle then
                                                                                                 // converted into
                                                                                                 // radians
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(DTrotToMeters(driveMotor.getPosition().getValueAsDouble()),
                new Rotation2d(steerMotor.getPosition().getValueAsDouble()));
        // returns the swerve module Position
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(DTrotToMeters(driveMotor.getVelocity().getValueAsDouble()),
                new Rotation2d(steerMotor.getPosition().getValueAsDouble()));
        // Returns the swerve module state
    }

    private double DTrotToMeters(double rotations) {
        return rotations * Math.PI * DriveTrainConstants.WHEEL_DIAMETER_METERS; // returns the rotations into meters
    }

    private double DTMetersToTrot(double meters) {
        return meters / (Math.PI * DriveTrainConstants.WHEEL_DIAMETER_METERS); // return the meters of movement into
                                                                               // rotations
    }

    public Rotation2d canCoderRot() {
        return new Rotation2d(canCoder.getAbsolutePosition().getValueAsDouble()); // converts the absolute position from
                                                                                  // canCoder into a Rotation2d object
    }

    public void setModuleState(SwerveModuleState state) {
        SwerveModuleState newState = SwerveModuleState.optimize(state, canCoderRot()); // Creates a new state that is
                                                                                       // both optimal and calibrated
                                                                                       // for the canCoder Rotations and
                                                                                       // specific module

        double velocitySetpoint = DTMetersToTrot(newState.speedMetersPerSecond); // target velocity of the new state
        Rotation2d rotSetpoint = newState.angle; // target rotation of the new state

        if (Math.abs(velocitySetpoint) > 0.00) { // if the velocity setpoint is bigger than 0, which would mean we would
                                                 // have to move
            driveMotor.setControl(new VelocityVoltage(velocitySetpoint).withEnableFOC(true)); // sets the drive motor
                                                                                              // with a voltage to
                                                                                              // prompt it to go the
                                                                                              // given velocity with
                                                                                              // easier method chaining
            steerMotor.setPosition(rotSetpoint.getRotations()); // sets the target position of the motors to be at using
                                                                // the rotational setpoint
        } else { // if the velocity setpoint is equal to 0 (meaning we dont have to do anything)
            driveMotor.stopMotor(); // Stop the driving motor
            steerMotor.stopMotor(); // Stop the steering motor
        }
    }

    public void resetController() {
        steerMotor.setPosition(canCoder.getPosition().getValueAsDouble()); // Unsure whether needed?
    }

    public void log(String name) { // just logs the data
        SteelTalonsLogger.post(name + " velocity", getModuleState().speedMetersPerSecond);
        SteelTalonsLogger.post(name + " position", getModulePosition().distanceMeters);
        SteelTalonsLogger.post(name + " angle", getModuleState().angle.getRadians());
        SteelTalonsLogger.post(name + " absolute angle", canCoder.getAbsolutePosition().getValueAsDouble());
        // steerMotor.log();
    }

}
