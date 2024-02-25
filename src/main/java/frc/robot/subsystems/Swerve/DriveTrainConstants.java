package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Current;
import frc.robot.util.STSmaxConfig;

public class DriveTrainConstants {
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.88); // Diameter of the wheels
    public static final double TRACKWIDTH = Units.inchesToMeters(19.5); // Distance from Centers of Wheels diagonally
    public static final double WHEELBASE = Units.inchesToMeters(19.5); // Distance from Centers of Wheels horizontally
                                                                       // and vertically
    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEELBASE / 2, TRACKWIDTH / 2), // Calculates the distance in X and Y from the Robot
                                                              // Center
            new Translation2d(WHEELBASE / 2, -TRACKWIDTH / 2), // Different wheels which cover all 4 quadrants of the
                                                               // robot, assuming origin is in Center
            new Translation2d(-WHEELBASE / 2, TRACKWIDTH / 2),
            new Translation2d(-WHEELBASE / 2, -TRACKWIDTH / 2));

    // MAX_PHYSICAL_SPEED_M_S calculates the Maximum Speed of the Kraken Motors
    // (6000) and multiplies it by the gear ratio
    // then multiplies the Diameter of the wheels and PI to find the max speed the
    // wheel can turn, then divided by 60 to convert to M_S
    public static final double MAX_PHYSICAL_SPEED_M_S = ((6000 * (16.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0)
            * WHEEL_DIAMETER_METERS * Math.PI) / 60.0);

    public static final double MAX_TRANSLATION_SPEED_M_S_TELEOP = MAX_PHYSICAL_SPEED_M_S * 1.0; // Maximum Speed when
                                                                                                // going diagonal, since
                                                                                                // the robot is a square
                                                                                                // this will be 1 value
                                                                                                // == to the max speed
    public static final double MAX_ROTATION_SPEED_RAD_S_TELEOP = 3 * Math.PI; // Maximum is 3 pi radians a second to
                                                                              // prevent too fast movements

    // Add Pidgeon and Motor CANCODER IDs

    public static final int PIGEON_CAN_ID = 16; // dummy values
    public static final int FRONT_LEFT_DRIVE_ID = 0; // dummy values
    public static final int FRONT_RIGHT_DRIVE_ID = 1;
    public static final int BACK_LEFT_DRIVE_ID = 2;
    public static final int BACK_RIGHT_DRIVE_ID = 3;

    public static final double FRONT_LEFT_OFFSET = 0.48; // Calibrated Offsets
    public static final double FRONT_RIGHT_OFFSET = 0.255; // Calibrated Offsets
    public static final double BACK_LEFT_OFFSET = -0.38; // Calibrated Offsets
    public static final double BACK_RIGHT_OFFSET = 0.278; // Calibrated Offsets

    public static final TalonFXConfiguration FRONT_LEFT_DRIVE = new TalonFXConfiguration(); // Creates a DRIVE motor
                                                                                            // configuration
    public static final TalonFXConfiguration BACK_LEFT_DRIVE = new TalonFXConfiguration(); // Creates a DRIVE motor
                                                                                           // configuration
    public static final TalonFXConfiguration FRONT_RIGHT_DRIVE = new TalonFXConfiguration(); // Creates a DRIVE motor
                                                                                             // configuration
    public static final TalonFXConfiguration BACK_RIGHT_DRIVE = new TalonFXConfiguration(); // Creates a DRIVE motor
                                                                                            // configuration

    // ADD MOTOR IDs

    public static final int FRONT_LEFT_STEER_ID = 4; // dummy values
    public static final int FRONT_RIGHT_STEER_ID = 5;
    public static final int BACK_LEFT_STEER_ID = 6;
    public static final int BACK_RIGHT_STEER_ID = 7;

    
    public static final TalonFXConfiguration FRONT_LEFT_STEER = new TalonFXConfiguration(); // Creates a STEER motor
                                                                                            // configuration
    public static final TalonFXConfiguration BACK_LEFT_STEER = new TalonFXConfiguration();
    public static final TalonFXConfiguration FRONT_RIGHT_STEER = new TalonFXConfiguration();
    public static final TalonFXConfiguration BACK_RIGHT_STEER = new TalonFXConfiguration();

    public static final int FRONT_LEFT_CANCODER_ID = 12; // Dummy Values
    public static final int FRONT_RIGHT_CANCODER_ID = 13;
    public static final int BACK_LEFT_CANCODER_ID = 14;
    public static final int BACK_RIGHT_CANCODER_ID = 15;

    public static void configureMotors() {
        FRONT_RIGHT_DRIVE.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Wheel is between pi/2 and -pi/2 in
                                                                                   // coordinate plane
        BACK_RIGHT_DRIVE.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Wheel is between pi/2 and -pi/2 in
                                                                                  // coordinate plane
        FRONT_LEFT_DRIVE.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Wheel is between pi/2 and
                                                                                         // 3pi/2 in coordinate plane
        BACK_LEFT_DRIVE.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Wheel is between pi/2 and
                                                                                        // 3pi/2 in coordinate plane
    }

    public static void configureDriveTalons(TalonFX motor) {
        Slot0Configs velConstants = new Slot0Configs();
        velConstants.kP = 0.1; // proportional gain
        velConstants.kS = 0.0; // No idea what this does? apprently it is Static Feedforward Gain
        velConstants.kV = 12 / (MAX_PHYSICAL_SPEED_M_S); // How fast the velocity is increased by, increased in
                                                         // incrementations
        motor.getConfigurator().apply(velConstants); // applying the constants that was defined to the given TalonFx
                                                     // motor

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs(); // Motor Feedback intilization
        feedbackConfigs.SensorToMechanismRatio = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0); // gear ratio multiplications for sensor to
                                                                        // match motors
        motor.getConfigurator().apply(feedbackConfigs); // applying the feedBackConfigs defined above to the motor

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs(); // configures the amount of electroncity
                                                                                // the motor should have
        currentLimitsConfigs.StatorCurrentLimitEnable = true; // when using, limit amount of power going into the motor
                                                              // stator for movement can be limited: yes or no
        currentLimitsConfigs.SupplyCurrentLimitEnable = true; // when moving, amount of power going into the motor can
                                                              // be limited: yes or no
        currentLimitsConfigs.StatorCurrentLimit = 70; // amount of amps when running going into stator;
        currentLimitsConfigs.SupplyCurrentLimit = 60; // amount of amps when running motor into the rotor;
        currentLimitsConfigs.SupplyCurrentThreshold = 120; // only activate limiter when reaching this value
        currentLimitsConfigs.SupplyTimeThreshold = 0.2; // how long the maximum current can be sent to motor in seconds
        motor.getConfigurator().apply(currentLimitsConfigs); // applys the current configurations into the motor

        motor.setNeutralMode(NeutralModeValue.Brake); // when motors are on neutral (not wanting to be ran), they should
                                                      // be on brake
    }

    public static void configureSteerTalons(TalonFX motor) { // this method is sketchy as hell pls verify this
        Slot0Configs velConstants = new Slot0Configs();
        velConstants.kP = 7.0; // proportional gain
        velConstants.kD = 0.15; // Derivative Gain 1.6?
        velConstants.kV = 12 / (MAX_PHYSICAL_SPEED_M_S); // How fast the velocity is increased by, increased in
                                                         // incrementations
        motor.getConfigurator().apply(velConstants); // applying the constants that was defined to the given TalonFx
                                                     // motor

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs(); // configures the amount of electroncity
                                                                                // the motor should have
        currentLimitsConfigs.StatorCurrentLimitEnable = true; // when using, limit amount of power going into the motor
                                                              // stator for movement can be limited: yes or no
        currentLimitsConfigs.SupplyCurrentLimitEnable = true; // when moving, amount of power going into the motor can
                                                              // be limited: yes or no
        currentLimitsConfigs.StatorCurrentLimit = 70; // amount of amps when running going into stator;
        currentLimitsConfigs.SupplyCurrentLimit = 60; // amount of amps when running motor into the rotor;
        currentLimitsConfigs.SupplyCurrentThreshold = 120; // only activate limiter when reaching this value
        currentLimitsConfigs.SupplyTimeThreshold = 0.2; // how long the maximum current can be sent to motor in seconds
        motor.getConfigurator().apply(currentLimitsConfigs); // applys the current configurations into the motor

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs(); // Motor Feedback intilization
        feedbackConfigs.SensorToMechanismRatio = ((150.0 / 7.0) / 1.0); // gear ratio multiplications for sensor to
                                                                        // match motors
        motor.getConfigurator().apply(feedbackConfigs); // applying the feedBackConfigs defined above to the motor
        motor.setNeutralMode(NeutralModeValue.Brake); // when motors are on neutral (not wanting to be ran), they should
                                                      // be on brake
    }

    public static void configureCanCoders(CANcoder encoder, double offset) {

        CANcoderConfiguration configuration = new CANcoderConfiguration(); // Creates a new CanCoder Configuration for
                                                                           // determining angles
        configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; // range that
                                                                                                        // the sensor
                                                                                                        // can detect
        configuration.MagnetSensor.MagnetOffset = offset; // offset of the magnetic sensor due to inaccuracies
        configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // Direction that
                                                                                                     // the sensor
                                                                                                     // should count the
                                                                                                     // values in

        encoder.getConfigurator().apply(configuration);

    }

}
