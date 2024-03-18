package frc.robot.subsystems.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static final int INTAKE_PIVOT_PORT = 0;
    public static final int KRAKEN_ROLLER_MOTOR_PORT = 1;

    public static final double MAX_KRAKEN_ROLLER_SPEED = (6000.0 * Math.PI * Units.inchesToMeters(2.0) * (12.0 / 30.0))
            / 60.0;

    public static final Rotation2d STOPPED_POS = new Rotation2d(Units.degreesToRadians(0));
    public static final Rotation2d INTAKING_POS = new Rotation2d(Units.degreesToRadians(-200.0));
    public static final Rotation2d HANDOFF_POS = new Rotation2d(Units.degreesToRadians(-35.0));
    public static final Rotation2d REST_POS = new Rotation2d(Units.degreesToRadians(-50.0));
    public static final Rotation2d EJECTING_POS = new Rotation2d(Units.degreesToRadians(-110.0));

    public static final double INTAKE_SPEED_INTAKING = 5.0;
    public static final double INTAKE_SPEED_HANDOFF = -0.4;
    public static final double INTAKE_SPEED_EJECTING = -5.0;
    public static final double INTAKE_SPEED_REST = 1.0;
    public static final double INTAKE_SPEED_HOLD = 0.25;

    public static final double PIVOT_TOLERANCE_RAD = Units.degreesToRadians(2.0);

    public static final int BEAM_BREAK_PORT = 10;

    public static void configureIntake() {
        // FIXME Will do config stuff later
    }

}
