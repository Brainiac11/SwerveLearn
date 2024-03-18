package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private TalonFX rollerTalon;
    private TalonFX pivotTalon;

    private TalonFXConfiguration rollerConfig;
    private TalonFXConfiguration pivotConfig;

    private double rolelrSetpoint = IntakeConstants.INTAKE_SPEED_HOLD;
    private Rotation2d pivotSetpoint = IntakeConstants.STOPPED_POS;

    private static Intake instance;

    private boolean isHoming;

    private boolean isBeamBroken;

    private DigitalInput beamBreak;

    public Intake() {
        rollerTalon = new TalonFX(IntakeConstants.KRAKEN_ROLLER_MOTOR_PORT);
        pivotTalon = new TalonFX(IntakeConstants.INTAKE_PIVOT_PORT);

        rollerConfig = new TalonFXConfiguration();
        pivotConfig = new TalonFXConfiguration();

        instance = this;

        rollerConfig.Slot0.kP = 0.1;
        rollerConfig.Slot0.kI = 0.0;
        rollerConfig.Slot0.kD = 0.0;

        resetPivotSetpoint(IntakeConstants.STOPPED_POS);

        beamBreak = new DigitalInput(IntakeConstants.BEAM_BREAK_PORT);
        isBeamBroken = beamBreak.get();

        isHoming = false;
    }

    private Intake getInstance() {
        return this;
    }

    private void resetPivotSetpoint(Rotation2d setpoint) {
        pivotSetpoint = setpoint;
    }
}
