package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class Arm extends SubsystemBase{
    public enum ArmStates{
        StateZero,
        StateIdle,
        StateShooting,
    }
    public ArmStates currentState = ArmStates.StateZero;
    public ArmStates requestedState = ArmStates.StateZero;

    private final TalonFX m_talon = new TalonFX(2, "rio");
    private PIDController controller = new PIDController(0.2, 0, 0);

    DoublePublisher voltagePub;
    DoublePublisher positionPub;
    DoublePublisher targetPub;

    public Arm(){
        var talonFXConfigs = new TalonFXConfiguration();
        //var slot0Configs = talonFXConfigs.Slot0;
        // slot0Configs.kP = .02;
        // slot0Configs.kI = 0;
        // slot0Configs.kD = .01;

        var motorOutputConfigs = talonFXConfigs.MotorOutput;
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 70;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 120;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        m_talon.getConfigurator().apply(talonFXConfigs);
        controller.reset();
        m_talon.setPosition(0);
        controller.setTolerance(0);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
    
        NetworkTable table = inst.getTable("datatable");

        voltagePub = table.getDoubleTopic("voltage").publish();
        positionPub = table.getDoubleTopic("position").publish();
        targetPub = table.getDoubleTopic("target").publish();
    }
    
    double voltage;
    double position;
    double target;
    @Override
    public void periodic(){
        switch (currentState) {
            case StateZero:
            // System.out.println(m_talon.getPosition());
                double power = controller.calculate(m_talon.getPosition().getValueAsDouble(), 0);
                m_talon.set(power);
                break;
        
            case StateShooting:
                double power2 = controller.calculate(m_talon.getPosition().getValueAsDouble(), -0.15 * 4);    
                m_talon.set(power2);
                // System.out.println(position/4);
                break;
            case StateIdle:
                m_talon.set(0);    
                break;

        }

        currentState = requestedState;
        voltage = m_talon.getMotorVoltage().getValueAsDouble();
        voltagePub.set(voltage);
        position = m_talon.getPosition().getValueAsDouble();
        positionPub.set(position);
        
    }

    public void requestState(ArmStates requestedState){
        this.requestedState = requestedState;
    }
}
