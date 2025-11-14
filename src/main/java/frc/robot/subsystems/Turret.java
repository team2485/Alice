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


public class Turret extends SubsystemBase{
    public enum TurretStates{
        StateZero,
        StateIdle,
        StateMoving,
    }
    public TurretStates currentState = TurretStates.StateZero;
    public TurretStates requestedState = TurretStates.StateZero;

    private final TalonFX m_talon = new TalonFX(1, "rio");
    private PIDController controller = new PIDController(0, 0, 0);
    public DoubleSupplier axisSupplier;

    DoublePublisher voltagePub;
    DoublePublisher positionPub;
    DoublePublisher targetPub;

    public Turret(DoubleSupplier axisSupplier){
        this.axisSupplier = axisSupplier;
        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 2;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        var motorOutputConfigs = talonFXConfigs.MotorOutput;
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 70;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 120;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        m_talon.getConfigurator().apply(talonFXConfigs);

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
                double power = controller.calculate(m_talon.getPosition().getValueAsDouble(), 0);
                m_talon.set(power);
                break;
        
            case StateMoving:
                m_talon.set(axisSupplier.getAsDouble()*0.015);
                if((position > 2.5 && -axisSupplier.getAsDouble() > 0) || (position < -2.5 && -axisSupplier.getAsDouble() < 0)){
                    m_talon.set(0);
                }
                System.out.println(position/5);
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

    public void requestState(TurretStates requestedState){
        this.requestedState = requestedState;
    }
}
