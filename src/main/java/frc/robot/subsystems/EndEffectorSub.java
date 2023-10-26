package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class EndEffectorSub extends SubsystemBase {
    private final TalonSRX lower;
    private final TalonSRX upper;

    public EndEffectorSub() {
        lower = new TalonSRX(Constants.EfConstants.EF_LOWER_PORT);
        upper = new TalonSRX(Constants.EfConstants.EF_UPPER_PORT);

        upper.configFactoryDefault();
        upper.configFactoryDefault();

        upper.setNeutralMode(NeutralMode.Coast);
        lower.setNeutralMode(NeutralMode.Coast);


        upper.setInverted(true);
        lower.setInverted(false);

        lower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                true, // enabled
                10, // Limit (amp)
                10, // Trigger Threshold (amp)
                0)); // Trigger Threshold Time(s)


        upper.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                true, // enabled
                10, // Limit (amp)
                10, // Trigger Threshold (amp)
                0)); // Trigger Threshold Time(s)


        holdCone();

    }

    public void intakeCone() {
        upper.set(TalonSRXControlMode.PercentOutput, -1);
        lower.set(TalonSRXControlMode.PercentOutput, 1);
    }

    public void intakeCube() {
        upper.set(TalonSRXControlMode.PercentOutput, 0);
        lower.set(TalonSRXControlMode.PercentOutput, -1);
    }

    public void holdCone() {

        upper.set(TalonSRXControlMode.PercentOutput, -0.1);
        lower.set(TalonSRXControlMode.PercentOutput, 0.1);
    }

    public void holdCube() {

        upper.set(TalonSRXControlMode.PercentOutput, 0);
        lower.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void placeCone() {
        upper.set(TalonSRXControlMode.PercentOutput, 1);
        lower.set(TalonSRXControlMode.PercentOutput, -1);
    }

    public void placeCubeBottom() {
        upper.set(TalonSRXControlMode.PercentOutput, 1);
        upper.set(TalonSRXControlMode.PercentOutput, 1);
        lower.set(TalonSRXControlMode.PercentOutput, 0.3);
    }
    public void placeCubeTop() {

        upper.set(TalonSRXControlMode.PercentOutput, -1);
        lower.set(TalonSRXControlMode.PercentOutput, -0.3);
    }

    public void stopMotors() {
        upper.set(TalonSRXControlMode.PercentOutput, 0);
        lower.set(TalonSRXControlMode.PercentOutput, 0);
    }

}