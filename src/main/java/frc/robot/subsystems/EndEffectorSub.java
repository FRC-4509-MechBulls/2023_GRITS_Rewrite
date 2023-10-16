package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class EndEffectorSub extends SubsystemBase {
    private final TalonSRX endEffectorLower;
    private final TalonSRX endEffectorUpper;

    public EndEffectorSub() {
        endEffectorLower = new TalonSRX(13);
        endEffectorUpper= new TalonSRX(14);

        endEffectorUpper.configFactoryDefault();
        endEffectorUpper.configFactoryDefault();

        endEffectorUpper.setNeutralMode(NeutralMode.Coast);
        endEffectorLower.setNeutralMode(NeutralMode.Coast);


        endEffectorUpper.setInverted(true);
        endEffectorLower.setInverted(false);

        endEffectorLower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                true, // enabled
                10, // Limit (amp)
                10, // Trigger Threshold (amp)
                0)); // Trigger Threshold Time(s)


        endEffectorUpper.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                true, // enabled
                10, // Limit (amp)
                10, // Trigger Threshold (amp)
                0)); // Trigger Threshold Time(s)


    }

    public void intakeCone() {
        endEffectorUpper.set(TalonSRXControlMode.PercentOutput, -1);
        endEffectorLower.set(TalonSRXControlMode.PercentOutput, 1);
    }

    public void intakeCube() {
        endEffectorUpper.set(TalonSRXControlMode.PercentOutput, 0);
        endEffectorLower.set(TalonSRXControlMode.PercentOutput, -1);
    }

    public void holdCone() {

        endEffectorUpper.set(TalonSRXControlMode.PercentOutput, -0.1);
        endEffectorLower.set(TalonSRXControlMode.PercentOutput, 0.1);
    }

    public void holdCube() {

        endEffectorUpper.set(TalonSRXControlMode.PercentOutput, 0);
        endEffectorLower.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void placeCone() {
        endEffectorUpper.set(TalonSRXControlMode.PercentOutput, 1);
        endEffectorLower.set(TalonSRXControlMode.PercentOutput, -.1);
    }

    public void placeCubeBottom() {
        endEffectorUpper.set(TalonSRXControlMode.PercentOutput, 1);
        endEffectorLower.set(TalonSRXControlMode.PercentOutput, -0.3);
    }
    public void placeCubeTop() {

        endEffectorUpper.set(TalonSRXControlMode.PercentOutput, -1);
        endEffectorLower.set(TalonSRXControlMode.PercentOutput, -0.3);
    }

    public void stopMotors() {
        endEffectorUpper.set(TalonSRXControlMode.PercentOutput, 0);
        endEffectorLower.set(TalonSRXControlMode.PercentOutput, 0);
    }

}