package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class TankBase extends SubsystemBase {
  private final CANBus rio = new CANBus("rio");
  private final TalonFX rMotor = new TalonFX(0, rio);
  private final TalonFX lMotor = new TalonFX(1, rio);

  private AnalogGyro m_gyro = new AnalogGyro(1);
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  static final double KvLinear = 1.98;
  static final double KaLinear = 0.2;
  static final double KvAngular = 1.5;
  static final double KaAngular = 0.3;

  private DifferentialDriveOdometry m_odometry =
      new DifferentialDriveOdometry(
          m_gyro.getRotation2d(),
          lMotor.getPosition().getValueAsDouble(),
          rMotor.getPosition().getValueAsDouble());
  private Field2d m_field = new Field2d();

  private final DCMotorSim lMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1), // 😊
          DCMotor.getKrakenX60(1));
  private final DCMotorSim rMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
          DCMotor.getKrakenX60(1));

  DifferentialDrivetrainSim drivetrainSim =
      new DifferentialDrivetrainSim(
          LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
          DCMotor.getKrakenX60(1), // motor types
          1, // gear ratio 5:1
          Units.inchesToMeters(15), // track width
          Units.inchesToMeters(2.8), // wheel radius
          null);

  public TankBase() {
    m_odometry.resetPose(new Pose2d());
    m_gyroSim.setAngle(0);
    var slot0Configs = new Slot0Configs();
    TalonFXConfiguration config = new TalonFXConfiguration();
    slot0Configs.kP = 4;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.5;
    slot0Configs.kS = 0.3;
    rMotor.getConfigurator().apply(config.withSlot0(slot0Configs));
    lMotor.getConfigurator().apply(config.withSlot0(slot0Configs));

    SmartDashboard.putData("Field", m_field);
  }

  private void drive(DoubleSupplier speed, DoubleSupplier turn) {
    System.out.println("Speed: " + speed.getAsDouble() + " Turn: " + turn.getAsDouble());
    lMotor.set((-speed.getAsDouble() + turn.getAsDouble()));
    rMotor.set((turn.getAsDouble() + speed.getAsDouble()));
  }

  private void stop() {
    lMotor.set(0);
    rMotor.set(0);
  }

  private void resetPose() {
    lMotor.setPosition(0);
    rMotor.setPosition(0);
    m_odometry.resetPose(new Pose2d());
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  public Command driveCommand(DoubleSupplier speed, DoubleSupplier turn) {
    return runEnd(() -> drive(speed, turn), () -> stop());
  }

  public Command resetPoseCommand() {
    return runOnce(() -> resetPose());
  }

  public void periodic() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        lMotor.getPosition().getValueAsDouble(),
        rMotor.getPosition().getValueAsDouble());
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  public void simulationInit() {
    var LtalonFXSim = lMotor.getSimState();
    LtalonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    var RtalonFXSim = rMotor.getSimState();
    RtalonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    var LtalonFXSim = lMotor.getSimState();
    LtalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    var LmotorVoltage = LtalonFXSim.getMotorVoltageMeasure();
    lMotorSim.setInputVoltage(LmotorVoltage.in(Volts)); // assume 20 ms loop time
    LtalonFXSim.setRawRotorPosition(lMotorSim.getAngularPosition());
    LtalonFXSim.setRotorVelocity(lMotorSim.getAngularVelocity());
    lMotorSim.update(0.020);

    var RtalonFXSim = rMotor.getSimState();
    RtalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    var RmotorVoltage = RtalonFXSim.getMotorVoltageMeasure();
    rMotorSim.setInputVoltage(RmotorVoltage.in(Volts));
    RtalonFXSim.setRawRotorPosition(rMotorSim.getAngularPosition());
    RtalonFXSim.setRotorVelocity(rMotorSim.getAngularVelocity());
    rMotorSim.update(0.020); // assume 20 ms loop time

    drivetrainSim.setInputs(
        lMotorSim.getInputVoltage() * lMotor.get(), rMotorSim.getInputVoltage() * rMotor.get());
    drivetrainSim.update(0.02);
    m_gyroSim.setAngle(
        -drivetrainSim
            .getHeading()
            .getDegrees()); // heading is negative bc hover over getHeading man
  }
}
