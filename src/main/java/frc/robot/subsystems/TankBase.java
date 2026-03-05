package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

// Gear Ratio: 8.3:1, Track width, 17 inches, 2.85 inches diameter
// I'm so tiredness

public class TankBase extends SubsystemBase {
  private final CANBus rio = new CANBus("rio");
  private final TalonFX rMotor = new TalonFX(0, rio);
  private final TalonFX lMotor = new TalonFX(1, rio);

  private AnalogGyro m_gyro = new AnalogGyro(1);
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  static final double KvLinear = 1;
  static final double KaLinear = 0.1;
  static final double KvAngular = 0.7;
  static final double KaAngular = 0.15;

  private DifferentialDriveOdometry m_odometry =
      new DifferentialDriveOdometry(
          m_gyro.getRotation2d(),
          lMotor.getPosition().getValueAsDouble() * (2*Math.PI*Constants.kWheelRadius),
          rMotor.getPosition().getValueAsDouble() * (2*Math.PI*Constants.kWheelRadius),
          new Pose2d(0,0, new Rotation2d()));
  
  private Field2d m_field = new Field2d();

  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.kTrackWidth);

  // private final DCMotorSim lMotorSim =
  //     new DCMotorSim(
  //         LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1), // 😊
  //         DCMotor.getKrakenX60(1));
  // private final DCMotorSim rMotorSim =
  //     new DCMotorSim(
  //         LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
  //         DCMotor.getKrakenX60(1));

  DifferentialDrivetrainSim drivetrainSim =
      new DifferentialDrivetrainSim(
          LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
          DCMotor.getKrakenX60(1),
          Constants.kWheelGearRatio, 
          Constants.kTrackWidth,
          Constants.kWheelRadius,
          null);

  public TankBase() {
    resetPose();
    var slot0Configs = new Slot0Configs();
    TalonFXConfiguration config = new TalonFXConfiguration();
    slot0Configs.kP = 4;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.5;
    slot0Configs.kS = 0.3;
    rMotor.getConfigurator().apply(config.withSlot0(slot0Configs));
    lMotor.getConfigurator().apply(config.withSlot0(slot0Configs));
    lMotor.setNeutralMode(NeutralModeValue.Brake);
    rMotor.setNeutralMode(NeutralModeValue.Brake);
    SmartDashboard.putData("Field", m_field);
  }

private void drive(DoubleSupplier speed, DoubleSupplier turn) {
    // Max free speed for Kraken X60 with 8.3:1 and 2.85" wheels
    // v = (free_rpm / gear_ratio) * 2π * radius
    // ≈ (6000 / 8.3) * 2π * 0.0362 ≈ 1.64 m/s (conservative)
    final double kMaxSpeedMetersPerSecond = 1.64;

    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(
            deadband(speed.getAsDouble()),
            0,
            Units.degreesToRadians(deadband(turn.getAsDouble()) * 180))
    );
    System.out.println("left: " + wheelSpeeds.leftMetersPerSecond + " right: " + wheelSpeeds.rightMetersPerSecond);
    lMotor.set(-wheelSpeeds.leftMetersPerSecond / kMaxSpeedMetersPerSecond);
    rMotor.set(-wheelSpeeds.rightMetersPerSecond / kMaxSpeedMetersPerSecond);
}

  private void stop() {
    lMotor.set(0);
    rMotor.set(0);
  }

  private void resetPose() {
    lMotor.setPosition(0);
    rMotor.setPosition(0);
    m_gyroSim.setAngle(0);
    m_odometry.resetPose(new Pose2d());
    m_field.setRobotPose(
      m_odometry.getPoseMeters().getMeasureX(), 
      m_odometry.getPoseMeters().getMeasureY(), 
      m_odometry.getPoseMeters().getRotation());
  }

  private double deadband(double value){return Math.abs(value) > 0.2 ? value : 0;}

  public Command driveCommand(DoubleSupplier speed, DoubleSupplier turn) {
    return runEnd(() -> drive(speed, turn), () -> stop());
  }

  public Command resetPoseCommand() {
    return runOnce(() -> resetPose());
  }

  public void periodic() {
    m_odometry.update(
        m_gyro.getRotation2d(),
          lMotor.getPosition().getValueAsDouble() * (2*Math.PI*Constants.kWheelRadius),
          rMotor.getPosition().getValueAsDouble() * (2*Math.PI*Constants.kWheelRadius));
    m_field.setRobotPose(
      m_odometry.getPoseMeters().getMeasureX(), 
      m_odometry.getPoseMeters().getMeasureY(), 
      m_odometry.getPoseMeters().getRotation());
  }

  public void simulationInit() {
    var LtalonFXSim = lMotor.getSimState();
    LtalonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    var RtalonFXSim = rMotor.getSimState();
    RtalonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

    @Override
    public void simulationPeriodic() {
      var LtalonFXSim = lMotor.getSimState();
      LtalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      var RtalonFXSim = rMotor.getSimState();
      RtalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
      System.out.println("voltage l " + LtalonFXSim.getMotorVoltageMeasure().in(Volts) * lMotor.get() + " voltage r: " + RtalonFXSim.getMotorVoltageMeasure().in(Volts) * rMotor.get());
      // Feed TalonFX voltages directly into the drivetrain sim
      drivetrainSim.setInputs(
          LtalonFXSim.getMotorVoltageMeasure().in(Volts) * lMotor.get(),
          RtalonFXSim.getMotorVoltageMeasure().in(Volts) * rMotor.get());
      drivetrainSim.update(0.020);

      // Write drivetrain sim results back to TalonFX sim state
      // Divide by gear ratio: drivetrainSim gives wheel-shaft positions,
      // but TalonFX reports rotor rotations
      LtalonFXSim.setRawRotorPosition(
          drivetrainSim.getLeftPositionMeters() / (2 * Math.PI * Constants.kWheelRadius)
          * Constants.kWheelGearRatio);
      LtalonFXSim.setRotorVelocity(
          drivetrainSim.getLeftVelocityMetersPerSecond() / (2 * Math.PI * Constants.kWheelRadius)
          * Constants.kWheelGearRatio);

      RtalonFXSim.setRawRotorPosition(
          drivetrainSim.getRightPositionMeters() / (2 * Math.PI * Constants.kWheelRadius)
          * Constants.kWheelGearRatio);
      RtalonFXSim.setRotorVelocity(
          drivetrainSim.getRightVelocityMetersPerSecond() / (2 * Math.PI * Constants.kWheelRadius)
          * Constants.kWheelGearRatio);

      m_gyroSim.setAngle(-drivetrainSim.getHeading().getDegrees());

  // @Override
  // public void simulationPeriodic() {
  //   super.simulationPeriodic();

  //   var LtalonFXSim = lMotor.getSimState();
  //   LtalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
  //   var LmotorVoltage = LtalonFXSim.getMotorVoltageMeasure();
  //   lMotorSim.setInputVoltage(LmotorVoltage.in(Volts)); // assume 20 ms loop time
  //   LtalonFXSim.setRawRotorPosition(lMotorSim.getAngularPosition());
  //   LtalonFXSim.setRotorVelocity(lMotorSim.getAngularVelocity());
  //   lMotorSim.update(0.020);

  //   var RtalonFXSim = rMotor.getSimState();
  //   RtalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
  //   var RmotorVoltage = RtalonFXSim.getMotorVoltageMeasure();
  //   rMotorSim.setInputVoltage(RmotorVoltage.in(Volts));
  //   RtalonFXSim.setRawRotorPosition(rMotorSim.getAngularPosition());
  //   RtalonFXSim.setRotorVelocity(rMotorSim.getAngularVelocity());
  //   rMotorSim.update(0.020); // assume 20 ms loop time

  //   drivetrainSim.setInputs(
  //     lMotorSim.getInputVoltage(),
  //     rMotorSim.getInputVoltage());
  //   drivetrainSim.update(0.02);
  //   m_gyroSim.setAngle(
  //       -drivetrainSim.getHeading().getDegrees()); // heading is negative bc hover over getHeading man
    }
}