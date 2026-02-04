// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final StatusSignal<Angle> pitch = pigeon.getPitch();
  private final StatusSignal<Angle> roll = pigeon.getRoll();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final Queue<Double> pitchPositionQueue;
  private final Queue<Double> rollPositionQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
      pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    pitch.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    roll.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);

    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
    pitchPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pitch.clone());
    rollPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(roll.clone());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, pitch, roll, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.yawPitchRollPosition =
        new Rotation3d(
            Units.degreesToRadians(roll.getValueAsDouble()),
            Units.degreesToRadians(pitch.getValueAsDouble()),
            Units.degreesToRadians(yaw.getValueAsDouble()));
    List<Double> rollPositions = new ArrayList<>(rollPositionQueue);
    List<Double> pitchPositions = new ArrayList<>(pitchPositionQueue);
    List<Double> yawPositions = new ArrayList<>(yawPositionQueue);
    int sampleCount =
        Math.min(rollPositions.size(), Math.min(pitchPositions.size(), yawPositions.size()));
    Rotation3d[] yawPitchRollPositions = new Rotation3d[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      yawPitchRollPositions[i] =
          new Rotation3d(
              Units.degreesToRadians(rollPositions.get(i)),
              Units.degreesToRadians(pitchPositions.get(i)),
              Units.degreesToRadians(yawPositions.get(i)));
    }
    inputs.odometryYawPitchRollPositions = yawPitchRollPositions;
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
    pitchPositionQueue.clear();
    rollPositionQueue.clear();
  }
}
