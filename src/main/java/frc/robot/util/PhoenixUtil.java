// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /**
   * Creates a TalonFX configuration with Phoenix Pro MotionMagic parameters.
   *
   * @param motionMagicKV MotionMagic velocity feedforward (default: 0.12)
   * @param motionMagicKA MotionMagic acceleration feedforward (default: 0.1)
   * @return Configured TalonFXConfiguration
   */
  public static TalonFXConfiguration createPhoenixProConfig(
      double motionMagicKV, double motionMagicKA) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotionMagic.MotionMagicExpo_kV = motionMagicKV;
    config.MotionMagic.MotionMagicExpo_kA = motionMagicKA;
    return config;
  }

  /**
   * Creates a TalonFX configuration with default Phoenix Pro MotionMagic parameters.
   *
   * @return Configured TalonFXConfiguration with kV=0.12 and kA=0.1
   */
  public static TalonFXConfiguration createPhoenixProConfig() {
    return createPhoenixProConfig(0.12, 0.1);
  }
}
