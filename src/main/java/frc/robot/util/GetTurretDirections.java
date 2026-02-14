package frc.robot.util;

import edu.wpi.first.math.Pair;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniverseConstants;

public class GetTurretDirections {

  Pair<Double, Double> locationAtHeight(double elevation, double azimuth, double vx, double vy) {

    // Calculate vector components
    double v0z = TurretConstants.exitVelocity * Math.sin(elevation);
    double horTotal = Math.cos(elevation);
    double horX = Math.cos(azimuth) * horTotal;
    double horY = Math.sin(azimuth) * horTotal;

    // Solving quadratic stuff
    double a = 0.5 * UniverseConstants.g;
    double b = -v0z;
    double c = FieldConstants.hubHeight - TurretConstants.launchHeight;

    double disc = b * b - 4 * a * c;
    if (disc < 0) {
      return new Pair<>(-1.0, -1.0); // No solution (Should never happen?)
    }

    double t1 = (-b + Math.sqrt(disc)) / (2 * a);
    double t2 = (-b - Math.sqrt(disc)) / (2 * a);

    double solution = Math.max(t1, t2);
    if (solution > 0) {
      return new Pair<>(
          (horX + vx) * solution, (horY + vy) * solution); // X and Y coords of where the ball lands
    }
    return new Pair<>(-1.0, -1.0);
  }

  double getAngle(double x, double y) {
    // Gets angle of point with respect to (0, 0) and a horizontal line
    return Math.toDegrees((Math.atan2(y, x) + 2 * Math.PI) % (2 * Math.PI));
  }

  double dist(Pair<Double, Double> p1, Pair<Double, Double> p2) {
    // Gets distance between 2 points
    return Math.sqrt(
        Math.pow(p1.getFirst() - p2.getFirst(), 2) + Math.pow(p1.getSecond() - p2.getSecond(), 2));
  }

  Pair<Double, Double> shootingDirections(
      Pair<Double, Double> botLoc, Pair<Double, Double> botVel, Pair<Double, Double> goal) {
    // Takes in the bot location and velocity vectors, and returns optimal elevation and azimuth
    // angles
    // If both are -1, no good solution exists

    // Starting constraints to guarantee finding a good shot.
    double elevation = Math.toRadians(51);
    double azimuth = Math.toRadians(45);

    for (int i = 0; i < 150; i++) {
      // Determine where the ball should land with given elevation and azimuth
      Pair<Double, Double> ballLoc =
          locationAtHeight(elevation, azimuth, botVel.getFirst(), botVel.getSecond());
      ballLoc =
          new Pair<>(
              ballLoc.getFirst() + botLoc.getFirst(), ballLoc.getSecond() + botLoc.getSecond());

      // See how far that is from the goal
      double ballAngle =
          getAngle(
              ballLoc.getFirst() - botLoc.getFirst(), ballLoc.getSecond() - botLoc.getSecond());
      double hubAngle =
          getAngle(goal.getFirst() - botLoc.getFirst(), goal.getSecond() - botLoc.getSecond());

      // Adjust numbers to get closer to goal number
      double error = hubAngle - ballAngle;
      error = (error + 540) % 360 - 180; // wrap to [-180, 180]
      azimuth += Math.toRadians(error * 0.1);

      double dHub = dist(goal, botLoc);
      double dBall = dist(ballLoc, botLoc);

      if (dHub < dBall) {
        elevation = Math.min(elevation - 0.01, Math.toRadians(51));
      } else {
        elevation = Math.min(elevation + 0.01, Math.toRadians(85));
      }

      // Repeat 150 times
    }

    // Check the final distance from goal and determine if within tolerance (50 cm).
    // If not, no solution at this location with this speed
    Pair<Double, Double> ballLoc =
        locationAtHeight(elevation, azimuth, botVel.getFirst(), botVel.getSecond());
    ballLoc =
        new Pair<>(
            ballLoc.getFirst() + botLoc.getFirst(), ballLoc.getSecond() + botLoc.getSecond());
    if (dist(goal, ballLoc) < TurretConstants.aimTolerance) {
      return new Pair<Double, Double>(elevation, azimuth);
    }
    return new Pair<Double, Double>(-1.0, -1.0);
  }
}
