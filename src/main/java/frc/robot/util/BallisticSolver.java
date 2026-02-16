package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;

public class BallisticSolver {

    public static class ShotSolution {
        public final double azimuthDeg;
        public final double elevationDeg;

        public ShotSolution(double azimuthDeg, double elevationDeg) {
            this.azimuthDeg = azimuthDeg;
            this.elevationDeg = elevationDeg;
        }
    }

    // Solve F(t) = 0 using Newton's method, biased toward the high-arc root
    private static Double solveTimeNewton(
            double dx, double dy, double dz,
            double vrx, double vry,
            double v0, double t0
    ) {
        double t = t0;

        for (int i = 0; i < 40; i++) {

            double Ft = F(t, dx, dy, dz, vrx, vry, v0);
            if (Math.abs(Ft) < 1e-6) return t;

            double dFt = dFdt(t, dx, dy, dz, vrx, vry, v0);
            if (Math.abs(dFt) < 1e-9) return null;

            t = t - Ft / dFt;
            if (t <= 0) return null;
        }
        return null;
    }

    private static double F(double t, double dx, double dy, double dz,
                            double vrx, double vry, 
                            double v0) {

        if (t <= 0) return 1e9;

        double vzTerm = (dz + 0.5 * Constants.UniverseConstants.g * t * t) / (v0 * t);

        double hx = dx - vrx * t;
        double hy = dy - vry * t;
        double R2 = hx * hx + hy * hy;

        return v0 * v0 * (1 - vzTerm * vzTerm) - R2 / (t * t);
    }

    private static double dFdt(double t, double dx, double dy, double dz,
                               double vrx, double vry,
                               double v0) {

        double h = 1e-5;
        return (F(t + h, dx, dy, dz, vrx, vry, v0)
              - F(t - h, dx, dy, dz, vrx, vry, v0)) / (2 * h);
    }

    // Main function you call from robot code
    public static ShotSolution solveShot(
            Pose2d robotPose, Pose3d goalPose,
            double robotVx, double robotVy, 
            double launchSpeed
    ) {
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        double dz = goalPose.getZ() - Constants.TurretConstants.launchHeight;

        // Bias toward the high-arc root
        Double t = solveTimeNewton(dx, dy, dz, robotVx, robotVy,
                                   Constants.TurretConstants.launchSpeed, 2.0);

        if (t == null) return null;

        // Compute azimuth
        double phi = Math.atan2(dy - robotVy * t, dx - robotVx * t);

        // Compute elevation
        double sinTheta = (dz + 0.5 * Constants.UniverseConstants.g * t * t) / (launchSpeed * t);
        if (sinTheta > 1 || sinTheta < -1) return null;

        double theta = Math.asin(sinTheta);

        return new ShotSolution(
                Math.toDegrees(phi),
                Math.toDegrees(theta)
        );
    }
}
