package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniverseConstants;
import frc.robot.Constants.FieldConstants;

public class Turret {
    TalonFX hood = new TalonFX(0);
    TalonFX rotation = new TalonFX(1);

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
        
        double disc = b*b - 4*a*c;
        if (disc < 0) {
            return new Pair<>(-1.0, -1.0); // No solution
        }

        double t1 = (-b + Math.sqrt(disc)) / (2*a);
        double t2 = (-b + Math.sqrt(disc)) / (2*a);

        double solution = Math.max(t1, t2);
        if (solution > 0) { 
            return new Pair<>((horX+vx) * solution, (horY + vy) * solution);
        } return new Pair<>(-1.0, -1.0);
    }
}
