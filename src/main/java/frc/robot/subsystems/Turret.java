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
            return new Pair<>((horX+vx) * solution, (horY + vy) * solution); // X and Y coords of where the ball lands in the hub
        } return new Pair<>(-1.0, -1.0);
    }

    double getAngle(double x, double y) {
        double d = Math.sqrt(x*x+y*y);
        double st = y/d;
        double ct = x/d;

        if (ct >= 0) {
            return (Math.asin(st) + 2*Math.PI) % (2*Math.PI) * 180 / Math.PI;
        } return (3*Math.PI - Math.asin(st)) % (2*Math.PI) * 180 / Math.PI;
    }

    double dist(Pair<Double, Double> p1, Pair<Double, Double> p2) {
        return Math.sqrt(
            Math.pow(p1.getFirst()-p2.getFirst(), 2) +
            Math.pow(p1.getSecond()-p2.getSecond(), 2)
        );
    }

    Pair<Double, Double> shootingDirections(Pair<Double, Double> botLoc, Pair<Double, Double> botVel) {
        double elevation = Math.toRadians(51);
        double azimuth = Math.toRadians(45);

        for (int i = 0; i < 150; i++) {
            Pair<Double, Double> ballLoc = locationAtHeight(
                elevation, azimuth, botVel.getFirst(), botVel.getSecond()
            );
            ballLoc = new Pair<>(ballLoc.getFirst() + botLoc.getFirst(), ballLoc.getSecond() + botLoc.getSecond());
            double ballAngle = getAngle(ballLoc.getFirst()-botLoc.getFirst(), ballLoc.getSecond()-botLoc.getSecond());
            double hubAngle = getAngle(FieldConstants.hubX-botLoc.getFirst(), FieldConstants.hubY-botLoc.getSecond());

            azimuth += Math.toRadians((hubAngle-ballAngle) * 0.1);

            double dHub = dist(FieldConstants.hubLoc, botLoc);
            double dBall = dist(ballLoc, botLoc);

            if (dHub < dBall) {
                elevation = Math.min(elevation+0.01, Math.toRadians(85));
            } else {
                elevation = Math.min(elevation-0.01, Math.toRadians(51));
            }
        }
        return new Pair<Double,Double>(elevation, azimuth);
    }
}
