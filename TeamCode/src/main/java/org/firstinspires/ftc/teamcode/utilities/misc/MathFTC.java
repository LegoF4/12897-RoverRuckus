package org.firstinspires.ftc.teamcode.utilities.misc;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.NavUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static java.lang.Math.acos;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

/**
 * Created by LeviG on 9/17/2017.
 */

public class MathFTC {

    public static final double cos45 = 0.70710678118654752440084436210485d;
    public static final double sin45 = 0.70710678118654752440084436210485d;

    public static double norm(List<Double> start) {
        double norm = 0;
        for (Double val : start) {
            norm += Math.abs(val*val);
        }
        return sqrt(norm);
    }

    public static double simplifyDeg(double angle) {
        return angle >= 0 ? angle % 360 : (angle % 360) + 360;
    }

    public static double simplifyRad(double angle) {
        return angle >= 0 ? angle % 2*Math.PI : (angle % 2*Math.PI) + 2*Math.PI;
    }

    public static List<Double> normalizeToMax(List<Double> start, double maxV) {
        double max = Collections.max(start);
        for (int i = 0; i < start.size(); i++) {
            start.set(i, start.get(i)*(maxV/max));
        }
        return start;
    }

    public static List<Double> scalarMultiply(List<Double> start, double scalar) {
        for (int i = 0; i < start.size(); i++) {
            start.set(i, start.get(i)*scalar);
        }
        return start;
    }

    public static boolean threshold(Position pos) {
        return (Math.abs(pos.x) > 0.1 || Math.abs(pos.y) > 0.1 || Math.abs(pos.z) > 0.1);
    }

    public static boolean threshold(Velocity pos) {
        return (Math.abs(pos.xVeloc) > 0.1 || Math.abs(pos.yVeloc) > 0.1 || Math.abs(pos.zVeloc) > 0.1);
    }
    public static boolean threshold(Acceleration pos) {
        return (Math.abs(pos.xAccel) > 0.1 || Math.abs(pos.yAccel) > 0.1 || Math.abs(pos.zAccel) > 0.1);
    }

    public static double clamp(double value, double min, double max) {
        if (value > max) value = max;
        if (value < min)  value = min;
        return value;
    }

    public static boolean inRange(double value, double min, double max) {
        if(value >= min && value <= max) return true;
        else return false;
    }

    public static double solveFourBar(double g,double f,double a, double b,double alpha,boolean flipped) {
        double l = cosLawLength(a, g, alpha);
        double beta1 = cosLawAngle(g, l, a);
        double beta2 = cosLawAngle(l, b, f);
        if (sin(alpha) > 0) {
            if (flipped)
                return Math.PI - beta1 + beta2;
            else
                return Math.PI - beta1 - beta2;
        } else {
            if (flipped) {
                return Math.PI + beta1 + beta2;
            } else {
                return Math.PI + beta1 - beta2;
            }
        }
    }

    public static double cosLawLength(double a, double b, double C) {
        return sqrt(a*a + b*b - 2 * a * b * cos(C));
    }

    public static double cosLawAngle(double a, double b, double c) {
        if (a >0 && b > 0)
            return acos((a*a + b*b - c*c) / (2 * a * b));
        else
            return 0;
    }

    public static double distance(double xT, double yT, double xF, double yF, double cos, double sin) {
        return (xF - xT)*cos + (yF - yT)*sin;
    }


}
