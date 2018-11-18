package org.firstinspires.ftc.teamcode.navigation;

/**
 * Created by LeviG on 11/18/2018.
 */

public class Position {

    public long creationTime;
    public double x;
    public double y;
    public double phi;

    public Position(double x, double y, double phi, long creationTime) {
        this.x = x;
        this.y = y;
        this.phi = phi;
        this.creationTime = creationTime;
    }
}
