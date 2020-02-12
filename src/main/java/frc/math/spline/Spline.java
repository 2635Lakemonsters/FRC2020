package frc.math.spline;

import frc.math.Rotation2;
import frc.math.Vector2;

public abstract class Spline {

    public abstract Vector2 getPoint(double t);

    public abstract Rotation2 getHeading(double t);
}
