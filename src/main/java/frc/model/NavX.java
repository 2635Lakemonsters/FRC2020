package frc.model;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import frc.math.Rotation2;

public final class NavX {
    private final AHRS navX;
    private Rotation2 adjustmentAngle = Rotation2.ZERO;
	private boolean inverted;


    public NavX(SPI.Port port) {
        this(port, (byte) 200);
    }

    public NavX(SPI.Port port, byte updateRate) {
        navX = new AHRS(port, updateRate);
    }


    public void reset() {
        navX.reset();
    }

    public double getAngle(){
        return navX.getAngle();
    }

	public final Rotation2 getAdjustmentAngle() {
		return adjustmentAngle;
	}

	public void setAdjustmentAngle(Rotation2 adjustmentAngle) {
		this.adjustmentAngle = adjustmentAngle;
	}

    public Rotation2 getUnadjustedAngle() {
        return Rotation2.fromRadians(getAxis(Axis.YAW));
    }

    public double getUnadjustedRate() {
        return Math.toRadians(navX.getRate());
    }

    public double getAxis(Axis axis) {
        switch (axis) {
            case PITCH:
                return Math.toRadians(navX.getPitch());
            case ROLL:
                return Math.toRadians(navX.getRoll());
            case YAW:
                return Math.toRadians(navX.getYaw());
            default:
                return 0.0;
        }
    }

    public enum Axis {
        PITCH,
        ROLL,
        YAW
    }
}
