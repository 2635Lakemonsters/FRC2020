package frc.model;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
