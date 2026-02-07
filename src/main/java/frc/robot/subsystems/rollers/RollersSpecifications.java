package frc.robot.subsystems.rollers;

public record RollersSpecifications(
    double gearRatio, boolean inverted, int currentLimitAmps, double rollerRadiusMeters) {}
