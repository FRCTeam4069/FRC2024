package frc.robot.subsystems;

public enum RevBlinkinPatterns {
    RAINBOW_RAINBOW_PALETTE(0.31),
    RAINBOW_PARTY_PALETTE(0.41),
    RAINBOW_OCEAN_PALETTE(0.51),
    RAINBOW_LAVA_PALETTE(0.61),
    FIRE_LARGE(0.71),
    FIRE_MEDIUM(0.73),
    FIRE_SMALL(0.75),
    TWINKLES_RAINBOW_PALETTE(0.81),
    TWINKLES_PARTY_PALETTE(0.83),
    TWINKLES_OCEAN_PALETTE(0.85),
    TWINKLES_LAVA_PALETTE(0.87),
    SHOT_RED(0.91),
    SHOT_BLUE(0.93),
    SHOT_WHITE(0.95),
    STROBE_RED_GRAY(0.51),
    STROBE_BLUE_GRAY(0.53),
    STROBE_GOLD(0.55),
    STROBE_WHITE(0.57),
    STROBE_RED_BLUE(0.07),
    STROBE_GREEN(0.29),
    STROBE_YELLOW(0.69),
    STROBE_RED(0.13),
    //STROBE_BLUE_GRAY(0.09),
    STROBE_GOLD_GRAY(0.11),
    STROBE_WHITE_GRAY(0.13),
    END_TO_END_BLEND_TO_BLACK_GRAY(0.15);

    

    private final double value;

    RevBlinkinPatterns(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }
}