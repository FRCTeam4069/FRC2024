package frc.robot.subsystems;

public enum RevBlinkinPatterns {
    FIRE_LARGE(1215),
    FIRE_MEDIUM(1205),
    TWINKLES_OCEAN_PALETTE(1295),
    TWINKLES_LAVA_PALETTE(1305),
    OCEAN_PALLETE(1125),
    FOREST_PALLETE(1145),
    COLOR_WAVES_OCEAN_PALLETE(1295),
    COLOR_WAVES_LAVA_PALLETE(1305),
    COLOR_WAVES_FOREST_PALLETE(1315),
    SHOT_RED(1345),
    SHOT_BLUE(1355),
    SHOT_WHITE(1365),
    GOLD(1835),
    YELLOW(1845),
    GREEN(1875),
    RED(1805),
    VIOLET(1955),
    BLUE(1935),
    ORANGE(1825),
    DARK_BLUE(1925),
    STROBE_GOLD_GRAY(0.11),
    WHITE(1965),
    END_TO_END_BLEND_TO_BLACK_GRAY(0.15);

    private final double value;

    RevBlinkinPatterns(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }
}