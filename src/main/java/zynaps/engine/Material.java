package zynaps.engine;

import java.util.Objects;

import static java.lang.Math.max;
import static java.lang.Math.min;

public final class Material {
    public static final Material WHITE = new Material(0xFFFFFF);

    public final int color;

    public Material(int color) {
        this.color = 0xFFFFFF & color;
    }

    public Material(float r, float g, float b) {
        var red = (int) max(0, min(255, r * 255));
        var grn = (int) max(0, min(255, g * 255));
        var blu = (int) max(0, min(255, b * 255));
        color = red << 0x10 | grn << 0x08 | blu;
    }

    @Override
    public int hashCode() {
        return Objects.hash(color);
    }

    @Override
    public String toString() {
        return "Material{" + "color=" + color + '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        return color == ((Material) o).color;
    }
}
