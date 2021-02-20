package zynaps.engine;

public final class ColorBuffer {
    public final int width;
    public final int height;
    public final int[] data;

    public ColorBuffer(int width, int height) {
        this(width, height, new int[width * height]);
    }

    public ColorBuffer(int width, int height, int[] data) {
        this.width = width;
        this.height = height;
        this.data = data;
    }
}
