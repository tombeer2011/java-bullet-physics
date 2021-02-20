package zynaps.engine;

public final class DepthBuffer {
    public final int width;
    public final int height;
    public final float[] data;

    public DepthBuffer(int width, int height) {
        this(width, height, new float[width * height]);
    }

    public DepthBuffer(int width, int height, float[] data) {
        this.width = width;
        this.height = height;
        this.data = data;
    }
}
