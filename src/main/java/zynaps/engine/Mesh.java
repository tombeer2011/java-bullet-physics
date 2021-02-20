package zynaps.engine;

public final class Mesh {
    public final int[] indexBuffer;
    public final float[] vertexBuffer;

    public Mesh(int[] indexBuffer, float[] vertexBuffer) {
        this.indexBuffer = indexBuffer;
        this.vertexBuffer = vertexBuffer;
    }
}
