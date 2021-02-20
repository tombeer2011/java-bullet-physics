package zynaps.engine;

import javax.vecmath.Matrix4;

public abstract class Geometry implements Renderable {
    public abstract void localBoundsToWorld(Box result, Matrix4 transform);
}
