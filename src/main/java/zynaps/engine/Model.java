package zynaps.engine;

import javax.vecmath.Matrix4;
import java.util.HashMap;

public final class Model extends Geometry {
    private final HashMap<Material, Mesh> meshGroups;
    private final Box localBounds;

    public Model(HashMap<Material, Mesh> meshGroups, Box localBounds) {
        this.meshGroups = meshGroups;
        this.localBounds = localBounds;
    }

    public void localBoundsToWorld(Box result, Matrix4 transform) {
        localBounds.aggregateInto(result, transform);
    }

    public void render(Renderer renderer) {
        for (var item : meshGroups.entrySet()) {
            renderer.draw(item.getValue().vertexBuffer, item.getValue().indexBuffer, item.getKey());
        }
    }
}
