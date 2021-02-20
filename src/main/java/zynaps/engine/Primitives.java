package zynaps.engine;

import javax.vecmath.Vector3;

public final class Primitives {
    public static Model buildPlane(Material material) {
        var factory = new Factory();
        factory.addVertex(-1, 0, 1);
        factory.addVertex(1, 0, 1);
        factory.addVertex(1, 0, -1);
        factory.addVertex(-1, 0, -1);
        factory.createTriangle(0, 1, 2);
        factory.createTriangle(2, 3, 0);
        factory.changeColor(material);
        return factory.compile(true);
    }

    public static Model buildCube(Material material) {
        return buildCube(material, new Vector3(-1, -1, -1), new Vector3(1, 1, 1));
    }

    public static Model buildCube(Material material, Vector3 min, Vector3 max) {
        var factory = new Factory();
        factory.addVertex(min.x, max.y, min.z);
        factory.addVertex(max.x, max.y, min.z);
        factory.addVertex(max.x, min.y, min.z);
        factory.addVertex(min.x, min.y, min.z);
        factory.addVertex(max.x, max.y, max.z);
        factory.addVertex(min.x, max.y, max.z);
        factory.addVertex(min.x, min.y, max.z);
        factory.addVertex(max.x, min.y, max.z);
        factory.createTriangle(0, 1, 2);
        factory.createTriangle(2, 3, 0);
        factory.createTriangle(1, 4, 7);
        factory.createTriangle(7, 2, 1);
        factory.createTriangle(4, 5, 6);
        factory.createTriangle(6, 7, 4);
        factory.createTriangle(0, 3, 6);
        factory.createTriangle(6, 5, 0);
        factory.createTriangle(0, 5, 4);
        factory.createTriangle(4, 1, 0);
        factory.createTriangle(3, 2, 7);
        factory.createTriangle(7, 6, 3);
        factory.changeColor(material);
        return factory.compile(true);
    }
}
