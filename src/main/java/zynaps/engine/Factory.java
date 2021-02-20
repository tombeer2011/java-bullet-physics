package zynaps.engine;

import javax.vecmath.Vector3;
import java.util.ArrayList;
import java.util.HashMap;

public class Factory implements MaterialModifier {
    private final ArrayList<Vector3> vertices;
    private final ArrayList<Triangle> triangles;
    private final HashMap<String, ArrayList<Triangle>> groups;
    private ArrayList<Triangle> currentGroup;

    public Factory() {
        vertices = new ArrayList<>();
        triangles = new ArrayList<>();
        groups = new HashMap<>();
        setCurrentGroup("");
    }

    public void setCurrentGroup(String value) {
        if (!groups.containsKey(value)) {
            groups.put(value, new ArrayList<>());
        }
        currentGroup = groups.get(value);
    }

    public void addVertex(float x, float y, float z) {
        vertices.add(new Vector3(x, y, z));
    }

    public void createTriangle(int a, int b, int c) {
        var triangle = new Triangle(a, b, c);
        triangles.add(triangle);
        currentGroup.add(triangle);
    }

    public void changeColor(Material material) {
        triangles.forEach(triangle -> triangle.changeColor(material));
    }

    public Model compile(boolean optimize) {
        var localBounds = new Box();
        for (var v : vertices) {
            localBounds.aggregate(v.x, v.y, v.z);
        }
        return new Model(compileBuffers(optimize), localBounds);
    }

    private HashMap<Material, Mesh> compileBuffers(boolean optimize) {
        var matBuckets = new HashMap<Material, ArrayList<Triangle>>();
        for (var triangle : triangles) {
            if (!matBuckets.containsKey(triangle.material)) {
                matBuckets.put(triangle.material, new ArrayList<>());
            }
            matBuckets.get(triangle.material).add(triangle);
        }
        var meshByMaterial = new HashMap<Material, Mesh>();
        for (var bucket : matBuckets.entrySet()) {
            var triangles = bucket.getValue();
            var elementCount = triangles.size() * 3;
            var ib = new int[elementCount];
            var vb = new Vector3[elementCount];
            var idx = 0;
            for (var triangle : triangles) {
                vb[idx] = vertices.get(triangle.a);
                ib[idx] = idx++;
                vb[idx] = vertices.get(triangle.b);
                ib[idx] = idx++;
                vb[idx] = vertices.get(triangle.c);
                ib[idx] = idx++;
            }
            meshByMaterial.put(bucket.getKey(), optimize ? optimize(vb, ib) : createMesh(vb, ib));
        }
        return meshByMaterial;
    }

    public Mesh optimize(Vector3[] vertices, int[] triangles) {
        var vertexMap = new HashMap<Vector3, Integer>();
        var optTriangles = new int[triangles.length];
        for (var i = 0; i < triangles.length; i++) {
            var va = vertices[triangles[i]];
            if (!vertexMap.containsKey(va)) {
                vertexMap.put(va, vertexMap.size());
            }
            optTriangles[i] = vertexMap.get(va);
        }
        var optVertices = new Vector3[vertexMap.size()];
        for (var entry : vertexMap.entrySet()) {
            optVertices[entry.getValue()] = entry.getKey();
        }
        return createMesh(optVertices, optTriangles);
    }

    private Mesh createMesh(Vector3[] verts, int[] indexBuffer) {
        float[] vertexBuffer = new float[verts.length * 3];
        var index = 0;
        for (var v : verts) {
            vertexBuffer[index++] = v.x;
            vertexBuffer[index++] = v.y;
            vertexBuffer[index++] = v.z;
        }
        return new Mesh(indexBuffer, vertexBuffer);
    }
}
