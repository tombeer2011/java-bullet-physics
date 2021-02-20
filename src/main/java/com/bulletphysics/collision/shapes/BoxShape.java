package com.bulletphysics.collision.shapes;

import com.bulletphysics.AabbUtil2;
import com.bulletphysics.Transform;

import javax.vecmath.Vector3;

public class BoxShape extends PolyhedralConvexShape {
    public BoxShape(Vector3 boxHalfExtents) {
        var margin = new Vector3(getMargin(), getMargin(), getMargin());
        implicitShapeDimensions.mul(boxHalfExtents, localScaling);
        implicitShapeDimensions.sub(margin);
    }

    public Vector3 getHalfExtentsWithMargin(Vector3 out) {
        var halfExtents = getHalfExtentsWithoutMargin(out);
        var margin = new Vector3();
        margin.set(getMargin(), getMargin(), getMargin());
        halfExtents.add(margin);
        return out;
    }

    public Vector3 getHalfExtentsWithoutMargin(Vector3 out) {
        out.set(implicitShapeDimensions);
        return out;
    }

    @Override
    public Vector3 localGetSupportingVertex(Vector3 vec, Vector3 out) {
        var halfExtents = getHalfExtentsWithoutMargin(out);
        var margin = getMargin();
        halfExtents.x += margin;
        halfExtents.y += margin;
        halfExtents.z += margin;
        out.set(fsel(vec.x, halfExtents.x, -halfExtents.x), fsel(vec.y, halfExtents.y, -halfExtents.y), fsel(vec.z, halfExtents.z, -halfExtents.z));
        return out;
    }

    @Override
    public Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec, Vector3 out) {
        var halfExtents = getHalfExtentsWithoutMargin(out);
        out.set(fsel(vec.x, halfExtents.x, -halfExtents.x), fsel(vec.y, halfExtents.y, -halfExtents.y), fsel(vec.z, halfExtents.z, -halfExtents.z));
        return out;
    }

    @Override
    public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut, int numVectors) {
        var halfExtents = getHalfExtentsWithoutMargin(new Vector3());
        for (var i = 0; i < numVectors; i++) {
            var vec = vectors[i];
            supportVerticesOut[i].set(fsel(vec.x, halfExtents.x, -halfExtents.x), fsel(vec.y, halfExtents.y, -halfExtents.y), fsel(vec.z, halfExtents.z, -halfExtents.z));
        }
    }

    @Override
    public void setMargin(float margin) {
        var oldMargin = new Vector3();
        oldMargin.set(getMargin(), getMargin(), getMargin());
        var implicitShapeDimensionsWithMargin = new Vector3();
        implicitShapeDimensionsWithMargin.set(implicitShapeDimensions).add(oldMargin);
        super.setMargin(margin);
        var newMargin = new Vector3();
        newMargin.set(getMargin(), getMargin(), getMargin());
        implicitShapeDimensions.set(implicitShapeDimensionsWithMargin).sub(newMargin);
    }

    @Override
    public void setLocalScaling(Vector3 scaling) {
        var oldMargin = new Vector3();
        oldMargin.set(getMargin(), getMargin(), getMargin());
        var implicitShapeDimensionsWithMargin = new Vector3();
        implicitShapeDimensionsWithMargin.set(implicitShapeDimensions).add(oldMargin);
        var unScaledImplicitShapeDimensionsWithMargin = new Vector3();
        unScaledImplicitShapeDimensionsWithMargin.div(implicitShapeDimensionsWithMargin, localScaling);
        super.setLocalScaling(scaling);
        implicitShapeDimensions.mul(unScaledImplicitShapeDimensionsWithMargin, localScaling);
        implicitShapeDimensions.sub(oldMargin);
    }

    @Override
    public void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
        AabbUtil2.transformAabb(getHalfExtentsWithoutMargin(new Vector3()), getMargin(), t, aabbMin, aabbMax);
    }

    @Override
    public void calculateLocalInertia(float mass, Vector3 inertia) {
        var halfExtents = getHalfExtentsWithMargin(new Vector3());
        var lx = 2f * halfExtents.x;
        var ly = 2f * halfExtents.y;
        var lz = 2f * halfExtents.z;
        inertia.set(mass / 12f * (ly * ly + lz * lz), mass / 12f * (lx * lx + lz * lz), mass / 12f * (lx * lx + ly * ly));
    }

    @Override
    public int getNumVertices() {
        return 8;
    }

    @Override
    public void getVertex(int i, Vector3 vtx) {
        var halfExtents = getHalfExtentsWithoutMargin(new Vector3());
        vtx.set(halfExtents.x * (1 - (i & 1)) - halfExtents.x * (i & 1), halfExtents.y * (1 - ((i & 2) >> 1)) - halfExtents.y * ((i & 2) >> 1), halfExtents.z * (1 - ((i & 4) >> 2)) - halfExtents.z * ((i & 4) >> 2));
    }

    public static float fsel(float a, float b, float c) {
        return a >= 0 ? b : c;
    }
}
