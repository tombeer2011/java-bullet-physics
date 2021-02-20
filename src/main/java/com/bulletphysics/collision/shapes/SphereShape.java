package com.bulletphysics.collision.shapes;

import com.bulletphysics.Transform;

import javax.vecmath.Vector3;

public class SphereShape extends ConvexInternalShape {
    public SphereShape(float radius) {
        implicitShapeDimensions.x = radius;
        collisionMargin = radius;
    }

    @Override
    public Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec, Vector3 out) {
        out.set(0f, 0f, 0f);
        return out;
    }

    @Override
    public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut, int numVectors) {
        for (var i = 0; i < numVectors; i++) {
            supportVerticesOut[i].set(0f, 0f, 0f);
        }
    }

    @Override
    public void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
        var center = t.origin;
        var extent = new Vector3();
        extent.set(getMargin(), getMargin(), getMargin());
        aabbMin.set(center).sub(extent);
        aabbMax.set(center).add(extent);
    }

    @Override
    public void calculateLocalInertia(float mass, Vector3 inertia) {
        var elem = 0.4f * mass * getMargin() * getMargin();
        inertia.set(elem, elem, elem);
    }

    public float getRadius() {
        return implicitShapeDimensions.x * localScaling.x;
    }

    @Override
    public void setMargin(float margin) {
        super.setMargin(margin);
    }

    @Override
    public float getMargin() {
        return getRadius();
    }
}
