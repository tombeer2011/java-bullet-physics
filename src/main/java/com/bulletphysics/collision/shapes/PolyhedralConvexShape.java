package com.bulletphysics.collision.shapes;

import com.bulletphysics.AabbUtil2;
import com.bulletphysics.Transform;

import javax.vecmath.Vector3;

public abstract class PolyhedralConvexShape extends ConvexInternalShape {
    private static final Vector3[] _directions = new Vector3[]{new Vector3(1f, 0f, 0f), new Vector3(0f, 1f, 0f), new Vector3(0f, 0f, 1f), new Vector3(-1f, 0f, 0f), new Vector3(0f, -1f, 0f), new Vector3(0f, 0f, -1f)};
    private static final Vector3[] _supporting = new Vector3[]{new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 0f)};
    protected final Vector3 localAabbMin = new Vector3(1f, 1f, 1f);
    protected final Vector3 localAabbMax = new Vector3(-1f, -1f, -1f);
    protected boolean isLocalAabbValid = false;

    @Override
    public Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec0, Vector3 out) {
        int i;
        out.set(0f, 0f, 0f);
        var maxDot = -1e30f;
        var vec = new Vector3(vec0);
        var lenSqr = vec.lengthSquared();
        if (lenSqr < 0.0001f) {
            vec.set(1f, 0f, 0f);
        } else {
            var rlen = 1f / (float) Math.sqrt(lenSqr);
            vec.mul(rlen);
        }
        var vtx = new Vector3();
        float newDot;
        for (i = 0; i < getNumVertices(); i++) {
            getVertex(i, vtx);
            newDot = vec.dot(vtx);
            if (newDot > maxDot) {
                maxDot = newDot;
            }
        }
        return out;
    }

    @Override
    public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut, int numVectors) {
        int i;
        var vtx = new Vector3();
        float newDot;
        var wcoords = new float[numVectors];
        for (i = 0; i < numVectors; i++) {
            wcoords[i] = -1e30f;
        }
        for (var j = 0; j < numVectors; j++) {
            var vec = vectors[j];
            for (i = 0; i < getNumVertices(); i++) {
                getVertex(i, vtx);
                newDot = vec.dot(vtx);
                if (newDot > wcoords[j]) {
                    supportVerticesOut[j].set(vtx);
                    wcoords[j] = newDot;
                }
            }
        }
    }

    @Override
    public void calculateLocalInertia(float mass, Vector3 inertia) {
        var margin = getMargin();
        var ident = new Transform();
        ident.identity();
        Vector3 aabbMin = new Vector3(), aabbMax = new Vector3();
        getAabb(ident, aabbMin, aabbMax);
        var halfExtents = new Vector3();
        halfExtents.set(aabbMax).sub(aabbMin);
        halfExtents.mul(0.5f);
        var lx = 2f * (halfExtents.x + margin);
        var ly = 2f * (halfExtents.y + margin);
        var lz = 2f * (halfExtents.z + margin);
        var x2 = lx * lx;
        var y2 = ly * ly;
        var z2 = lz * lz;
        var scaledmass = mass * 0.08333333f;
        inertia.set(y2 + z2, x2 + z2, x2 + y2);
        inertia.mul(scaledmass);
    }

    private void getNonvirtualAabb(Transform trans, Vector3 aabbMin, Vector3 aabbMax, float margin) {

        AabbUtil2.transformAabb(localAabbMin, localAabbMax, margin, trans, aabbMin, aabbMax);
    }

    @Override
    public void getAabb(Transform trans, Vector3 aabbMin, Vector3 aabbMax) {
        getNonvirtualAabb(trans, aabbMin, aabbMax, getMargin());
    }

    public void recalcLocalAabb() {
        isLocalAabbValid = true;
        batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);
        for (var i = 0; i < 3; i++) {
            localAabbMax.setCoord(i, _supporting[i].getCoord(i) + collisionMargin);
            localAabbMin.setCoord(i, _supporting[i + 3].getCoord(i) - collisionMargin);
        }
    }

    @Override
    public void setLocalScaling(Vector3 scaling) {
        super.setLocalScaling(scaling);
        recalcLocalAabb();
    }

    public abstract int getNumVertices();

    public abstract void getVertex(int i, Vector3 vtx);
}
