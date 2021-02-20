package com.bulletphysics.collision.shapes;

import com.bulletphysics.Transform;

import javax.vecmath.Vector3;

public abstract class ConvexShape extends CollisionShape {
    public abstract Vector3 localGetSupportingVertex(Vector3 vec, Vector3 out);

    public abstract Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec, Vector3 out);

    public abstract void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut, int numVectors);

    public abstract void getAabbSlow(Transform t, Vector3 aabbMin, Vector3 aabbMax);

    public abstract void setLocalScaling(Vector3 scaling);

    public abstract void setMargin(float margin);

    public abstract float getMargin();
}
