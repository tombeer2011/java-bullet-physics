package com.bulletphysics.collision.shapes;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.Transform;

import javax.vecmath.Vector3;

public abstract class ConvexInternalShape extends ConvexShape {
    protected final Vector3 localScaling = new Vector3(1f, 1f, 1f);
    protected final Vector3 implicitShapeDimensions = new Vector3();
    protected float collisionMargin = BulletGlobals.CONVEX_DISTANCE_MARGIN;

    @Override
    public void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
        getAabbSlow(t, aabbMin, aabbMax);
    }

    @Override
    public void getAabbSlow(Transform trans, Vector3 minAabb, Vector3 maxAabb) {
        var margin = getMargin();
        var vec = new Vector3();
        var tmp1 = new Vector3();
        var tmp2 = new Vector3();
        for (var i = 0; i < 3; i++) {
            vec.set(0f, 0f, 0f);
            vec.setCoord(i, 1f);
            trans.basis.transposeTransform(tmp1, vec);
            localGetSupportingVertex(tmp1, tmp2);
            trans.transform(tmp2);
            maxAabb.setCoord(i, tmp2.getCoord(i) + margin);
            vec.setCoord(i, -1f);
            trans.basis.transposeTransform(tmp1, vec);
            localGetSupportingVertex(tmp1, tmp2);
            trans.transform(tmp2);
            minAabb.setCoord(i, tmp2.getCoord(i) - margin);
        }
    }

    @Override
    public Vector3 localGetSupportingVertex(Vector3 vec, Vector3 out) {
        var supVertex = localGetSupportingVertexWithoutMargin(vec, out);
        if (getMargin() != 0f) {
            var vecnorm = new Vector3(vec);
            if (vecnorm.lengthSquared() < BulletGlobals.EPSILON * BulletGlobals.EPSILON) {
                vecnorm.set(-1f, -1f, -1f);
            }
            vecnorm.normalize();
            supVertex.scaleAdd(getMargin(), vecnorm, supVertex);
        }
        return out;
    }

    public void setLocalScaling(Vector3 scaling) {
        localScaling.set(scaling).absolute();
    }

    public float getMargin() {
        return collisionMargin;
    }

    public void setMargin(float margin) {
        collisionMargin = margin;
    }
}
