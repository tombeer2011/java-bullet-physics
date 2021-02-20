package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.Transform;

import javax.vecmath.Vector3;

public interface ConvexCast {
    boolean calcTimeOfImpact(Transform fromA, Transform toA, Transform fromB, Transform toB, CastResult result);

    class CastResult {
        public final Vector3 normal = new Vector3();
        public final Vector3 hitPoint = new Vector3();
        public float fraction = 1e30f;
        public float allowedPenetration = 0f;
    }
}
