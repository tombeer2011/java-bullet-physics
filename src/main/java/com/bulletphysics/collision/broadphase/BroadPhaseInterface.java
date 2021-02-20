package com.bulletphysics.collision.broadphase;

import javax.vecmath.Vector3;

public abstract class BroadPhaseInterface {
    public abstract BroadPhaseProxy createProxy(Vector3 aabbMin, Vector3 aabbMax, Object userPtr, short collisionFilterGroup, short collisionFilterMask);

    public abstract void setAabb(BroadPhaseProxy proxy, Vector3 aabbMin, Vector3 aabbMax);

    public abstract void calculateOverlappingPairs(Dispatcher dispatcher);

    public abstract OverlappingPairCache getOverlappingPairCache();
}
