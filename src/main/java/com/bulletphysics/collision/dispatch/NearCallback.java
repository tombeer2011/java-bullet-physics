package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.BroadPhasePair;
import com.bulletphysics.collision.broadphase.DispatcherInfo;

public abstract class NearCallback {
    public abstract void handleCollision(BroadPhasePair collisionPair, CollisionDispatcher dispatcher, DispatcherInfo dispatchInfo);
}
