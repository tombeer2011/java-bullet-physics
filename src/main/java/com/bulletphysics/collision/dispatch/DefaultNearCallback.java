package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.BroadPhasePair;
import com.bulletphysics.collision.broadphase.DispatcherInfo;

public class DefaultNearCallback extends NearCallback {
    private final ManifoldResult contactPointResult = new ManifoldResult();

    public void handleCollision(BroadPhasePair collisionPair, CollisionDispatcher dispatcher, DispatcherInfo dispatchInfo) {
        var colObj0 = (CollisionObject) collisionPair.pProxy0.clientObject;
        var colObj1 = (CollisionObject) collisionPair.pProxy1.clientObject;
        if (dispatcher.needsCollision(colObj0, colObj1)) {
            if (collisionPair.algorithm == null) {
                collisionPair.algorithm = dispatcher.findAlgorithm(colObj0, colObj1);
            }
            if (collisionPair.algorithm != null) {
                contactPointResult.init(colObj0, colObj1);
                collisionPair.algorithm.processCollision(colObj0, colObj1, contactPointResult);
            }
        }
    }
}
