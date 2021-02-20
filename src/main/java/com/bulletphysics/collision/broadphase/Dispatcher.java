package com.bulletphysics.collision.broadphase;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.narrowphase.PersistentManifold;

public abstract class Dispatcher {
    public final CollisionAlgorithm findAlgorithm(CollisionObject body0, CollisionObject body1) {
        return findAlgorithm(body0, body1, null);
    }

    public abstract CollisionAlgorithm findAlgorithm(CollisionObject body0, CollisionObject body1, PersistentManifold sharedManifold);

    public abstract PersistentManifold getNewManifold(Object body0, Object body1);

    public abstract void releaseManifold(PersistentManifold manifold);

    public abstract void clearManifold(PersistentManifold manifold);

    public abstract boolean needsCollision(CollisionObject body0, CollisionObject body1);

    public abstract boolean needsResponse(CollisionObject body0, CollisionObject body1);

    public abstract void dispatchAllCollisionPairs(OverlappingPairCache pairCache, DispatcherInfo dispatchInfo, Dispatcher dispatcher);

    public abstract int getNumManifolds();

    public abstract PersistentManifold getManifoldByIndexInternal(int index);

    public abstract void freeCollisionAlgorithm(CollisionAlgorithm algo);
}
