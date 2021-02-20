package com.bulletphysics.collision.broadphase;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.ManifoldResult;
import com.bulletphysics.collision.narrowphase.PersistentManifold;

import java.util.List;

public abstract class CollisionAlgorithm {
    protected Dispatcher dispatcher;

    public void init(CollisionAlgorithmConstructionInfo ci) {
        dispatcher = ci.dispatcher1;
    }

    public abstract void destroy();

    public abstract void processCollision(CollisionObject body0, CollisionObject body1, ManifoldResult resultOut);

    public abstract void getAllContactManifolds(List<PersistentManifold> manifoldArray);

}
