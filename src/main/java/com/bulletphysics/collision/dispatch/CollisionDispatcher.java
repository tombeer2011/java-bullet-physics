package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.*;
import com.bulletphysics.collision.narrowphase.GjkEpaPenetrationDepthSolver;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.narrowphase.VoronoiSimplexSolver;

import java.util.ArrayList;
import java.util.Collections;

public class CollisionDispatcher extends Dispatcher {
    private final ArrayList<PersistentManifold> manifoldsPtr = new ArrayList<>();
    private boolean staticWarningReported = false;
    private NearCallback nearCallback;
    private final CollisionAlgorithmConstructionInfo tmpCI = new CollisionAlgorithmConstructionInfo();
    private final VoronoiSimplexSolver simplexSolver = new VoronoiSimplexSolver();
    private final GjkEpaPenetrationDepthSolver pdSolver = new GjkEpaPenetrationDepthSolver();

    public CollisionDispatcher() {
        setNearCallback(new DefaultNearCallback());
    }

    public NearCallback getNearCallback() {
        return nearCallback;
    }

    public void setNearCallback(NearCallback nearCallback) {
        this.nearCallback = nearCallback;
    }

    @Override
    public CollisionAlgorithm findAlgorithm(CollisionObject body0, CollisionObject body1, PersistentManifold sharedManifold) {
        tmpCI.dispatcher1 = this;
        tmpCI.manifold = sharedManifold;
        return new ConvexConvexAlgorithm(tmpCI.manifold, tmpCI, simplexSolver, pdSolver);
    }

    @Override
    public void freeCollisionAlgorithm(CollisionAlgorithm algo) {
        algo.destroy();
    }

    @Override
    public PersistentManifold getNewManifold(Object b0, Object b1) {
        var body0 = (CollisionObject) b0;
        var body1 = (CollisionObject) b1;
        var manifold = new PersistentManifold();
        manifold.init(body0, body1);
        manifold.index1a = manifoldsPtr.size();
        manifoldsPtr.add(manifold);
        return manifold;
    }

    @Override
    public void releaseManifold(PersistentManifold manifold) {
        clearManifold(manifold);
        var findIndex = manifold.index1a;

        Collections.swap(manifoldsPtr, findIndex, manifoldsPtr.size() - 1);
        manifoldsPtr.get(findIndex).index1a = findIndex;
        manifoldsPtr.remove(manifoldsPtr.size() - 1);
    }

    @Override
    public void clearManifold(PersistentManifold manifold) {
        manifold.clearManifold();
    }

    @Override
    public boolean needsCollision(CollisionObject body0, CollisionObject body1) {


        var needsCollision = true;
        if (!staticWarningReported) {
            if ((body0.isStaticObject() || body0.isKinematicObject()) && (body1.isStaticObject() || body1.isKinematicObject())) {
                staticWarningReported = true;
                System.err.println("warning CollisionDispatcher.needsCollision: static-static collision!");
            }
        }
        if (!body0.isActive() && !body1.isActive()) {
            needsCollision = false;
        } else if (!body0.checkCollideWith()) {
            needsCollision = false;
        }
        return needsCollision;
    }

    @Override
    public boolean needsResponse(CollisionObject body0, CollisionObject body1) {
        var hasResponse = body0.hasContactResponse() && body1.hasContactResponse();
        hasResponse = hasResponse && (!body0.isStaticOrKinematicObject() || !body1.isStaticOrKinematicObject());
        return hasResponse;
    }

    private static class CollisionPairCallback extends OverlapCallback {
        private DispatcherInfo dispatchInfo;
        private CollisionDispatcher dispatcher;

        public void init(DispatcherInfo dispatchInfo, CollisionDispatcher dispatcher) {
            this.dispatchInfo = dispatchInfo;
            this.dispatcher = dispatcher;
        }

        public boolean processOverlap(BroadPhasePair pair) {
            dispatcher.getNearCallback().handleCollision(pair, dispatcher, dispatchInfo);
            return false;
        }
    }

    private final CollisionPairCallback collisionPairCallback = new CollisionPairCallback();

    @Override
    public void dispatchAllCollisionPairs(OverlappingPairCache pairCache, DispatcherInfo dispatchInfo, Dispatcher dispatcher) {
        collisionPairCallback.init(dispatchInfo, this);
        pairCache.processAllOverlappingPairs(collisionPairCallback, dispatcher);
    }

    @Override
    public int getNumManifolds() {
        return manifoldsPtr.size();
    }

    @Override
    public PersistentManifold getManifoldByIndexInternal(int index) {
        return manifoldsPtr.get(index);
    }
}
