package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.CollisionAlgorithm;
import com.bulletphysics.collision.broadphase.CollisionAlgorithmConstructionInfo;
import com.bulletphysics.collision.narrowphase.ConvexPenetrationDepthSolver;
import com.bulletphysics.collision.narrowphase.DiscreteCollisionDetectorInterface.ClosestPointInput;
import com.bulletphysics.collision.narrowphase.GjkPairDetector;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.narrowphase.SimplexSolverInterface;
import com.bulletphysics.collision.shapes.ConvexShape;

import java.util.List;

public class ConvexConvexAlgorithm extends CollisionAlgorithm {
    private final GjkPairDetector gjkPairDetector = new GjkPairDetector();
    public boolean ownManifold;
    public PersistentManifold manifoldPtr;
    public boolean lowLevelOfDetail;

    public ConvexConvexAlgorithm(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, SimplexSolverInterface simplexSolver, ConvexPenetrationDepthSolver pdSolver) {
        init(mf, ci, simplexSolver, pdSolver);
    }

    public void init(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, SimplexSolverInterface simplexSolver, ConvexPenetrationDepthSolver pdSolver) {
        super.init(ci);
        gjkPairDetector.init(null, null, simplexSolver, pdSolver);
        manifoldPtr = mf;
        ownManifold = false;
        lowLevelOfDetail = false;
    }

    @Override
    public void destroy() {
        if (ownManifold) {
            if (manifoldPtr != null) dispatcher.releaseManifold(manifoldPtr);
            manifoldPtr = null;
        }
    }

    @Override
    public void processCollision(CollisionObject body0, CollisionObject body1, ManifoldResult resultOut) {
        if (manifoldPtr == null) {
            manifoldPtr = dispatcher.getNewManifold(body0, body1);
            ownManifold = true;
        }
        resultOut.setPersistentManifold(manifoldPtr);
        var min0 = (ConvexShape) body0.getCollisionShape();
        var min1 = (ConvexShape) body1.getCollisionShape();
        var input = new ClosestPointInput();
        gjkPairDetector.setMinkowskiA(min0);
        gjkPairDetector.setMinkowskiB(min1);
        input.maximumDistanceSquared = min0.getMargin() + min1.getMargin() + manifoldPtr.getContactBreakingThreshold();
        input.maximumDistanceSquared *= input.maximumDistanceSquared;
        body0.getWorldTransform(input.transformA);
        body1.getWorldTransform(input.transformB);
        gjkPairDetector.getClosestPoints(input, resultOut);
        if (ownManifold) resultOut.refreshContactPoints();
    }

    @Override
    public void getAllContactManifolds(List<PersistentManifold> manifoldArray) {
        if (manifoldPtr != null && ownManifold) manifoldArray.add(manifoldPtr);
    }
}
