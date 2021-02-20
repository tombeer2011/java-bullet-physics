package com.bulletphysics.collision.dispatch;

import com.bulletphysics.AabbUtil2;
import com.bulletphysics.ArrayList;
import com.bulletphysics.BulletGlobals;
import com.bulletphysics.Transform;
import com.bulletphysics.collision.broadphase.*;
import com.bulletphysics.collision.narrowphase.ConvexCast;
import com.bulletphysics.collision.narrowphase.ConvexCast.CastResult;
import com.bulletphysics.collision.narrowphase.GjkConvexCast;
import com.bulletphysics.collision.narrowphase.VoronoiSimplexSolver;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.ConvexShape;

import javax.vecmath.Quaternion;
import javax.vecmath.Vector3;

public class CollisionWorld {
    protected final ArrayList<CollisionObject> collisionObjects = new ArrayList<>();
    protected final Dispatcher dispatcher1;
    protected final DispatcherInfo dispatchInfo = new DispatcherInfo();
    protected BroadPhaseInterface broadphasePairCache;

    public CollisionWorld(Dispatcher dispatcher, BroadPhaseInterface broadphasePairCache) {
        dispatcher1 = dispatcher;
        this.broadphasePairCache = broadphasePairCache;
    }

    public void addCollisionObject(CollisionObject collisionObject, short collisionFilterGroup, short collisionFilterMask) {

        collisionObjects.add(collisionObject);
        var trans = collisionObject.getWorldTransform(new Transform());
        var minAabb = new Vector3();
        var maxAabb = new Vector3();
        collisionObject.getCollisionShape().getAabb(trans, minAabb, maxAabb);
        collisionObject.setBroadphaseHandle(getBroadphase().createProxy(minAabb, maxAabb, collisionObject, collisionFilterGroup, collisionFilterMask));
    }

    public void performDiscreteCollisionDetection() {
        updateAabbs();
        broadphasePairCache.calculateOverlappingPairs(dispatcher1);
        var dispatcher = getDispatcher();
        if (dispatcher != null) {
            dispatcher.dispatchAllCollisionPairs(broadphasePairCache.getOverlappingPairCache(), dispatchInfo, dispatcher1);
        }
    }

    public void setBroadphase(BroadPhaseInterface pairCache) {
        broadphasePairCache = pairCache;
    }

    public BroadPhaseInterface getBroadphase() {
        return broadphasePairCache;
    }

    public OverlappingPairCache getPairCache() {
        return broadphasePairCache.getOverlappingPairCache();
    }

    public Dispatcher getDispatcher() {
        return dispatcher1;
    }

    public DispatcherInfo getDispatchInfo() {
        return dispatchInfo;
    }

    public void updateSingleAabb(CollisionObject colObj) {
        Vector3 minAabb = new Vector3(), maxAabb = new Vector3();
        var tmp = new Vector3();
        var tmpTrans = new Transform();
        colObj.getCollisionShape().getAabb(colObj.getWorldTransform(tmpTrans), minAabb, maxAabb);
        var contactThreshold = new Vector3();
        contactThreshold.set(BulletGlobals.CONTACT_BREAKING_THRESHOLD, BulletGlobals.CONTACT_BREAKING_THRESHOLD, BulletGlobals.CONTACT_BREAKING_THRESHOLD);
        minAabb.sub(contactThreshold);
        maxAabb.add(contactThreshold);
        var bp = broadphasePairCache;
        tmp.set(maxAabb).sub(minAabb);
        if (colObj.isStaticObject() || tmp.lengthSquared() < 1e12f) {
            bp.setAabb(colObj.getBroadphaseHandle(), minAabb, maxAabb);
        } else {
            colObj.setActivationState(CollisionObject.DISABLE_SIMULATION);
        }
    }

    public void updateAabbs() {
        for (CollisionObject colObj : collisionObjects) {
            if (colObj.isActive()) {
                updateSingleAabb(colObj);
            }
        }
    }

    public int getNumCollisionObjects() {
        return collisionObjects.size();
    }

    public static void objectQuerySingle(ConvexShape castShape, Transform convexFromTrans, Transform convexToTrans, CollisionObject collisionObject, CollisionShape collisionShape, Transform colObjWorldTransform, ConvexResultCallback resultCallback, float allowedPenetration) {
        if (collisionShape.isConvex()) {
            var castResult = new CastResult();
            castResult.allowedPenetration = allowedPenetration;
            castResult.fraction = 1f;
            var convexShape = (ConvexShape) collisionShape;
            var simplexSolver = new VoronoiSimplexSolver();
            if (((ConvexCast) new GjkConvexCast(castShape, convexShape, simplexSolver)).calcTimeOfImpact(convexFromTrans, convexToTrans, colObjWorldTransform, colObjWorldTransform, castResult)) {
                if (castResult.normal.lengthSquared() > 0.0001f) {
                    if (castResult.fraction < resultCallback.closestHitFraction) {
                        castResult.normal.normalize();
                        var localConvexResult = new LocalConvexResult(collisionObject, null, castResult.normal, castResult.hitPoint, castResult.fraction);
                        resultCallback.addSingleResult(localConvexResult, true);
                    }
                }
            }
        }
    }

    public void convexSweepTest(ConvexShape castShape, Transform convexFromWorld, Transform convexToWorld, ConvexResultCallback resultCallback) {
        var convexFromTrans = new Transform();
        var convexToTrans = new Transform();
        convexFromTrans.set(convexFromWorld);
        convexToTrans.set(convexToWorld);
        var castShapeAabbMin = new Vector3();
        var castShapeAabbMax = new Vector3();
        var linVel = new Vector3();
        var angVel = new Vector3();
        Transform.calculateVelocity(convexFromTrans, convexToTrans, 1f, linVel, angVel);
        var R = new Transform();
        R.identity();
        R.setRotation(convexFromTrans.getRotation(new Quaternion()));
        castShape.calculateTemporalAabb(R, linVel, angVel, 1f, castShapeAabbMin, castShapeAabbMax);
        var tmpTrans = new Transform();
        var collisionObjectAabbMin = new Vector3();
        var collisionObjectAabbMax = new Vector3();
        var hitLambda = new float[1];
        for (CollisionObject collisionObject : collisionObjects) {
            if (resultCallback.needsCollision(collisionObject.getBroadphaseHandle())) {
                collisionObject.getWorldTransform(tmpTrans);
                collisionObject.getCollisionShape().getAabb(tmpTrans, collisionObjectAabbMin, collisionObjectAabbMax);
                collisionObjectAabbMin.add(castShapeAabbMin);
                collisionObjectAabbMax.add(castShapeAabbMax);
                hitLambda[0] = 1f;
                var hitNormal = new Vector3();
                if (AabbUtil2.rayAabb(convexFromWorld.origin, convexToWorld.origin, collisionObjectAabbMin, collisionObjectAabbMax, hitLambda, hitNormal)) {
                    objectQuerySingle(castShape, convexFromTrans, convexToTrans, collisionObject, collisionObject.getCollisionShape(), tmpTrans, resultCallback, getDispatchInfo().allowedCcdPenetration);
                }
            }
        }
    }

    public ArrayList<CollisionObject> getCollisionObjectArray() {
        return collisionObjects;
    }

    public static class LocalShapeInfo {
    }

    public static class LocalConvexResult {
        public final CollisionObject hitCollisionObject;
        public final LocalShapeInfo localShapeInfo;
        public final Vector3 hitNormalLocal = new Vector3();
        public final Vector3 hitPointLocal = new Vector3();
        public final float hitFraction;

        public LocalConvexResult(CollisionObject hitCollisionObject, LocalShapeInfo localShapeInfo, Vector3 hitNormalLocal, Vector3 hitPointLocal, float hitFraction) {
            this.hitCollisionObject = hitCollisionObject;
            this.localShapeInfo = localShapeInfo;
            this.hitNormalLocal.set(hitNormalLocal);
            this.hitPointLocal.set(hitPointLocal);
            this.hitFraction = hitFraction;
        }
    }

    public static abstract class ConvexResultCallback {
        public float closestHitFraction = 1f;
        public short collisionFilterGroup = CollisionFilterGroups.DEFAULT_FILTER;
        public short collisionFilterMask = CollisionFilterGroups.ALL_FILTER;

        public boolean hasHit() {
            return closestHitFraction < 1f;
        }

        public boolean needsCollision(BroadPhaseProxy proxy0) {
            var collides = (proxy0.collisionFilterGroup & collisionFilterMask & 0xFFFF) != 0;
            collides = collides && (collisionFilterGroup & proxy0.collisionFilterMask & 0xFFFF) != 0;
            return collides;
        }

        public abstract float addSingleResult(LocalConvexResult convexResult, boolean normalInWorldSpace);
    }

    public static class ClosestConvexResultCallback extends ConvexResultCallback {
        public final Vector3 convexFromWorld = new Vector3();
        public final Vector3 convexToWorld = new Vector3();
        public final Vector3 hitNormalWorld = new Vector3();
        public final Vector3 hitPointWorld = new Vector3();
        public CollisionObject hitCollisionObject;

        public ClosestConvexResultCallback(Vector3 convexFromWorld, Vector3 convexToWorld) {
            this.convexFromWorld.set(convexFromWorld);
            this.convexToWorld.set(convexToWorld);
            hitCollisionObject = null;
        }

        @Override
        public float addSingleResult(LocalConvexResult convexResult, boolean normalInWorldSpace) {

            closestHitFraction = convexResult.hitFraction;
            hitCollisionObject = convexResult.hitCollisionObject;
            if (normalInWorldSpace) {
                hitNormalWorld.set(convexResult.hitNormalLocal);
                if (hitNormalWorld.length() > 2) {
                    System.out.println("CollisionWorld.addSingleResult world " + hitNormalWorld);
                }
            } else {
                hitNormalWorld.set(convexResult.hitNormalLocal);
                hitCollisionObject.getWorldTransform(new Transform());
                hitNormalWorld.transform(hitCollisionObject.getWorldTransform(new Transform()).basis);
                if (hitNormalWorld.length() > 2) {
                    System.out.println("CollisionWorld.addSingleResult world " + hitNormalWorld);
                }
            }
            hitPointWorld.set(convexResult.hitPointLocal);
            return convexResult.hitFraction;
        }
    }
}
