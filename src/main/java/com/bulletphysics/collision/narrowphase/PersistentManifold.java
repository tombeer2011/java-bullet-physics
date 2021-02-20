package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.Transform;

import javax.vecmath.Vector3;
import java.util.stream.IntStream;

public class PersistentManifold {
    public static final int MANIFOLD_CACHE_SIZE = 4;
    private final ManifoldPoint[] pointCache = new ManifoldPoint[MANIFOLD_CACHE_SIZE];
    private Object body0;
    private Object body1;
    private int cachedPoints;
    public int index1a;

    public PersistentManifold() {
        IntStream.range(0, pointCache.length).forEach(i -> pointCache[i] = new ManifoldPoint());
    }

    public PersistentManifold(Object body0, Object body1) {
        this();
        init(body0, body1);
    }

    public void init(Object body0, Object body1) {
        this.body0 = body0;
        this.body1 = body1;
        cachedPoints = 0;
        index1a = 0;
    }

    private int sortCachedPoints(ManifoldPoint pt) {
        var maxPenetrationIndex = -1;
        var maxPenetration = pt.getDistance();
        for (var i = 0; i < 4; i++) {
            if (pointCache[i].getDistance() < maxPenetration) {
                maxPenetrationIndex = i;
                maxPenetration = pointCache[i].getDistance();
            }
        }
        var res0 = 0f;
        if (maxPenetrationIndex != 0) {
            var a0 = new Vector3(pt.localPointA).sub(pointCache[1].localPointA);
            var b0 = new Vector3(pointCache[3].localPointA).sub(pointCache[2].localPointA);
            res0 = a0.cross(a0, b0).lengthSquared();
        }
        var res1 = 0f;
        if (maxPenetrationIndex != 1) {
            var a1 = new Vector3(pt.localPointA).sub(pointCache[0].localPointA);
            var b1 = new Vector3(pointCache[3].localPointA).sub(pointCache[2].localPointA);
            res1 = a1.cross(a1, b1).lengthSquared();
        }
        var res2 = 0f;
        if (maxPenetrationIndex != 2) {
            var a2 = new Vector3(pt.localPointA).sub(pointCache[0].localPointA);
            var b2 = new Vector3(pointCache[3].localPointA).sub(pointCache[1].localPointA);
            res2 = a2.cross(a2, b2).lengthSquared();
        }
        var res3 = 0f;
        if (maxPenetrationIndex != 3) {
            var a3 = new Vector3(pt.localPointA).sub(pointCache[0].localPointA);
            var b3 = new Vector3(pointCache[2].localPointA).sub(pointCache[1].localPointA);
            res3 = a3.cross(a3, b3).lengthSquared();
        }
        var maxVal = -1.0E30F;
        var maxIndex = -1;
        var absX = Math.abs(res0);
        if (absX > maxVal) {
            maxIndex = 0;
            maxVal = absX;
        }
        var absY = Math.abs(res1);
        if (absY > maxVal) {
            maxIndex = 1;
            maxVal = absY;
        }
        var absZ = Math.abs(res2);
        if (absZ > maxVal) {
            maxIndex = 2;
            maxVal = absZ;
        }
        if (Math.abs(res3) > maxVal) {
            maxIndex = 3;
        }
        return maxIndex;
    }

    public Object getBody0() {
        return body0;
    }

    public Object getBody1() {
        return body1;
    }

    public int getNumContacts() {
        return cachedPoints;
    }

    public ManifoldPoint getContactPoint(int index) {
        return pointCache[index];
    }

    public float getContactBreakingThreshold() {
        return BulletGlobals.CONTACT_BREAKING_THRESHOLD;
    }

    public int getCacheEntry(ManifoldPoint newPoint) {
        var shortestDist = getContactBreakingThreshold() * getContactBreakingThreshold();
        var size = getNumContacts();
        var nearestPoint = -1;
        var diffA = new Vector3();
        for (var i = 0; i < size; i++) {
            var mp = pointCache[i];
            diffA.set(mp.localPointA).sub(newPoint.localPointA);
            var distToManiPoint = diffA.dot(diffA);
            if (distToManiPoint < shortestDist) {
                shortestDist = distToManiPoint;
                nearestPoint = i;
            }
        }
        return nearestPoint;
    }

    public int addManifoldPoint(ManifoldPoint newPoint) {
        var insertIndex = getNumContacts();
        if (insertIndex == MANIFOLD_CACHE_SIZE) {
            if (MANIFOLD_CACHE_SIZE >= 4) insertIndex = sortCachedPoints(newPoint);
            else insertIndex = 0;
        } else cachedPoints++;

        pointCache[insertIndex].set(newPoint);
        return insertIndex;
    }

    public void removeContactPoint(int index) {
        var lastUsedIndex = getNumContacts() - 1;
        if (index != lastUsedIndex) {
            pointCache[index].set(pointCache[lastUsedIndex]);
            pointCache[lastUsedIndex].userPersistentData = null;
            pointCache[lastUsedIndex].appliedImpulse = 0f;
            pointCache[lastUsedIndex].lateralFrictionInitialized = false;
            pointCache[lastUsedIndex].appliedImpulseLateral1 = 0f;
            pointCache[lastUsedIndex].appliedImpulseLateral2 = 0f;
            pointCache[lastUsedIndex].lifeTime = 0;
        }

        cachedPoints--;
    }

    public void replaceContactPoint(ManifoldPoint newPoint, int insertIndex) {

        var lifeTime = pointCache[insertIndex].getLifeTime();
        var appliedImpulse = pointCache[insertIndex].appliedImpulse;
        var appliedLateralImpulse1 = pointCache[insertIndex].appliedImpulseLateral1;
        var appliedLateralImpulse2 = pointCache[insertIndex].appliedImpulseLateral2;

        var cache = pointCache[insertIndex].userPersistentData;
        pointCache[insertIndex].set(newPoint);
        pointCache[insertIndex].userPersistentData = cache;
        pointCache[insertIndex].appliedImpulse = appliedImpulse;
        pointCache[insertIndex].appliedImpulseLateral1 = appliedLateralImpulse1;
        pointCache[insertIndex].appliedImpulseLateral2 = appliedLateralImpulse2;
        pointCache[insertIndex].lifeTime = lifeTime;
    }

    private boolean validContactDistance(ManifoldPoint pt) {
        return pt.distance1 <= getContactBreakingThreshold();
    }

    public void refreshContactPoints(Transform trA, Transform trB) {
        var tmp = new Vector3();
        for (int i = getNumContacts() - 1; i >= 0; i--) {
            var manifoldPoint = pointCache[i];
            manifoldPoint.positionWorldOnA.set(manifoldPoint.localPointA);
            trA.transform(manifoldPoint.positionWorldOnA);
            manifoldPoint.positionWorldOnB.set(manifoldPoint.localPointB);
            trB.transform(manifoldPoint.positionWorldOnB);
            tmp.set(manifoldPoint.positionWorldOnA).sub(manifoldPoint.positionWorldOnB);
            manifoldPoint.distance1 = tmp.dot(manifoldPoint.normalWorldOnB);
            manifoldPoint.lifeTime++;
        }
        var projectedDifference = new Vector3();
        var projectedPoint = new Vector3();
        for (int i = getNumContacts() - 1; i >= 0; i--) {
            var manifoldPoint = pointCache[i];
            if (!validContactDistance(manifoldPoint)) removeContactPoint(i);
            else {
                tmp.set(manifoldPoint.normalWorldOnB).mul(manifoldPoint.distance1);
                projectedPoint.set(manifoldPoint.positionWorldOnA).sub(tmp);
                projectedDifference.set(manifoldPoint.positionWorldOnB).sub(projectedPoint);
                var distance2d = projectedDifference.dot(projectedDifference);
                if (distance2d > getContactBreakingThreshold() * getContactBreakingThreshold()) {
                    removeContactPoint(i);
                }
            }
        }
    }

    public void clearManifold() {
        cachedPoints = 0;
    }
}
