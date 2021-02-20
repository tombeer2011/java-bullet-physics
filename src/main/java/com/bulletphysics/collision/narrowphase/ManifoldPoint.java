package com.bulletphysics.collision.narrowphase;

import javax.vecmath.Vector3;

public class ManifoldPoint {
    public final Vector3 localPointA = new Vector3();
    public final Vector3 localPointB = new Vector3();
    public final Vector3 positionWorldOnB = new Vector3();
    public final Vector3 positionWorldOnA = new Vector3();
    public final Vector3 normalWorldOnB = new Vector3();
    public float distance1;
    public float combinedFriction;
    public float combinedRestitution;
    public int partId0;
    public int partId1;
    public int index0;
    public int index1;
    public Object userPersistentData;
    public float appliedImpulse;
    public boolean lateralFrictionInitialized;
    public float appliedImpulseLateral1;
    public float appliedImpulseLateral2;
    public int lifeTime;
    public final Vector3 lateralFrictionDir1 = new Vector3();
    public final Vector3 lateralFrictionDir2 = new Vector3();

    public ManifoldPoint() {
        userPersistentData = null;
        appliedImpulse = 0f;
        lateralFrictionInitialized = false;
        lifeTime = 0;
    }

    public void init(Vector3 pointA, Vector3 pointB, Vector3 normal, float distance) {
        localPointA.set(pointA);
        localPointB.set(pointB);
        normalWorldOnB.set(normal);
        distance1 = distance;
        combinedFriction = 0f;
        combinedRestitution = 0f;
        userPersistentData = null;
        appliedImpulse = 0f;
        lateralFrictionInitialized = false;
        appliedImpulseLateral1 = 0f;
        appliedImpulseLateral2 = 0f;
        lifeTime = 0;
    }

    public float getDistance() {
        return distance1;
    }

    public int getLifeTime() {
        return lifeTime;
    }

    public void set(ManifoldPoint p) {
        localPointA.set(p.localPointA);
        localPointB.set(p.localPointB);
        positionWorldOnA.set(p.positionWorldOnA);
        positionWorldOnB.set(p.positionWorldOnB);
        normalWorldOnB.set(p.normalWorldOnB);
        distance1 = p.distance1;
        combinedFriction = p.combinedFriction;
        combinedRestitution = p.combinedRestitution;
        partId0 = p.partId0;
        partId1 = p.partId1;
        index0 = p.index0;
        index1 = p.index1;
        userPersistentData = p.userPersistentData;
        appliedImpulse = p.appliedImpulse;
        lateralFrictionInitialized = p.lateralFrictionInitialized;
        appliedImpulseLateral1 = p.appliedImpulseLateral1;
        appliedImpulseLateral2 = p.appliedImpulseLateral2;
        lifeTime = p.lifeTime;
        lateralFrictionDir1.set(p.lateralFrictionDir1);
        lateralFrictionDir2.set(p.lateralFrictionDir2);
    }

    public Vector3 getPositionWorldOnA(Vector3 out) {
        out.set(positionWorldOnA);
        return out;
    }

    public Vector3 getPositionWorldOnB(Vector3 out) {
        out.set(positionWorldOnB);
        return out;
    }
}
