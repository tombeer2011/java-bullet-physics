package com.bulletphysics.collision.dispatch;

import com.bulletphysics.Transform;
import com.bulletphysics.collision.narrowphase.DiscreteCollisionDetectorInterface;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.narrowphase.PersistentManifold;

import javax.vecmath.Vector3;

public class ManifoldResult extends DiscreteCollisionDetectorInterface.Result {
    private PersistentManifold manifoldPtr;
    private final Transform rootTransA = new Transform();
    private final Transform rootTransB = new Transform();
    private CollisionObject body0;
    private CollisionObject body1;

    public ManifoldResult() {
    }

    public void init(CollisionObject body0, CollisionObject body1) {
        this.body0 = body0;
        this.body1 = body1;
        body0.getWorldTransform(rootTransA);
        body1.getWorldTransform(rootTransB);
    }

    public PersistentManifold getPersistentManifold() {
        return manifoldPtr;
    }

    public void setPersistentManifold(PersistentManifold manifoldPtr) {
        this.manifoldPtr = manifoldPtr;
    }

    public void addContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, float depth) {

        if (depth > manifoldPtr.getContactBreakingThreshold()) {
            return;
        }
        var isSwapped = manifoldPtr.getBody0() != body0;
        var pointA = new Vector3();
        pointA.scaleAdd(depth, normalOnBInWorld, pointInWorld);
        var localA = new Vector3();
        var localB = new Vector3();
        if (isSwapped) {
            rootTransB.invXform(pointA, localA);
            rootTransA.invXform(pointInWorld, localB);
        } else {
            rootTransA.invXform(pointA, localA);
            rootTransB.invXform(pointInWorld, localB);
        }
        var newPt = new ManifoldPoint();
        newPt.init(localA, localB, normalOnBInWorld, depth);
        newPt.positionWorldOnA.set(pointA);
        newPt.positionWorldOnB.set(pointInWorld);
        var insertIndex = manifoldPtr.getCacheEntry(newPt);
        newPt.combinedFriction = calculateCombinedFriction(body0, body1);
        newPt.combinedRestitution = calculateCombinedRestitution(body0, body1);
        newPt.partId0 = 0;
        newPt.partId1 = 0;
        newPt.index0 = 0;
        newPt.index1 = 0;
        if (insertIndex >= 0) {
            manifoldPtr.replaceContactPoint(newPt, insertIndex);
        } else {
            manifoldPtr.addManifoldPoint(newPt);
        }
    }

    private static float calculateCombinedFriction(CollisionObject body0, CollisionObject body1) {
        var friction = body0.getFriction() * body1.getFriction();
        var MAX_FRICTION = 10f;
        if (friction < -MAX_FRICTION) {
            friction = -MAX_FRICTION;
        }
        if (friction > MAX_FRICTION) {
            friction = MAX_FRICTION;
        }
        return friction;
    }

    private static float calculateCombinedRestitution(CollisionObject body0, CollisionObject body1) {
        return body0.getRestitution() * body1.getRestitution();
    }

    public void refreshContactPoints() {

        if (manifoldPtr.getNumContacts() == 0) {
            return;
        }
        var isSwapped = manifoldPtr.getBody0() != body0;
        if (isSwapped) {
            manifoldPtr.refreshContactPoints(rootTransB, rootTransA);
        } else {
            manifoldPtr.refreshContactPoints(rootTransA, rootTransB);
        }
    }
}
