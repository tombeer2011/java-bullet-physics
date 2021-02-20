package com.bulletphysics.collision.shapes;

import com.bulletphysics.Transform;

import javax.vecmath.Vector3;

public abstract class CollisionShape {
    public abstract void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax);

    public void getBoundingSphere(Vector3 center, float[] radius) {
        var tmp = new Vector3();
        var tr = new Transform();
        tr.identity();
        Vector3 aabbMin = new Vector3(), aabbMax = new Vector3();
        getAabb(tr, aabbMin, aabbMax);
        tmp.set(aabbMax).sub(aabbMin);
        radius[0] = tmp.length() * 0.5f;
        tmp.set(aabbMin).add(aabbMax);
        center.set(tmp).mul(0.5f);
    }

    public float getAngularMotionDisc() {
        var center = new Vector3();
        var disc = new float[1];
        getBoundingSphere(center, disc);
        disc[0] += center.length();
        return disc[0];
    }

    public void calculateTemporalAabb(Transform curTrans, Vector3 linvel, Vector3 angvel, float timeStep, Vector3 temporalAabbMin, Vector3 temporalAabbMax) {
        getAabb(curTrans, temporalAabbMin, temporalAabbMax);
        var temporalAabbMaxx = temporalAabbMax.x;
        var temporalAabbMaxy = temporalAabbMax.y;
        var temporalAabbMaxz = temporalAabbMax.z;
        var temporalAabbMinx = temporalAabbMin.x;
        var temporalAabbMiny = temporalAabbMin.y;
        var temporalAabbMinz = temporalAabbMin.z;
        var linMotion = new Vector3(linvel);
        linMotion.mul(timeStep);
        if (linMotion.x > 0f) {
            temporalAabbMaxx += linMotion.x;
        } else {
            temporalAabbMinx += linMotion.x;
        }
        if (linMotion.y > 0f) {
            temporalAabbMaxy += linMotion.y;
        } else {
            temporalAabbMiny += linMotion.y;
        }
        if (linMotion.z > 0f) {
            temporalAabbMaxz += linMotion.z;
        } else {
            temporalAabbMinz += linMotion.z;
        }
        var angularMotion = angvel.length() * getAngularMotionDisc() * timeStep;
        var angularMotion3d = new Vector3();
        angularMotion3d.set(angularMotion, angularMotion, angularMotion);
        temporalAabbMin.set(temporalAabbMinx, temporalAabbMiny, temporalAabbMinz);
        temporalAabbMax.set(temporalAabbMaxx, temporalAabbMaxy, temporalAabbMaxz);
        temporalAabbMin.sub(angularMotion3d);
        temporalAabbMax.add(angularMotion3d);
    }

    public boolean isConvex() {
        return true;
    }

    public abstract void setLocalScaling(Vector3 scaling);

    public abstract void calculateLocalInertia(float mass, Vector3 inertia);

    public abstract void setMargin(float margin);

    public abstract float getMargin();
}
