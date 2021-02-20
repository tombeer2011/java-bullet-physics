package com.bulletphysics;

import javax.vecmath.Matrix3;
import javax.vecmath.Matrix4;
import javax.vecmath.Quaternion;
import javax.vecmath.Vector3;
import java.util.Objects;

public final class Transform {
    public final Matrix3 basis = new Matrix3();
    public final Vector3 origin = new Vector3();

    public Transform() {
    }

    public Transform(Matrix3 m) {
        set(m);
    }

    public Transform(Matrix4 m) {
        set(m);
    }

    public Transform(Transform t) {
        set(t);
    }

    public Transform set(Transform t) {
        basis.set(t.basis);
        origin.set(t.origin);
        return this;
    }

    public Transform set(Matrix3 m) {
        basis.set(m);
        origin.set(0, 0, 0);
        return this;
    }

    public Transform set(Matrix4 m) {
        m.getRotationScale(basis);
        origin.set(m.m03, m.m13, m.m23);
        return this;
    }

    public Vector3 transform(Vector3 v) {
        return v.transform(basis).add(origin);
    }

    public Transform identity() {
        basis.identity();
        origin.set(0f, 0f, 0f);
        return this;
    }

    public Transform inverse() {
        basis.transpose();
        origin.mul(-1).transform(basis);
        return this;
    }

    public Transform inverse(Transform t) {
        set(t).inverse();
        return this;
    }

    public Transform mul(Transform t) {
        basis.mul(t.basis);
        origin.set(transform(new Vector3(t.origin)));
        return this;
    }

    public Transform mul(Transform t1, Transform t2) {
        basis.mul(t1.basis, t2.basis);
        origin.set(t1.transform(new Vector3(t2.origin)));
        return this;
    }

    public Transform invXform(Vector3 inVec, Vector3 out) {
        out.set(inVec).sub(origin).transform(new Matrix3(basis).transpose());
        return this;
    }

    public Quaternion getRotation(Quaternion out) {
        basis.getRotation(out);
        return out;
    }

    public Transform setRotation(Quaternion q) {
        basis.setRotation(q);
        return this;
    }

    public Matrix4 getMatrix(Matrix4 out) {
        out.set(basis);
        out.m03 = origin.x;
        out.m13 = origin.y;
        out.m23 = origin.z;
        return out;
    }

    @Override
    public int hashCode() {
        return Objects.hash(basis, origin);
    }

    @Override
    public String toString() {
        return "Transform{" + "basis=" + basis + ", origin=" + origin + '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Transform transform = (Transform) o;
        return basis.equals(transform.basis) && origin.equals(transform.origin);
    }

    private static final float SIMDSQRT12 = 0.7071067811865475244008443621048490f;
    private static final float ANGULAR_MOTION_THRESHOLD = 0.5f * BulletGlobals.HALF_PI;

    public static void planeSpace1(Vector3 n, Vector3 p, Vector3 q) {
        if (Math.abs(n.z) > SIMDSQRT12) {
            var a = n.y * n.y + n.z * n.z;
            var k = (float) (1 / Math.sqrt(a));
            p.set(0, -n.z * k, n.y * k);
            q.set(a * k, -n.x * p.z, n.x * p.y);
        } else {
            var a = n.x * n.x + n.y * n.y;
            var k = (float) (1 / Math.sqrt(a));
            p.set(-n.y * k, n.x * k, 0);
            q.set(-n.z * p.y, n.z * p.x, a * k);
        }
    }

    public static void integrateTransform(Transform curTrans, Vector3 linvel, Vector3 angvel, float timeStep, Transform predictedTransform) {
        predictedTransform.origin.scaleAdd(timeStep, linvel, curTrans.origin);
        var axis = new Vector3();
        var fAngle = angvel.length();
        if (fAngle * timeStep > ANGULAR_MOTION_THRESHOLD) {
            fAngle = ANGULAR_MOTION_THRESHOLD / timeStep;
        }
        if (fAngle < 0.001f) {
            axis.set(angvel).mul(0.5f * timeStep - timeStep * timeStep * timeStep * 0.020833333333f * fAngle * fAngle);
        } else {
            axis.set(angvel).mul((float) Math.sin(0.5f * fAngle * timeStep) / fAngle);
        }
        var dorn = new Quaternion(axis.x, axis.y, axis.z, (float) Math.cos(fAngle * timeStep * 0.5f));
        var orn0 = curTrans.getRotation(new Quaternion());
        predictedTransform.setRotation(new Quaternion().mul(dorn, orn0).normalize());
    }

    public static void calculateVelocity(Transform transform0, Transform transform1, float timeStep, Vector3 linVel, Vector3 angVel) {
        linVel.set(transform1.origin).sub(transform0.origin);
        linVel.mul(1f / timeStep);
        var angle = new float[1];
        var axis = new Vector3();
        calculateDiffAxisAngle(transform0, transform1, axis, angle);
        angVel.set(axis).mul(angle[0] / timeStep);
    }

    public static void calculateDiffAxisAngle(Transform transform0, Transform transform1, Vector3 axis, float[] angle) {
        var dmat = new Matrix3().mul(transform1.basis, new Matrix3(transform0.basis).invert());
        var dorn = dmat.getRotation(new Quaternion()).normalize();
        angle[0] = dorn.getAngle();
        axis.set(dorn.x, dorn.y, dorn.z);
        var len = axis.lengthSquared();
        if (len < BulletGlobals.EPSILON * BulletGlobals.EPSILON) {
            axis.set(1, 0, 0);
        } else {
            axis.mul((float) (1 / Math.sqrt(len)));
        }
    }
}
