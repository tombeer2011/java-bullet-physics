package com.bulletphysics;

import javax.vecmath.Matrix3;
import javax.vecmath.Vector3;

public class AabbUtil2 {
    public static boolean rayAabb(Vector3 rayFrom, Vector3 rayTo, Vector3 aabbMin, Vector3 aabbMax, float[] param, Vector3 normal) {
        var aabbHalfExtent = new Vector3(aabbMax).sub(aabbMin).mul(0.5f);
        var aabbCenter = new Vector3(aabbMax).add(aabbMin).mul(0.5f);
        var source = new Vector3(rayFrom).sub(aabbCenter);
        var target = new Vector3(rayTo).sub(aabbCenter);
        var sourceOutCode = outCode(source, aabbHalfExtent);
        var targetOutCode = outCode(target, aabbHalfExtent);
        if ((sourceOutCode & targetOutCode) == 0x0) {
            var lambda_enter = 0f;
            var lambda_exit = param[0];
            var r = new Vector3(target).sub(source);
            var normSign = 1f;
            var bit = 1;
            var hitNormal = new Vector3();
            for (var j = 0; j < 2; ++j) {
                for (var i = 0; i != 3; ++i) {
                    if ((sourceOutCode & bit) != 0) {
                        var lambda = (-source.getCoord(i) - aabbHalfExtent.getCoord(i) * normSign) / r.getCoord(i);
                        if (lambda_enter <= lambda) {
                            lambda_enter = lambda;
                            hitNormal.set(0f, 0f, 0f);
                            hitNormal.setCoord(i, normSign);
                        }
                    } else if ((targetOutCode & bit) != 0) {
                        var lambda = (-source.getCoord(i) - aabbHalfExtent.getCoord(i) * normSign) / r.getCoord(i);
                        lambda_exit = Math.min(lambda_exit, lambda);
                    }
                    bit <<= 1;
                }
                normSign = -1f;
            }
            if (lambda_enter <= lambda_exit) {
                param[0] = lambda_enter;
                normal.set(hitNormal);
                return true;
            }
        }
        return false;
    }

    public static void transformAabb(Vector3 halfExtents, float margin, Transform t, Vector3 aabbMinOut, Vector3 aabbMaxOut) {
        var halfExtentsWithMargin = new Vector3(halfExtents.x + margin, halfExtents.y + margin, halfExtents.z + margin);
        var absBasis = new Matrix3(t.basis).absolute();
        var tmp = new Vector3();
        var extent = new Vector3(absBasis.getRow(0, tmp).dot(halfExtentsWithMargin), absBasis.getRow(1, tmp).dot(halfExtentsWithMargin), absBasis.getRow(2, tmp).dot(halfExtentsWithMargin));
        aabbMinOut.set(t.origin).sub(extent);
        aabbMaxOut.set(t.origin).add(extent);
    }

    public static void transformAabb(Vector3 localAabbMin, Vector3 localAabbMax, float margin, Transform trans, Vector3 aabbMinOut, Vector3 aabbMaxOut) {
        var center = trans.transform(new Vector3(localAabbMax).add(localAabbMin).mul(0.5f));
        var localHalfExtents = new Vector3(localAabbMax).sub(localAabbMin).mul(0.5f);
        localHalfExtents.x += margin;
        localHalfExtents.y += margin;
        localHalfExtents.z += margin;
        var absBasis = new Matrix3(trans.basis).absolute();
        var tmp = new Vector3();
        var extent = new Vector3(absBasis.getRow(0, tmp).dot(localHalfExtents), absBasis.getRow(1, tmp).dot(localHalfExtents), absBasis.getRow(2, tmp).dot(localHalfExtents));
        aabbMinOut.set(center).sub(extent);
        aabbMaxOut.set(center).add(extent);
    }

    private static int outCode(Vector3 p, Vector3 halfExtent) {
        var i0 = p.x < -halfExtent.x ? 0x01 : 0x0;
        var i2 = p.y < -halfExtent.y ? 0x02 : 0x0;
        var i4 = p.z < -halfExtent.z ? 0x04 : 0x0;
        var i1 = p.x > +halfExtent.x ? 0x08 : 0x0;
        var i3 = p.y > +halfExtent.y ? 0x10 : 0x0;
        var i5 = p.z > +halfExtent.z ? 0x20 : 0x0;
        return i0 | i1 | i2 | i3 | i4 | i5;
    }
}
