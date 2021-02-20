package zynaps.engine;

import javax.vecmath.Matrix4;
import javax.vecmath.Vector4;

public final class Box {
    public float minX;
    public float minY;
    public float minZ;
    public float maxX;
    public float maxY;
    public float maxZ;

    public Box() {
        reset();
    }

    public void reset() {
        minX = Float.POSITIVE_INFINITY;
        minY = Float.POSITIVE_INFINITY;
        minZ = Float.POSITIVE_INFINITY;
        maxX = Float.NEGATIVE_INFINITY;
        maxY = Float.NEGATIVE_INFINITY;
        maxZ = Float.NEGATIVE_INFINITY;
    }

    public int eval(Vector4 plane) {
        var xMin = plane.x * minX;
        var xMax = plane.x * maxX;
        var yMin = plane.y * minY;
        var yMax = plane.y * maxY;
        var zMin = plane.z * minZ;
        var zMax = plane.z * maxZ;
        return (xMin + yMax + zMin > plane.w ? 1 : 0) +
                (xMax + yMax + zMin > plane.w ? 1 : 0) +
                (xMax + yMin + zMin > plane.w ? 1 : 0) +
                (xMin + yMin + zMin > plane.w ? 1 : 0) +
                (xMin + yMax + zMax > plane.w ? 1 : 0) +
                (xMax + yMax + zMax > plane.w ? 1 : 0) +
                (xMax + yMin + zMax > plane.w ? 1 : 0) +
                (xMin + yMin + zMax > plane.w ? 1 : 0);
    }

    public void aggregate(float x, float y, float z) {
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
        if (z < minZ) minZ = z;
        if (z > maxZ) maxZ = z;
    }

    public void aggregateInto(Box target) {
        target.aggregate(minX, minY, minZ);
        target.aggregate(maxX, maxY, maxZ);
    }

    public void aggregateInto(Box target, Matrix4 matrix) {
        var min11 = matrix.m00 * minX;
        var max11 = matrix.m00 * maxX;
        var min12 = matrix.m01 * minY;
        var max12 = matrix.m01 * maxY;
        var min13 = matrix.m02 * minZ + matrix.m03;
        var max13 = matrix.m02 * maxZ + matrix.m03;
        var min21 = matrix.m10 * minX;
        var max21 = matrix.m10 * maxX;
        var min22 = matrix.m11 * minY;
        var max22 = matrix.m11 * maxY;
        var min23 = matrix.m12 * minZ + matrix.m13;
        var max23 = matrix.m12 * maxZ + matrix.m13;
        var min31 = matrix.m20 * minX;
        var max31 = matrix.m20 * maxX;
        var min32 = matrix.m21 * minY;
        var max32 = matrix.m21 * maxY;
        var min33 = matrix.m22 * minZ + matrix.m23;
        var max33 = matrix.m22 * maxZ + matrix.m23;
        var v3 = min12 + min13;
        var v4 = min22 + min23;
        var v5 = min32 + min33;
        var v0 = max12 + min13;
        var v1 = max22 + min23;
        var v2 = max32 + min33;
        target.aggregate(min11 + v0, min21 + v1, min31 + v2);
        target.aggregate(max11 + v0, max21 + v1, max31 + v2);
        target.aggregate(max11 + v3, max21 + v4, max31 + v5);
        target.aggregate(min11 + v3, min21 + v4, min31 + v5);
        v3 = min12 + max13;
        v4 = min22 + max23;
        v5 = min32 + max33;
        v0 = max12 + max13;
        v1 = max22 + max23;
        v2 = max32 + max33;
        target.aggregate(min11 + v0, min21 + v1, min31 + v2);
        target.aggregate(max11 + v0, max21 + v1, max31 + v2);
        target.aggregate(max11 + v3, max21 + v4, max31 + v5);
        target.aggregate(min11 + v3, min21 + v4, min31 + v5);
    }
}
