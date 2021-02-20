package javax.vecmath;

import java.util.Objects;

public final class Quaternion {
    public float x;
    public float y;
    public float z;
    public float w;

    public Quaternion() {
        this(0, 0, 0, 0);
    }

    public Quaternion(float x, float y, float z, float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    public Quaternion set(float x, float y, float z, float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
        return this;
    }

    public Quaternion mul(Quaternion q1) {
        return mul(this, q1);
    }

    public Quaternion mul(Quaternion q1, Quaternion q2) {
        var x = q1.x * q2.w + q1.w * q2.x + q1.y * q2.z - q1.z * q2.y;
        var y = q1.y * q2.w + q1.w * q2.y + q1.z * q2.x - q1.x * q2.z;
        var z = q1.z * q2.w + q1.w * q2.z + q1.x * q2.y - q1.y * q2.x;
        var w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
        return set(x, y, z, w);
    }

    public Quaternion normalize() {
        var n = (float) (1.0 / Math.sqrt(x * x + y * y + z * z + w * w));
        return set(x * n, y * n, z * n, w * n);
    }

    public float getAngle() {
        return 2 * (float) Math.acos(w);
    }

    public Quaternion setRotation(Vector3 axis, float angle) {
        var d = axis.length();
        var s = (float) Math.sin(angle * 0.5f) / d;
        return set(axis.x * s, axis.y * s, axis.z * s, (float) Math.cos(angle * 0.5f));
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, z, w);
    }

    @Override
    public String toString() {
        return String.format("Quaternion{x=%s, y=%s, z=%s, w=%s}", x, y, z, w);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }
        var q = (Quaternion) o;
        return Float.compare(q.x, x) == 0 && Float.compare(q.y, y) == 0 && Float.compare(q.z, z) == 0 && Float.compare(q.w, w) == 0;
    }
}
