package javax.vecmath;

import java.util.Objects;

public final class Vector3 {
    public float x;
    public float y;
    public float z;

    public Vector3(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3(Vector3 v) {
        this(v.x, v.y, v.z);
    }

    public Vector3() {
        this(0, 0, 0);
    }

    public float length() {
        return (float) Math.sqrt(lengthSquared());
    }

    public float lengthSquared() {
        return dot(this);
    }

    public float dot(Vector3 v) {
        return x * v.x + y * v.y + z * v.z;
    }

    public float getCoord(int index) {
        switch (index) {
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
            default:
                throw new ArrayIndexOutOfBoundsException();
        }
    }

    public void setCoord(int index, float value) {
        switch (index) {
            case 0:
                x = value;
                break;
            case 1:
                y = value;
                break;
            case 2:
                z = value;
                break;
            default:
                throw new ArrayIndexOutOfBoundsException();
        }
    }

    public Vector3 zero() {
        return set(0, 0, 0);
    }

    public Vector3 set(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }

    public Vector3 set(Vector3 v) {
        return set(v.x, v.y, v.z);
    }

    public Vector3 add(Vector3 v) {
        return set(x + v.x, y + v.y, z + v.z);
    }

    public Vector3 sub(Vector3 v) {
        return set(x - v.x, y - v.y, z - v.z);
    }

    public Vector3 mul(float s) {
        return set(x * s, y * s, z * s);
    }

    public Vector3 mul(Vector3 v1, Vector3 v2) {
        return set(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z);
    }

    public Vector3 div(Vector3 v1, Vector3 v2) {
        return set(v1.x / v2.x, v1.y / v2.y, v1.z / v2.z);
    }

    public Vector3 normalize() {
        return mul((float) (1.0 / length()));
    }

    public Vector3 absolute() {
        return set(Math.abs(x), Math.abs(y), Math.abs(z));
    }

    public Vector3 negate() {
        return set(-x, -y, -z);
    }

    public Vector3 cross(Vector3 v1, Vector3 v2) {
        return set(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
    }

    public Vector3 scaleAdd(float s, Vector3 v1, Vector3 v2) {
        return set(s * v1.x + v2.x, s * v1.y + v2.y, s * v1.z + v2.z);
    }

    public Vector3 interpolate(float s, Vector3 v0, Vector3 v1) {
        return set((1 - s) * v0.x + s * v1.x, (1 - s) * v0.y + s * v1.y, (1 - s) * v0.z + s * v1.z);
    }

    public Vector3 transform(Matrix3 m) {
        return set(m.m00 * x + m.m01 * y + m.m02 * z, m.m10 * x + m.m11 * y + m.m12 * z, m.m20 * x + m.m21 * y + m.m22 * z);
    }

    public Vector3 transform(Matrix4 m) {
        return set(m.m00 * x + m.m01 * y + m.m02 * z + m.m03, m.m10 * x + m.m11 * y + m.m12 * z + m.m13, m.m20 * x + m.m21 * y + m.m22 * z + m.m23);
    }

    public Vector3 project(Matrix4 m) {
        var w = m.m30 * x + m.m31 * y + m.m32 * z + m.m33;
        return transform(m).mul(1 / w);
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, z);
    }

    @Override
    public String toString() {
        return String.format("Vector3{x=%s, y=%s, z=%s}", x, y, z);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }
        var v = (Vector3) o;
        return Float.compare(v.x, x) == 0 && Float.compare(v.y, y) == 0 && Float.compare(v.z, z) == 0;
    }
}
