package javax.vecmath;

import java.util.Objects;

import static java.lang.Math.abs;

public final class Matrix3 {
    public float m00;
    public float m01;
    public float m02;
    public float m10;
    public float m11;
    public float m12;
    public float m20;
    public float m21;
    public float m22;

    public Matrix3(Matrix3 m) {
        set(m.m00, m.m01, m.m02, m.m10, m.m11, m.m12, m.m20, m.m21, m.m22);
    }

    public Matrix3() {
        set(0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public Matrix3 set(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22) {
        this.m00 = m00;
        this.m01 = m01;
        this.m02 = m02;
        this.m10 = m10;
        this.m11 = m11;
        this.m12 = m12;
        this.m20 = m20;
        this.m21 = m21;
        this.m22 = m22;
        return this;
    }

    public Matrix3 identity() {
        return set(1, 0, 0, 0, 1, 0, 0, 0, 1);
    }

    public Matrix3 set(Matrix3 m) {
        return set(m.m00, m.m01, m.m02, m.m10, m.m11, m.m12, m.m20, m.m21, m.m22);
    }

    public Matrix3 add(Matrix3 m) {
        return set(m00 + m.m00, m01 + m.m01, m02 + m.m02, m10 + m.m10, m11 + m.m11, m12 + m.m12, m20 + m.m20, m21 + m.m21, m22 + m.m22);
    }

    public Matrix3 mul(float s) {
        return set(m00 * s, m01 * s, m02 * s, m10 * s, m11 * s, m12 * s, m20 * s, m21 * s, m22 * s);
    }

    public Matrix3 mul(Matrix3 m) {
        return mul(this, m);
    }

    public Matrix3 mul(Matrix3 a, Matrix3 b) {
        return set(a.m00 * b.m00 + a.m01 * b.m10 + a.m02 * b.m20, a.m00 * b.m01 + a.m01 * b.m11 + a.m02 * b.m21, a.m00 * b.m02 + a.m01 * b.m12 + a.m02 * b.m22, a.m10 * b.m00 + a.m11 * b.m10 + a.m12 * b.m20, a.m10 * b.m01 + a.m11 * b.m11 + a.m12 * b.m21, a.m10 * b.m02 + a.m11 * b.m12 + a.m12 * b.m22, a.m20 * b.m00 + a.m21 * b.m10 + a.m22 * b.m20, a.m20 * b.m01 + a.m21 * b.m11 + a.m22 * b.m21, a.m20 * b.m02 + a.m21 * b.m12 + a.m22 * b.m22);
    }

    public Matrix3 transpose() {
        return set(m00, m10, m20, m01, m11, m21, m02, m12, m22);
    }

    public Matrix3 invert() {
        var co_x = getElement(1, 1) * getElement(2, 2) - getElement(1, 2) * getElement(2, 1);
        var co_y = getElement(1, 2) * getElement(2, 0) - getElement(1, 0) * getElement(2, 2);
        var co_z = getElement(1, 0) * getElement(2, 1) - getElement(1, 1) * getElement(2, 0);
        var det = m00 * co_x + m01 * co_y + m02 * co_z;
        var s = 1f / det;
        var m00 = co_x * s;
        var m01 = (getElement(0, 2) * getElement(2, 1) - getElement(0, 1) * getElement(2, 2)) * s;
        var m02 = (getElement(0, 1) * getElement(1, 2) - getElement(0, 2) * getElement(1, 1)) * s;
        var m10 = co_y * s;
        var m11 = (getElement(0, 0) * getElement(2, 2) - getElement(0, 2) * getElement(2, 0)) * s;
        var m12 = (getElement(0, 2) * getElement(1, 0) - getElement(0, 0) * getElement(1, 2)) * s;
        var m20 = co_z * s;
        var m21 = (getElement(0, 1) * getElement(2, 0) - getElement(0, 0) * getElement(2, 1)) * s;
        var m22 = (getElement(0, 0) * getElement(1, 1) - getElement(0, 1) * getElement(1, 0)) * s;
        return set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    public Vector3 getRow(int row, Vector3 v) {
        if (row == 0) {
            v.x = m00;
            v.y = m01;
            v.z = m02;
        } else if (row == 1) {
            v.x = m10;
            v.y = m11;
            v.z = m12;
        } else if (row == 2) {
            v.x = m20;
            v.y = m21;
            v.z = m22;
        } else {
            throw new ArrayIndexOutOfBoundsException("row must be 0 to 2 and is " + row);
        }
        return v;
    }

    private float getElement(int row, int column) {
        switch (row) {
            case 0:
                return new float[]{m00, m01, m02}[column];
            case 1:
                return new float[]{m10, m11, m12}[column];
            case 2:
                return new float[]{m20, m21, m22}[column];
            default:
                throw new ArrayIndexOutOfBoundsException();
        }
    }

    public Matrix3 scale(Matrix3 m, Vector3 s) {
        return set(m.m00 * s.x, m.m01 * s.y, m.m02 * s.z, m.m10 * s.x, m.m11 * s.y, m.m12 * s.z, m.m20 * s.x, m.m21 * s.y, m.m22 * s.z);
    }

    public Matrix3 absolute() {
        return set(abs(m00), abs(m01), abs(m02), abs(m10), abs(m11), abs(m12), abs(m20), abs(m21), abs(m22));
    }

    public void transposeTransform(Vector3 dst, Vector3 src) {
        dst.set(m00 * src.x + m10 * src.y + m20 * src.z, m01 * src.x + m11 * src.y + m21 * src.z, m02 * src.x + m12 * src.y + m22 * src.z);
    }

    public Matrix3 setRotation(Quaternion q) {
        var s = 2 / (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        float xs = q.x * s, ys = q.y * s, zs = q.z * s;
        float wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
        float xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
        float yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
        return set(1f - (yy + zz), xy - wz, xz + wy, xy + wz, 1f - (xx + zz), yz - wx, xz - wy, yz + wx, 1f - (xx + yy));
    }

    public Quaternion getRotation(Quaternion dest) {
        var trace = m00 + m11 + m22;
        var temp = new Float[4];
        if (trace > 0f) {
            var s = (float) Math.sqrt(trace + 1f);
            temp[3] = s * 0.5f;
            s = 0.5f / s;
            temp[0] = (m21 - m12) * s;
            temp[1] = (m02 - m20) * s;
            temp[2] = (m10 - m01) * s;
        } else {
            var i = m00 < m11 ? m11 < m22 ? 2 : 1 : m00 < m22 ? 2 : 0;
            var j = (i + 1) % 3;
            var k = (i + 2) % 3;
            var s = (float) Math.sqrt(getElement(i, i) - getElement(j, j) - getElement(k, k) + 1f);
            temp[i] = s * 0.5f;
            s = 0.5f / s;
            temp[3] = (getElement(k, j) - getElement(j, k)) * s;
            temp[j] = (getElement(j, i) + getElement(i, j)) * s;
            temp[k] = (getElement(k, i) + getElement(i, k)) * s;
        }
        return dest.set(temp[0], temp[1], temp[2], temp[3]);
    }

    @Override
    public int hashCode() {
        return Objects.hash(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    @Override
    public String toString() {
        var template = "Matrix3{m00=%s, m01=%s, m02=%s, m10=%s, m11=%s, m12=%s, m20=%s, m21=%s, m22=%s}";
        return String.format(template, m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }
        var m = (Matrix3) o;
        return Float.compare(m.m00, m00) == 0 && Float.compare(m.m01, m01) == 0 && Float.compare(m.m02, m02) == 0 && Float.compare(m.m10, m10) == 0 && Float.compare(m.m11, m11) == 0 && Float.compare(m.m12, m12) == 0 && Float.compare(m.m20, m20) == 0 && Float.compare(m.m21, m21) == 0 && Float.compare(m.m22, m22) == 0;
    }
}
