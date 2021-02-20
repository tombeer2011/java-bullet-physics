package javax.vecmath;

import java.util.Objects;

import static java.lang.Float.compare;

public final class Matrix4 {
    public float m00;
    public float m01;
    public float m02;
    public float m03;
    public float m10;
    public float m11;
    public float m12;
    public float m13;
    public float m20;
    public float m21;
    public float m22;
    public float m23;
    public float m30;
    public float m31;
    public float m32;
    public float m33;

    public Matrix4() {
        set(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public Matrix4 set(float m00, float m01, float m02, float m03, float m10, float m11, float m12, float m13, float m20, float m21, float m22, float m23, float m30, float m31, float m32, float m33) {
        this.m00 = m00;
        this.m01 = m01;
        this.m02 = m02;
        this.m03 = m03;
        this.m10 = m10;
        this.m11 = m11;
        this.m12 = m12;
        this.m13 = m13;
        this.m20 = m20;
        this.m21 = m21;
        this.m22 = m22;
        this.m23 = m23;
        this.m30 = m30;
        this.m31 = m31;
        this.m32 = m32;
        this.m33 = m33;
        return this;
    }

    public Matrix4 set(Matrix4 m) {
        return set(m.m00, m.m01, m.m02, m.m03, m.m10, m.m11, m.m12, m.m13, m.m20, m.m21, m.m22, m.m23, m.m30, m.m31, m.m32, m.m33);
    }

    public Matrix4 set(Matrix3 m1) {
        return set(m1.m00, m1.m01, m1.m02, 0, m1.m10, m1.m11, m1.m12, 0, m1.m20, m1.m21, m1.m22, 0, 0, 0, 0, 1);
    }

    public Matrix4 identity() {
        return set(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    }

    public Matrix4 mul(Matrix4 a) {
        return mul(a, this);
    }

    public Matrix4 mul(Matrix4 m1, Matrix4 m2) {
        return set(m1.m00 * m2.m00 + m1.m01 * m2.m10 + m1.m02 * m2.m20 + m1.m03 * m2.m30, m1.m00 * m2.m01 + m1.m01 * m2.m11 + m1.m02 * m2.m21 + m1.m03 * m2.m31, m1.m00 * m2.m02 + m1.m01 * m2.m12 + m1.m02 * m2.m22 + m1.m03 * m2.m32, m1.m00 * m2.m03 + m1.m01 * m2.m13 + m1.m02 * m2.m23 + m1.m03 * m2.m33, m1.m10 * m2.m00 + m1.m11 * m2.m10 + m1.m12 * m2.m20 + m1.m13 * m2.m30, m1.m10 * m2.m01 + m1.m11 * m2.m11 + m1.m12 * m2.m21 + m1.m13 * m2.m31, m1.m10 * m2.m02 + m1.m11 * m2.m12 + m1.m12 * m2.m22 + m1.m13 * m2.m32, m1.m10 * m2.m03 + m1.m11 * m2.m13 + m1.m12 * m2.m23 + m1.m13 * m2.m33, m1.m20 * m2.m00 + m1.m21 * m2.m10 + m1.m22 * m2.m20 + m1.m23 * m2.m30, m1.m20 * m2.m01 + m1.m21 * m2.m11 + m1.m22 * m2.m21 + m1.m23 * m2.m31, m1.m20 * m2.m02 + m1.m21 * m2.m12 + m1.m22 * m2.m22 + m1.m23 * m2.m32, m1.m20 * m2.m03 + m1.m21 * m2.m13 + m1.m22 * m2.m23 + m1.m23 * m2.m33, m1.m30 * m2.m00 + m1.m31 * m2.m10 + m1.m32 * m2.m20 + m1.m33 * m2.m30, m1.m30 * m2.m01 + m1.m31 * m2.m11 + m1.m32 * m2.m21 + m1.m33 * m2.m31, m1.m30 * m2.m02 + m1.m31 * m2.m12 + m1.m32 * m2.m22 + m1.m33 * m2.m32, m1.m30 * m2.m03 + m1.m31 * m2.m13 + m1.m32 * m2.m23 + m1.m33 * m2.m33);
    }

    public void transform(Vector3 src, Vector3 dst) {
        dst.set(m00 * src.x + m01 * src.y + m02 * src.z + m03, m10 * src.x + m11 * src.y + m12 * src.z + m13, m20 * src.x + m21 * src.y + m22 * src.z + m23);
    }

    public Matrix3 getRotationScale(Matrix3 m1) {
        return m1.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    public Matrix4 rotX(float angle) {
        var c = (float) Math.cos(angle);
        var s = (float) Math.sin(angle);
        return set(1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1);
    }

    public Matrix4 rotY(float angle) {
        var c = (float) Math.cos(angle);
        var s = (float) Math.sin(angle);
        return set(c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1);
    }

    public Matrix4 invert() {
        var b00 = m00 * m11 - m01 * m10;
        var b01 = m00 * m12 - m02 * m10;
        var b02 = m00 * m13 - m03 * m10;
        var b03 = m01 * m12 - m02 * m11;
        var b04 = m01 * m13 - m03 * m11;
        var b05 = m02 * m13 - m03 * m12;
        var b06 = m20 * m31 - m21 * m30;
        var b07 = m20 * m32 - m22 * m30;
        var b08 = m20 * m33 - m23 * m30;
        var b09 = m21 * m32 - m22 * m31;
        var b10 = m21 * m33 - m23 * m31;
        var b11 = m22 * m33 - m23 * m32;
        var invDet = 1 / (b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06);
        return set((m11 * b11 - m12 * b10 + m13 * b09) * invDet, (-m01 * b11 + m02 * b10 - m03 * b09) * invDet, (m31 * b05 - m32 * b04 + m33 * b03) * invDet, (-m21 * b05 + m22 * b04 - m23 * b03) * invDet, (-m10 * b11 + m12 * b08 - m13 * b07) * invDet, (m00 * b11 - m02 * b08 + m03 * b07) * invDet, (-m30 * b05 + m32 * b02 - m33 * b01) * invDet, (m20 * b05 - m22 * b02 + m23 * b01) * invDet, (m10 * b10 - m11 * b08 + m13 * b06) * invDet, (-m00 * b10 + m01 * b08 - m03 * b06) * invDet, (m30 * b04 - m31 * b02 + m33 * b00) * invDet, (-m20 * b04 + m21 * b02 - m23 * b00) * invDet, (-m10 * b09 + m11 * b07 - m12 * b06) * invDet, (m00 * b09 - m01 * b07 + m02 * b06) * invDet, (-m30 * b03 + m31 * b01 - m32 * b00) * invDet, (m20 * b03 - m21 * b01 + m22 * b00) * invDet);
    }

    public static Matrix4 createTransform(Vector3 scale, Vector3 rotate, Vector3 position) {
        var ci = (float) Math.cos(rotate.x);
        var si = (float) Math.sin(rotate.x);
        var cj = (float) Math.cos(rotate.y);
        var sj = (float) Math.sin(rotate.y);
        var ch = (float) Math.cos(rotate.z);
        var sh = (float) Math.sin(rotate.z);
        return new Matrix4().set(scale.x * (cj * ch), scale.x * (sj * si * ch - ci * sh), scale.x * (sj * ci * ch + si * sh), position.x, scale.y * (cj * sh), scale.y * (sj * si * sh + ci * ch), scale.y * (sj * ci * sh - si * ch), position.y, scale.z * (0 - sj), scale.z * (cj * si), scale.z * (cj * ci), position.z, 0, 0, 0, 1);
    }

    @Override
    public int hashCode() {
        return Objects.hash(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33);
    }

    @Override
    public String toString() {
        var template = "Matrix4{m00=%s, m01=%s, m02=%s, m03=%s, m10=%s, m11=%s, m12=%s, m13=%s, m20=%s, m21=%s, m22=%s, m23=%s, m30=%s, m31=%s, m32=%s, m33=%s}";
        return String.format(template, m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }
        Matrix4 m = (Matrix4) o;
        return compare(m.m00, m00) == 0 && compare(m.m01, m01) == 0 && compare(m.m02, m02) == 0 && compare(m.m03, m03) == 0 && compare(m.m10, m10) == 0 && compare(m.m11, m11) == 0 && compare(m.m12, m12) == 0 && compare(m.m13, m13) == 0 && compare(m.m20, m20) == 0 && compare(m.m21, m21) == 0 && compare(m.m22, m22) == 0 && compare(m.m23, m23) == 0 && compare(m.m30, m30) == 0 && compare(m.m31, m31) == 0 && compare(m.m32, m32) == 0 && compare(m.m33, m33) == 0;
    }
}
