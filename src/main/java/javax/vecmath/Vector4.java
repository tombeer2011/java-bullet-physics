package javax.vecmath;

import java.util.Objects;

public final class Vector4 {
    public float x;
    public float y;
    public float z;
    public float w;

    public Vector4() {
        this(0F, 0F, 0F, 0F);
    }

    public Vector4(float x, float y, float z, float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, z, w);
    }

    @Override
    public String toString() {
        return String.format("Vector4{x=%s, y=%s, z=%s, w=%s}", x, y, z, w);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }
        var v = (Vector4) o;
        return Float.compare(v.x, x) == 0 && Float.compare(v.y, y) == 0 && Float.compare(v.z, z) == 0 && Float.compare(v.w, w) == 0;
    }
}
