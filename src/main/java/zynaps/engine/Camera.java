package zynaps.engine;

import javax.vecmath.Matrix4;
import javax.vecmath.Vector3;
import javax.vecmath.Vector4;

public class Camera {
    private static final Vector3 UP_VECTOR = new Vector3(0, 1, 0);
    private final Vector3 cameraPosition = new Vector3(0, 0, 1);
    private final Vector3 targetPosition = new Vector3();
    private final Vector3 topLeft = new Vector3();
    private final Vector3 bottomRight = new Vector3();
    private final Matrix4 viewMatrix = new Matrix4().identity();
    private final Matrix4 projMatrix = new Matrix4();
    private final float aspectRatio;
    private final float near;
    private final float far;

    public Camera(float fov, float aspectRatio, float near, float far) {
        this.aspectRatio = aspectRatio;
        this.near = near;
        this.far = far;
        projMatrix.m11 = 1 / (float) Math.tan(fov / 2.0);
        projMatrix.m00 = projMatrix.m11 / aspectRatio;
        projMatrix.m22 = far / (far - near);
        projMatrix.m23 = -projMatrix.m22 * near;
        projMatrix.m32 = 1;
    }

    public Vector3 getPosition(Vector3 dest) {
        return dest.set(cameraPosition);
    }

    public Vector3 getTarget(Vector3 dest) {
        return dest.set(targetPosition);
    }

    public Matrix4 getProjection(Matrix4 dest) {
        return dest.mul(projMatrix, calculateViewMatrix());
    }

    public void moveTo(Vector3 v) {
        moveTo(v.x, v.y, v.z);
    }

    public void moveTo(float x, float y, float z) {
        cameraPosition.set(x, y, z);
    }

    public void lookAt(float x, float y, float z) {
        targetPosition.set(x, y, z);
    }

    public void configure(Vector4 left, Vector4 right, Vector4 top, Vector4 bottom, Vector4 near, Vector4 far) {
        calculateViewMatrix();
        right.x = topLeft.y * viewMatrix.m12 - topLeft.z * viewMatrix.m11;
        right.y = topLeft.z * viewMatrix.m10 - topLeft.x * viewMatrix.m12;
        right.z = topLeft.x * viewMatrix.m11 - topLeft.y * viewMatrix.m10;
        right.w = cameraPosition.x * right.x + cameraPosition.y * right.y + cameraPosition.z * right.z;
        top.x = topLeft.y * viewMatrix.m02 - topLeft.z * viewMatrix.m01;
        top.y = topLeft.z * viewMatrix.m00 - topLeft.x * viewMatrix.m02;
        top.z = topLeft.x * viewMatrix.m01 - topLeft.y * viewMatrix.m00;
        top.w = cameraPosition.x * top.x + cameraPosition.y * top.y + cameraPosition.z * top.z;
        left.x = viewMatrix.m11 * bottomRight.z - viewMatrix.m12 * bottomRight.y;
        left.y = viewMatrix.m12 * bottomRight.x - viewMatrix.m10 * bottomRight.z;
        left.z = viewMatrix.m10 * bottomRight.y - viewMatrix.m11 * bottomRight.x;
        left.w = cameraPosition.x * left.x + cameraPosition.y * left.y + cameraPosition.z * left.z;
        bottom.x = viewMatrix.m01 * bottomRight.z - viewMatrix.m02 * bottomRight.y;
        bottom.y = viewMatrix.m02 * bottomRight.x - viewMatrix.m00 * bottomRight.z;
        bottom.z = viewMatrix.m00 * bottomRight.y - viewMatrix.m01 * bottomRight.x;
        bottom.w = cameraPosition.x * bottom.x + cameraPosition.y * bottom.y + cameraPosition.z * bottom.z;
        near.x = -viewMatrix.m20;
        near.y = -viewMatrix.m21;
        near.z = -viewMatrix.m22;
        near.w = (cameraPosition.x + viewMatrix.m20 * this.near) * near.x + (cameraPosition.y + viewMatrix.m21 * this.near) * near.y + (cameraPosition.z + viewMatrix.m22 * this.near) * near.z;
        far.x = viewMatrix.m20;
        far.y = viewMatrix.m21;
        far.z = viewMatrix.m22;
        far.w = (cameraPosition.x + viewMatrix.m20 * this.far) * far.x + (cameraPosition.y + viewMatrix.m21 * this.far) * far.y + (cameraPosition.z + viewMatrix.m22 * this.far) * far.z;
    }

    private Matrix4 calculateViewMatrix() {
        viewMatrix.m20 = targetPosition.x - cameraPosition.x;
        viewMatrix.m21 = targetPosition.y - cameraPosition.y;
        viewMatrix.m22 = targetPosition.z - cameraPosition.z;
        var s = 1 / (float) Math.sqrt(viewMatrix.m20 * viewMatrix.m20 + viewMatrix.m21 * viewMatrix.m21 + viewMatrix.m22 * viewMatrix.m22);
        viewMatrix.m20 = viewMatrix.m20 * s;
        viewMatrix.m21 = viewMatrix.m21 * s;
        viewMatrix.m22 = viewMatrix.m22 * s;
        viewMatrix.m23 = -cameraPosition.x * viewMatrix.m20 - cameraPosition.y * viewMatrix.m21 - cameraPosition.z * viewMatrix.m22;
        viewMatrix.m00 = UP_VECTOR.y * viewMatrix.m22 - UP_VECTOR.z * viewMatrix.m21;
        viewMatrix.m01 = UP_VECTOR.z * viewMatrix.m20 - UP_VECTOR.x * viewMatrix.m22;
        viewMatrix.m02 = UP_VECTOR.x * viewMatrix.m21 - UP_VECTOR.y * viewMatrix.m20;
        s = 1 / (float) Math.sqrt(viewMatrix.m00 * viewMatrix.m00 + viewMatrix.m01 * viewMatrix.m01 + viewMatrix.m02 * viewMatrix.m02);
        viewMatrix.m00 = viewMatrix.m00 * s;
        viewMatrix.m01 = viewMatrix.m01 * s;
        viewMatrix.m02 = viewMatrix.m02 * s;
        viewMatrix.m03 = -cameraPosition.x * viewMatrix.m00 - cameraPosition.y * viewMatrix.m01 - cameraPosition.z * viewMatrix.m02;
        viewMatrix.m10 = viewMatrix.m01 * viewMatrix.m22 - viewMatrix.m02 * viewMatrix.m21;
        viewMatrix.m11 = viewMatrix.m02 * viewMatrix.m20 - viewMatrix.m00 * viewMatrix.m22;
        viewMatrix.m12 = viewMatrix.m00 * viewMatrix.m21 - viewMatrix.m01 * viewMatrix.m20;
        s = (float) (1.0 / Math.sqrt(viewMatrix.m10 * viewMatrix.m10 + viewMatrix.m11 * viewMatrix.m11 + viewMatrix.m12 * viewMatrix.m12));
        viewMatrix.m10 = viewMatrix.m10 * s;
        viewMatrix.m11 = viewMatrix.m11 * s;
        viewMatrix.m12 = viewMatrix.m12 * s;
        viewMatrix.m13 = -cameraPosition.x * viewMatrix.m10 - cameraPosition.y * viewMatrix.m11 - cameraPosition.z * viewMatrix.m12;
        topLeft.x = viewMatrix.m20 * projMatrix.m11 - viewMatrix.m10 + viewMatrix.m00 * aspectRatio;
        topLeft.y = viewMatrix.m21 * projMatrix.m11 - viewMatrix.m11 + viewMatrix.m01 * aspectRatio;
        topLeft.z = viewMatrix.m22 * projMatrix.m11 - viewMatrix.m12 + viewMatrix.m02 * aspectRatio;
        bottomRight.x = viewMatrix.m20 * projMatrix.m11 + viewMatrix.m10 - viewMatrix.m00 * aspectRatio;
        bottomRight.y = viewMatrix.m21 * projMatrix.m11 + viewMatrix.m11 - viewMatrix.m01 * aspectRatio;
        bottomRight.z = viewMatrix.m22 * projMatrix.m11 + viewMatrix.m12 - viewMatrix.m02 * aspectRatio;
        return viewMatrix;
    }
}
