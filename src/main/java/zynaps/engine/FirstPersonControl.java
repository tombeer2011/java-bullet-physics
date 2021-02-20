package zynaps.engine;

import javax.vecmath.Matrix4;
import javax.vecmath.Vector3;
import java.awt.event.KeyEvent;

import static java.lang.Math.max;
import static java.lang.Math.min;

public final class FirstPersonControl {
    private final static int MOVE_FORWARD = 1;
    private final static int MOVE_BACKWARD = 2;
    private final static int MOVE_LEFT = 4;
    private final static int MOVE_RIGHT = 8;
    private final Camera camera;
    private final Vector3 lookAt = new Vector3();
    private final Matrix4 rotY = new Matrix4();
    private final Matrix4 rotX = new Matrix4();
    private final Vector3 temp = new Vector3();
    private final Vector3 side = new Vector3();
    private float yaw;
    private float pitch;
    private boolean dragging;
    private int movementMask;
    private int currMouseX;
    private int currMouseY;
    private int lastMouseX;
    private int lastMouseY;

    public FirstPersonControl(Camera camera) {
        this.camera = camera;
        this.camera.getTarget(lookAt).sub(this.camera.getPosition(temp)).normalize();
    }

    public void keyUp(int code) {
        if (code == KeyEvent.VK_W) movementMask ^= MOVE_FORWARD;
        else if (code == KeyEvent.VK_S) movementMask ^= MOVE_BACKWARD;
        else if (code == KeyEvent.VK_A) movementMask ^= MOVE_LEFT;
        else if (code == KeyEvent.VK_D) movementMask ^= MOVE_RIGHT;
    }

    public void onKeyDown(int code) {
        if (code == KeyEvent.VK_W) movementMask |= MOVE_FORWARD;
        else if (code == KeyEvent.VK_S) movementMask |= MOVE_BACKWARD;
        else if (code == KeyEvent.VK_A) movementMask |= MOVE_LEFT;
        else if (code == KeyEvent.VK_D) movementMask |= MOVE_RIGHT;
    }

    public void onMouseDown() {
        dragging = true;
    }

    public void onMouseMove(int x, int y) {
        currMouseX = x;
        currMouseY = y;
    }

    public void onMouseUp() {
        dragging = false;
    }

    public void update(double seconds, float speed) {
        var scaledSpeed = (float) (seconds * speed);
        if (dragging) {
            yaw = yaw + (currMouseX - lastMouseX) * scaledSpeed * 0.05f;
            pitch = pitch + (currMouseY - lastMouseY) * scaledSpeed * 0.05f;
            pitch = max(-1.55f, min(1.55f, pitch));
            rotX.mul(rotY.rotY(yaw), rotX.rotX(pitch));
            rotX.transform(lookAt.set(0, 0, 1), lookAt);
        }
        lastMouseX = currMouseX;
        lastMouseY = currMouseY;

        camera.getPosition(temp);
        side.set(0, 1, 0).cross(side, lookAt).normalize().mul(scaledSpeed);

        if ((movementMask & MOVE_FORWARD) == MOVE_FORWARD) {
            temp.scaleAdd(scaledSpeed, lookAt, temp);
        } else if ((movementMask & MOVE_BACKWARD) == MOVE_BACKWARD) {
            temp.scaleAdd(-scaledSpeed, lookAt, temp);
        }

        if ((movementMask & MOVE_LEFT) == MOVE_LEFT) {
            temp.sub(side);
        } else if ((movementMask & MOVE_RIGHT) == MOVE_RIGHT) {
            temp.add(side);
        }

        camera.moveTo(temp);
        camera.lookAt(temp.x + lookAt.x, temp.y + lookAt.y, temp.z + lookAt.z);
    }
}
