package com.bulletphysics;

public class MotionState {
    public final Transform graphicsWorldTrans = new Transform();
    public final Transform centerOfMassOffset = new Transform();
    public final Transform startWorldTrans = new Transform();

    public MotionState(Transform startTrans) {
        graphicsWorldTrans.set(startTrans);
        centerOfMassOffset.identity();
        startWorldTrans.set(startTrans);
    }

    public void setWorldTransform(Transform t) {
        graphicsWorldTrans.set(t);
        graphicsWorldTrans.mul(centerOfMassOffset);
    }

    public Transform getWorldTransform(Transform out) {
        out.inverse(centerOfMassOffset);
        out.mul(graphicsWorldTrans);
        return out;
    }
}
