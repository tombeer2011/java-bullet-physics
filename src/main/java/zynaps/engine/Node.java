package zynaps.engine;

import javax.vecmath.Matrix4;

public class Node {
    public final static Node NULL = new Node();

    protected Node() {
    }

    public boolean parentIs(Node node) {
        return false;
    }

    public void attachToParent(Node node) {
    }

    public void childAttached(Node node) {
    }

    public void detachFromParent() {
    }

    public void childDetached(Node node) {
    }

    public void setLocalTransform(Matrix4 transform) {
    }

    public void traverseUp(NodeVisitor visitor) {
    }

    public void traverseDown(NodeVisitor visitor) {
    }

    public void update() {
    }

    public void updateTransform() {
    }

    public void concatParentTransform(Matrix4 source, Matrix4 result) {
        result.set(source);
    }

    public void updateBounds() {
    }

    public void aggregateWorldBoundsInto(Box other) {
    }

    public void render(Renderer renderer) {
    }

    public Containment isContainedBy(Renderer container) {
        return Containment.INSIDE;
    }
}
