package zynaps.engine;

import javax.vecmath.Matrix4;

public class LeafNode extends Node {
    private final Geometry geometry;
    private final Matrix4 localTransform = new Matrix4();
    private final Matrix4 worldTransform = new Matrix4();
    private final Box worldBounds = new Box();
    private Node parent = NULL;

    public LeafNode(Geometry geometry) {
        this(geometry, null);
    }

    public LeafNode(Geometry geometry, Matrix4 transform) {
        this.geometry = geometry;
        localTransform.set(transform != null ? transform : new Matrix4().identity());
    }

    public boolean parentIs(Node node) {
        return parent == node;
    }

    public void attachToParent(Node node) {
        detachFromParent();
        parent = node;
        parent.childAttached(this);
    }

    public void childAttached(Node node) {
        throw new UnsupportedOperationException();
    }

    public void detachFromParent() {
        var oldParent = parent;
        parent = NULL;
        oldParent.childDetached(this);
    }

    public void childDetached(Node node) {
        throw new UnsupportedOperationException();
    }

    public void setLocalTransform(Matrix4 transform) {
        localTransform.set(transform);
    }

    public void traverseUp(NodeVisitor visitor) {
        visitor.visit(this);
    }

    public void traverseDown(NodeVisitor visitor) {
        visitor.visit(this);
    }

    public void updateTransform() {
        parent.concatParentTransform(localTransform, worldTransform);
    }

    public void concatParentTransform(Matrix4 source, Matrix4 result) {
        result.mul(worldTransform, source);
    }

    public void updateBounds() {
        worldBounds.reset();
        geometry.localBoundsToWorld(worldBounds, worldTransform);
    }

    public void aggregateWorldBoundsInto(Box other) {
        worldBounds.aggregateInto(other);
    }

    public void render(Renderer renderer) {
        renderer.setWorldMatrix(worldTransform);
        geometry.render(renderer);
    }

    public Containment isContainedBy(Renderer container) {
        return container.test(worldBounds);
    }
}
