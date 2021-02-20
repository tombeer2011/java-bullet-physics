package zynaps.engine;

import javax.vecmath.Matrix4;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class BranchNode extends Node {
    private final List<Node> childNodes = Collections.synchronizedList(new ArrayList<>());
    private final Matrix4 localTransform = new Matrix4().identity();
    private final Matrix4 worldTransform = new Matrix4().identity();
    private final Box worldBounds = new Box();
    private Node parent = NULL;

    public BranchNode() {
        this(null);
    }

    public BranchNode(Matrix4 transform) {
        localTransform.set(transform != null ? transform : new Matrix4().identity());
    }

    public BranchNode addChild(Node node) {
        if (!node.parentIs(this)) {
            node.attachToParent(this);
        }
        return this;
    }

    public void concatParentTransform(Matrix4 source, Matrix4 result) {
        result.mul(worldTransform, source);
    }

    public void updateBounds() {
        worldBounds.reset();
        for (var childNode : childNodes) {
            childNode.aggregateWorldBoundsInto(worldBounds);
        }
    }

    public void aggregateWorldBoundsInto(Box other) {
        worldBounds.aggregateInto(other);
    }

    public Containment isContainedBy(Renderer container) {
        return container.test(worldBounds);
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
        childNodes.add(node);
    }

    public void detachFromParent() {
        var oldParent = parent;
        parent = NULL;
        oldParent.childDetached(this);
    }

    public void childDetached(Node node) {
        if (node != null) {
            childNodes.remove(node);
        }
    }

    public void setLocalTransform(Matrix4 transform) {
        localTransform.set(transform);
    }

    public void traverseUp(NodeVisitor visitor) {
        childNodes.forEach(childNode -> childNode.traverseUp(visitor));
        visitor.visit(this);
    }

    public void traverseDown(NodeVisitor visitor) {
        if (visitor.visit(this)) {
            childNodes.forEach(node -> node.traverseDown(visitor));
        }
    }

    public void updateTransform() {
        parent.concatParentTransform(localTransform, worldTransform);
    }
}
