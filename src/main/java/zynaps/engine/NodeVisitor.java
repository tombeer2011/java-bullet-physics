package zynaps.engine;

@FunctionalInterface
public interface NodeVisitor {
    boolean visit(Node node);
}
