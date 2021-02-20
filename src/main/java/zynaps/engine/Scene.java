package zynaps.engine;

public class Scene {
    public final Node root;

    public Scene(Node root) {
        this.root = root;
    }

    public void update() {
        root.traverseDown(node -> {
            node.update();
            node.updateTransform();
            return true;
        });
        root.traverseUp(node -> {
            node.updateBounds();
            return true;
        });
    }

    public void render(Renderer renderer) {
        root.traverseDown(node -> {
            var containment = node.isContainedBy(renderer);
            if (containment != Containment.OUTSIDE) {
                renderer.setClipping(containment == Containment.PARTIAL);
                node.render(renderer);
                return true;
            }
            return false;
        });
    }
}
