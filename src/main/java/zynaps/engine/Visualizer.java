package zynaps.engine;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ForkJoinPool;

public class Visualizer {
    private final BufferedImage image;
    private final Renderer renderer;
    private final Renderer shadowRenderer;
    private final Compositor compositor;
    private final List<Callable<Void>> tasks = new ArrayList<>();
    private final Camera shadowCamera;
    private final BranchNode rootNode;
    private final Scene scene;
    private final Camera camera;

    public Visualizer(int width, int height, int shadowSize) {
        var gc = GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice().getDefaultConfiguration();
        image = gc.createCompatibleImage(width, height, Transparency.OPAQUE);
        var pixelData = ((DataBufferInt) image.getRaster().getDataBuffer()).getData();

        var colorBuffer = new ColorBuffer(width, height, pixelData);
        var depthBuffer = new DepthBuffer(width, height);

        renderer = new Renderer(colorBuffer, depthBuffer);

        var shadowColorBuffer = new ColorBuffer(shadowSize, shadowSize);
        var shadowDepthBuffer = new DepthBuffer(shadowSize, shadowSize);
        shadowRenderer = new Renderer(shadowColorBuffer, shadowDepthBuffer);

        compositor = new Compositor(colorBuffer, depthBuffer, shadowDepthBuffer);
        camera = new Camera((float) Math.PI / 4.0f, width / (float) height, 1, 10000.0f);
        camera.moveTo(0, 0, -60);
        camera.lookAt(0, 0, 0);

        shadowCamera = new Camera((float) Math.PI / 4.0f, 1, 1, 1000.0f);
        shadowCamera.moveTo(0, 1, 0);
        shadowCamera.lookAt(0, 0, 0);

        rootNode = new BranchNode();
        scene = new Scene(rootNode);

        tasks.add(() -> {
            shadowRenderer.clear(0);
            shadowRenderer.setCamera(shadowCamera);
            scene.render(shadowRenderer);
            return null;
        });

        tasks.add(() -> {
            renderer.clear(0x88FFAA);
            renderer.setCamera(camera);
            scene.render(renderer);
            return null;
        });
    }

    public Camera getCamera() {
        return camera;
    }

    public BranchNode getRootNode() {
        return rootNode;
    }

    public Image getImage() {
        return image;
    }

    public void moveLight(float x, float y, float z) {
        renderer.moveLight(x, y, z);
        shadowCamera.moveTo(x, y, z);
        shadowCamera.lookAt(0, 0, 0);
    }

    public void update() {
        scene.update();
    }

    public void render() {
        ForkJoinPool.commonPool().invokeAll(tasks);
        compositor.process(shadowRenderer.getTransform(), renderer.getTransform());
    }

    public void present(Component control) {
        var g = (Graphics2D) control.getGraphics();
        if (g != null) {
            g.drawImage(getImage(), 0, 0, control.getWidth(), control.getHeight(), null);
            g.dispose();
        }
    }
}
