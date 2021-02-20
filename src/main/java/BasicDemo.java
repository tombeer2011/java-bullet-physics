import com.bulletphysics.MotionState;
import com.bulletphysics.Transform;
import com.bulletphysics.collision.broadphase.DbvtBroadPhase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import zynaps.engine.*;

import javax.swing.*;
import javax.vecmath.Matrix4;
import javax.vecmath.Quaternion;
import javax.vecmath.Vector3;
import java.awt.*;
import java.awt.event.*;
import java.util.Objects;

class BasicDemo extends JPanel implements Game {
    private static final int SHADOW_SIZE = 1024;
    private static final int ARRAY_SIZE_X = 5;
    private static final int ARRAY_SIZE_Y = 5;
    private static final int ARRAY_SIZE_Z = 5;
    private static final int START_POS_X = -5;
    private static final int START_POS_Y = -5;
    private static final int START_POS_Z = -3;
    private DynamicsWorld dynamicsWorld = null;
    private CollisionShape shootBoxShape = null;
    private GameLoop gameLoop;
    private FirstPersonControl fpsCamera;
    private Model cubeMesh3;
    private Visualizer visualizer;

    public BasicDemo() {
        var displayMode = GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice().getDisplayMode();
        int width = (int) (displayMode.getWidth() * 0.9);
        int height = (int) (displayMode.getHeight() * 0.85);
        setPreferredSize(new Dimension(width, height));
    }

    public void initialise() {
        initialiseGraphics();
        initialiseScene();
        initialiseEventHandlers();
    }

    private void initialiseEventHandlers() {
        addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                fpsCamera.onMouseDown();
            }

            public void mouseReleased(MouseEvent e) {
                fpsCamera.onMouseUp();
            }
        });

        addMouseMotionListener(new MouseMotionAdapter() {
            public void mouseMoved(MouseEvent e) {
                fpsCamera.onMouseMove(e.getX(), e.getY());
            }

            public void mouseDragged(MouseEvent e) {
                fpsCamera.onMouseMove(e.getX(), e.getY());
            }
        });

        addKeyListener(new KeyAdapter() {
            public void keyTyped(KeyEvent e) {
                if (e.getKeyChar() == '.') {
                    shootBox(visualizer.getCamera().getTarget(new Vector3()));
                }
            }

            public void keyPressed(KeyEvent e) {
                fpsCamera.onKeyDown(e.getKeyCode());
            }

            public void keyReleased(KeyEvent e) {
                fpsCamera.keyUp(e.getKeyCode());
            }
        });
    }

    private void initialiseGraphics() {
        visualizer = new Visualizer(getWidth(), getHeight(), SHADOW_SIZE);
        visualizer.moveLight(-70, 130, -80);

        fpsCamera = new FirstPersonControl(visualizer.getCamera());
        gameLoop = new GameLoop(this);
    }

    private void initialiseScene() {
        dynamicsWorld = new DiscreteDynamicsWorld(new CollisionDispatcher(), new DbvtBroadPhase(), new SequentialImpulseConstraintSolver());
        dynamicsWorld.setGravity(new Vector3(0f, -10f, 0f));

        var groundShape = new BoxShape(new Vector3(50f, 50f, 50f));
        var planeMesh = Primitives.buildPlane(new Material(0.3f, 0.2f, 0.4f));
        var cubeMesh = Primitives.buildCube(new Material(0.92f, 0.61f, 0.04f));
        var cubeMesh2 = Primitives.buildCube(new Material(0.04f, 0.61f, 0.94f));
        cubeMesh3 = Primitives.buildCube(new Material(0.94f, 0.01f, 0.04f));

        var root = visualizer.getRootNode();
        var groundTransform = new Transform().identity();
        groundTransform.origin.set(0, -56, 0);
        var localInertia = new Vector3();
        var myMotionState = new MotionState(groundTransform);
        var rbInfo = new RigidBodyConstructionInfo(0f, myMotionState, groundShape, localInertia);
        var body = new RigidBody(rbInfo);
        dynamicsWorld.addRigidBody(body);

        var transform = Matrix4.createTransform(new Vector3(50, 50, 50), new Vector3(), new Vector3(0, -6, 0));
        root.addChild(new LeafNode(planeMesh, transform));

        var colShape = new BoxShape(new Vector3(1, 1, 1));
        var startTransform = new Transform().identity();
        localInertia = new Vector3();
        colShape.calculateLocalInertia(1f, localInertia);
        var start_x = START_POS_X - ARRAY_SIZE_X / 2.0f;
        var start_z = START_POS_Z - ARRAY_SIZE_Z / 2.0f;
        var swap = true;
        for (var k = 0; k < ARRAY_SIZE_Y; k++) {
            for (var i = 0; i < ARRAY_SIZE_X; i++) {
                for (var j = 0; j < ARRAY_SIZE_Z; j++) {
                    startTransform.origin.set(2f * i + start_x, 10f + 2f * k + (float) START_POS_Y, 2f * j + start_z);
                    myMotionState = new MotionState(startTransform);
                    rbInfo = new RigidBodyConstructionInfo(1f, myMotionState, colShape, localInertia);
                    body = new RigidBody(rbInfo);
                    body.setActivationState(RigidBody.ISLAND_SLEEPING);
                    dynamicsWorld.addRigidBody(body);
                    body.setActivationState(RigidBody.ISLAND_SLEEPING);
                    if ((swap = !swap)) {
                        root.addChild(new PhysicsNode(body).addChild(new LeafNode(cubeMesh)));
                    } else {
                        root.addChild(new PhysicsNode(body).addChild(new LeafNode(cubeMesh2)));
                    }
                }
            }
        }
        clientResetScene();
    }

    public void clientResetScene() {
        int numObjects;
        if (dynamicsWorld != null) {
            dynamicsWorld.stepSimulation(1f / 60f, 0);
            numObjects = dynamicsWorld.getNumCollisionObjects();
        } else {
            numObjects = 0;
        }
        for (var i = 0; i < numObjects; i++) {
            var colObj = dynamicsWorld.getCollisionObjectArray().get(i);
            var body = RigidBody.upcast(colObj);
            if (body != null) {
                if (body.getMotionState() != null) {
                    var myMotionState = (MotionState) body.getMotionState();
                    myMotionState.graphicsWorldTrans.set(myMotionState.startWorldTrans);
                    colObj.setWorldTransform(myMotionState.graphicsWorldTrans);
                    colObj.setInterpolationWorldTransform(myMotionState.startWorldTrans);
                    colObj.activate();
                }
                dynamicsWorld.getBroadphase().getOverlappingPairCache().cleanProxyFromPairs(colObj.getBroadphaseHandle(), dynamicsWorld.getDispatcher());
                if (!body.isStaticObject()) {
                    Objects.requireNonNull(RigidBody.upcast(colObj)).setLinearVelocity(new Vector3());
                    Objects.requireNonNull(RigidBody.upcast(colObj)).setAngularVelocity(new Vector3());
                }
            }
        }
    }

    public void shootBox(Vector3 destination) {
        if (dynamicsWorld != null) {
            var mass = 10f;
            var startTransform = new Transform();
            startTransform.identity();
            var position = visualizer.getCamera().getPosition(new Vector3());
            var camPos = new Vector3(position.x, position.y, position.z);
            startTransform.origin.set(camPos);
            if (shootBoxShape == null) {
                shootBoxShape = new BoxShape(new Vector3(1f, 1f, 1f));
            }
            var body = localCreateRigidBody(mass, startTransform, shootBoxShape);
            visualizer.getRootNode().addChild(new PhysicsNode(body).addChild(new LeafNode(cubeMesh3)));
            var linVel = new Vector3(destination.x - camPos.x, destination.y - camPos.y, destination.z - camPos.z);
            linVel.normalize();
            var shootBoxInitialSpeed = 40f;
            linVel.mul(shootBoxInitialSpeed);
            var worldTrans = body.getWorldTransform(new Transform());
            worldTrans.origin.set(camPos);
            worldTrans.setRotation(new Quaternion(0f, 0f, 0f, 1f));
            body.setWorldTransform(worldTrans);
            body.setLinearVelocity(new Vector3(linVel.x, linVel.y, linVel.z));
            body.setAngularVelocity(new Vector3());
            body.setCcdMotionThreshold(1f);
            body.setCcdSweptSphereRadius(0.2f);
        }
    }

    public RigidBody localCreateRigidBody(float mass, Transform startTransform, CollisionShape shape) {
        var isDynamic = mass != 0f;
        var localInertia = new Vector3();
        if (isDynamic) shape.calculateLocalInertia(mass, localInertia);
        var myMotionState = new MotionState(startTransform);
        var body = new RigidBody(new RigidBodyConstructionInfo(mass, myMotionState, shape, localInertia));
        dynamicsWorld.addRigidBody(body);
        return body;
    }

    public void start() {
        gameLoop.start();
    }

    public void stop() {
        gameLoop.stop();
    }

    @Override
    public void update(double seconds) {
        dynamicsWorld.stepSimulation((float) seconds);
        fpsCamera.update(seconds, 20);
        visualizer.update();
    }

    @Override
    public void render() {
        visualizer.render();
        visualizer.present(this);
    }
}
