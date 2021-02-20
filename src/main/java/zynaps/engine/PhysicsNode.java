package zynaps.engine;

import com.bulletphysics.Transform;
import com.bulletphysics.dynamics.RigidBody;

import javax.vecmath.Matrix4;

public class PhysicsNode extends BranchNode {
    private final RigidBody body;
    private final Transform transform = new Transform();
    private final Matrix4 matrix4f = new Matrix4();

    public PhysicsNode(RigidBody body) {
        this.body = body;
        matrix4f.identity();
    }

    @Override
    public void update() {
        if (body != null) {
            var motionState = body.getMotionState();
            if (motionState != null) {
                transform.set(motionState.graphicsWorldTrans);
            } else {
                body.getWorldTransform(transform);
            }
            transform.getMatrix(matrix4f);
        }
        setLocalTransform(matrix4f);
    }
}
