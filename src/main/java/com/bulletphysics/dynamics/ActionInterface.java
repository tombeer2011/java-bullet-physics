package com.bulletphysics.dynamics;

import com.bulletphysics.collision.dispatch.CollisionWorld;

public abstract class ActionInterface {
    public abstract void updateAction(CollisionWorld collisionWorld, float deltaTimeStep);
}
