package com.bulletphysics.collision.narrowphase;

import javax.vecmath.Vector3;

public interface SimplexSolverInterface {
    void reset();

    void addVertex(Vector3 w, Vector3 p, Vector3 q);

    boolean closest(Vector3 v);

    boolean fullSimplex();

    boolean inSimplex(Vector3 w);

    void backup_closest(Vector3 v);

    void compute_points(Vector3 p1, Vector3 p2);

    int numVertices();
}
