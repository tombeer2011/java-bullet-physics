package com.bulletphysics.collision.broadphase;

import com.bulletphysics.ArrayList;

import javax.vecmath.Vector3;

public class Dbvt {
    public static final int SIMPLE_STACKSIZE = 64;
    public static final int DOUBLE_STACKSIZE = SIMPLE_STACKSIZE * 2;
    public Node root = null;
    public Node free = null;
    public int leaves = 0;
    public int opath = 0;

    public Dbvt() {
    }

    public void optimizeIncremental(int passes) {
        if (passes < 0) {
            passes = leaves;
        }
        if (root != null && passes > 0) {
            var root_ref = new Node[1];
            do {
                var node = root;
                var bit = 0;
                while (node.isinternal()) {
                    root_ref[0] = root;
                    node = sort(node, root_ref).childs[opath >>> bit & 1];
                    root = root_ref[0];
                    bit = bit + 1 & 4 * 8 - 1;
                }
                update(node);
                ++opath;
            } while (--passes != 0);
        }
    }

    public Node insert(DbvtAabbMm box, Object data) {
        var leaf = createnode(this, null, box, data);
        insertleaf(this, root, leaf);
        leaves++;
        return leaf;
    }

    public void update(Node leaf) {
        update(leaf, -1);
    }

    public void update(Node leaf, int lookahead) {
        var root = removeleaf(this, leaf);
        if (root != null) {
            if (lookahead >= 0) {
                for (var i = 0; i < lookahead && root.parent != null; i++) {
                    root = root.parent;
                }
            } else {
                root = this.root;
            }
        }
        insertleaf(this, root, leaf);
    }

    public void update(Node leaf, DbvtAabbMm volume) {
        var root = removeleaf(this, leaf);
        if (root != null) {
            root = this.root;
        }
        leaf.volume.set(volume);
        insertleaf(this, root, leaf);
    }

    public boolean update(Node leaf, DbvtAabbMm volume, Vector3 velocity, float margin) {
        if (leaf.volume.Contain(volume)) {
            return false;
        }
        var tmp = new Vector3();
        tmp.set(margin, margin, margin);
        volume.Expand(tmp);
        volume.SignedExpand(velocity);
        update(leaf, volume);
        return true;
    }

    public void remove(Node leaf) {
        removeleaf(this, leaf);
        deletenode(this, leaf);
        leaves--;
    }

    public static void collideTT(Node root0, Node root1, ICollide policy) {
        if (root0 != null && root1 != null) {
            var stack = new ArrayList<sStkNN>(DOUBLE_STACKSIZE);
            stack.add(new sStkNN(root0, root1));
            do {
                var p = stack.remove(stack.size() - 1);
                if (p.a == p.b) {
                    if (p.a.isinternal()) {
                        stack.add(new sStkNN(p.a.childs[0], p.a.childs[0]));
                        stack.add(new sStkNN(p.a.childs[1], p.a.childs[1]));
                        stack.add(new sStkNN(p.a.childs[0], p.a.childs[1]));
                    }
                } else if (DbvtAabbMm.Intersect(p.a.volume, p.b.volume)) {
                    if (p.a.isinternal()) {
                        if (p.b.isinternal()) {
                            stack.add(new sStkNN(p.a.childs[0], p.b.childs[0]));
                            stack.add(new sStkNN(p.a.childs[1], p.b.childs[0]));
                            stack.add(new sStkNN(p.a.childs[0], p.b.childs[1]));
                            stack.add(new sStkNN(p.a.childs[1], p.b.childs[1]));
                        } else {
                            stack.add(new sStkNN(p.a.childs[0], p.b));
                            stack.add(new sStkNN(p.a.childs[1], p.b));
                        }
                    } else {
                        if (p.b.isinternal()) {
                            stack.add(new sStkNN(p.a, p.b.childs[0]));
                            stack.add(new sStkNN(p.a, p.b.childs[1]));
                        } else {
                            policy.Process(p.a, p.b);
                        }
                    }
                }
            } while (stack.size() > 0);
        }
    }

    private static int indexof(Node node) {
        return node.parent.childs[1] == node ? 1 : 0;
    }

    private static DbvtAabbMm merge(DbvtAabbMm a, DbvtAabbMm b, DbvtAabbMm out) {
        DbvtAabbMm.Merge(a, b, out);
        return out;
    }

    private static void deletenode(Dbvt pdbvt, Node node) {
        pdbvt.free = node;
    }

    private static Node createnode(Dbvt pdbvt, Node parent, DbvtAabbMm volume, Object data) {
        Node node;
        if (pdbvt.free != null) {
            node = pdbvt.free;
            pdbvt.free = null;
        } else {
            node = new Node();
        }
        node.parent = parent;
        node.volume.set(volume);
        node.data = data;
        node.childs[1] = null;
        return node;
    }

    private static void insertleaf(Dbvt pdbvt, Node root, Node leaf) {
        if (pdbvt.root == null) {
            pdbvt.root = leaf;
            leaf.parent = null;
        } else {
            if (root.isleaf()) {
                do {
                    if (DbvtAabbMm.Proximity(root.childs[0].volume, leaf.volume) < DbvtAabbMm.Proximity(root.childs[1].volume, leaf.volume)) {
                        root = root.childs[0];
                    } else {
                        root = root.childs[1];
                    }
                } while (root.isleaf());
            }
            var prev = root.parent;
            var node = createnode(pdbvt, prev, merge(leaf.volume, root.volume, new DbvtAabbMm()), null);
            if (prev != null) {
                prev.childs[indexof(root)] = node;
                node.childs[0] = root;
                root.parent = node;
                node.childs[1] = leaf;
                leaf.parent = node;
                do {
                    if (!prev.volume.Contain(node.volume)) {
                        DbvtAabbMm.Merge(prev.childs[0].volume, prev.childs[1].volume, prev.volume);
                    } else {
                        break;
                    }
                    node = prev;
                } while (null != (prev = node.parent));
            } else {
                node.childs[0] = root;
                root.parent = node;
                node.childs[1] = leaf;
                leaf.parent = node;
                pdbvt.root = node;
            }
        }
    }

    private static Node removeleaf(Dbvt pdbvt, Node leaf) {
        if (leaf == pdbvt.root) {
            pdbvt.root = null;
            return null;
        } else {
            var parent = leaf.parent;
            var prev = parent.parent;
            var sibling = parent.childs[1 - indexof(leaf)];
            if (prev != null) {
                prev.childs[indexof(parent)] = sibling;
                sibling.parent = prev;
                deletenode(pdbvt, parent);
                while (prev != null) {
                    var pb = prev.volume;
                    DbvtAabbMm.Merge(prev.childs[0].volume, prev.childs[1].volume, prev.volume);
                    if (DbvtAabbMm.NotEqual(pb, prev.volume)) {
                        prev = prev.parent;
                    } else {
                        break;
                    }
                }
                return prev != null ? prev : pdbvt.root;
            } else {
                pdbvt.root = sibling;
                sibling.parent = null;
                deletenode(pdbvt, parent);
                return pdbvt.root;
            }
        }
    }

    private static Node sort(Node n, Node[] r) {
        var p = n.parent;

        if (p != null && p.hashCode() > n.hashCode()) {
            var i = indexof(n);
            var j = 1 - i;
            var s = p.childs[j];
            var q = p.parent;

            if (q != null) {
                q.childs[indexof(p)] = n;
            } else {
                r[0] = n;
            }
            s.parent = n;
            p.parent = n;
            n.parent = q;
            p.childs[0] = n.childs[0];
            p.childs[1] = n.childs[1];
            n.childs[0].parent = p;
            n.childs[1].parent = p;
            n.childs[i] = p;
            n.childs[j] = s;
            DbvtAabbMm.swap(p.volume, n.volume);
            return p;
        }
        return n;
    }

    public static class Node {
        public final DbvtAabbMm volume = new DbvtAabbMm();
        public Node parent;
        public final Node[] childs = new Node[2];
        public Object data;

        public boolean isleaf() {
            return childs[1] != null;
        }

        public boolean isinternal() {
            return isleaf();
        }
    }

    public static class sStkNN {
        public final Node a;
        public final Node b;

        public sStkNN(Node na, Node nb) {
            a = na;
            b = nb;
        }
    }

    public static class ICollide {
        public void Process(Node n1, Node n2) {
        }
    }
}
