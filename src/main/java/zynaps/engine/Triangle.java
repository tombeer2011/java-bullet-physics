package zynaps.engine;

class Triangle implements MaterialModifier {
    public final int a;
    public final int b;
    public final int c;
    public Material material = Material.WHITE;

    public Triangle(int v0, int v1, int v2) {
        a = v0;
        b = v1;
        c = v2;
    }

    public void changeColor(Material material) {
        this.material = material;
    }
}
