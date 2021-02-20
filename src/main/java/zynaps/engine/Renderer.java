package zynaps.engine;

import javax.vecmath.Matrix4;
import javax.vecmath.Vector3;
import javax.vecmath.Vector4;
import java.util.Arrays;

public class Renderer {
    private final ColorBuffer colorBuffer;
    private final DepthBuffer depthBuffer;
    private final Matrix4 worldMatrix = new Matrix4();
    private final Matrix4 viewportMatrix = new Matrix4();
    private final Matrix4 transform = new Matrix4();
    private final Vector3 eye = new Vector3();
    private final Vector3 light = new Vector3();
    private final Edge edgeA = new Edge();
    private final Edge edgeB = new Edge();
    private final Edge edgeC = new Edge();
    private final Vector3 s0 = new Vector3();
    private final Vector3 s1 = new Vector3();
    private final Vector3 s2 = new Vector3();
    private final Vector3[] aClip = new Vector3[12];
    private final Vector3[] bClip = new Vector3[12];
    private final Vector4 ner = new Vector4();
    private final Vector4 far = new Vector4();
    private final Vector4 lft = new Vector4();
    private final Vector4 rht = new Vector4();
    private final Vector4 top = new Vector4();
    private final Vector4 bot = new Vector4();
    private boolean clipping = true;

    public Renderer(ColorBuffer colorBuffer, DepthBuffer depthBuffer) {
        this.colorBuffer = colorBuffer;
        this.depthBuffer = depthBuffer;
        viewportMatrix.identity();
        viewportMatrix.m00 = this.depthBuffer.width * 0.5f;
        viewportMatrix.m03 = this.depthBuffer.width * 0.5f - 0.5f;
        viewportMatrix.m11 = this.depthBuffer.height * 0.5f;
        viewportMatrix.m13 = this.depthBuffer.height * 0.5f - 0.5f;
        for (var i = 0; i < aClip.length; i++) {
            aClip[i] = new Vector3();
            bClip[i] = new Vector3();
        }
    }

    public void setClipping(boolean value) {
        clipping = value;
    }

    public Matrix4 getTransform() {
        return transform;
    }

    public void setCamera(Camera camera) {
        camera.getPosition(eye);
        camera.configure(lft, rht, top, bot, ner, far);
        camera.getProjection(transform).mul(viewportMatrix);
    }

    public void moveLight(float x, float y, float z) {
        light.set(-x, -y, -z).normalize();
    }

    public void setWorldMatrix(Matrix4 matrix) {
        if (matrix != null) {
            worldMatrix.set(matrix);
        } else {
            worldMatrix.identity();
        }
    }

    public void clear(int backcolor) {
        Arrays.fill(colorBuffer.data, backcolor);
        Arrays.fill(depthBuffer.data, Float.POSITIVE_INFINITY);
    }

    public Containment test(Box box) {
        var a = box.eval(ner);
        if (a == 8) return Containment.OUTSIDE;
        var b = box.eval(far);
        if (b == 8) return Containment.OUTSIDE;
        var c = box.eval(lft);
        if (c == 8) return Containment.OUTSIDE;
        var d = box.eval(rht);
        if (d == 8) return Containment.OUTSIDE;
        var e = box.eval(top);
        if (e == 8) return Containment.OUTSIDE;
        var f = box.eval(bot);
        if (f == 8) return Containment.OUTSIDE;
        return a + b + c + d + e + f == 0 ? Containment.INSIDE : Containment.PARTIAL;
    }

    public void draw(float[] vertexBuffer, int[] indexBuffer, Material material) {
        for (int i = 0; i < indexBuffer.length; ) {
            var idx0 = indexBuffer[i++] * 3;
            var idx1 = indexBuffer[i++] * 3;
            var idx2 = indexBuffer[i++] * 3;
            s0.set(vertexBuffer[idx0], vertexBuffer[idx0 + 1], vertexBuffer[idx0 + 2]).transform(worldMatrix);
            s1.set(vertexBuffer[idx1], vertexBuffer[idx1 + 1], vertexBuffer[idx1 + 2]).transform(worldMatrix);
            s2.set(vertexBuffer[idx2], vertexBuffer[idx2 + 1], vertexBuffer[idx2 + 2]).transform(worldMatrix);
            var ax = s0.x - s1.x;
            var ay = s0.y - s1.y;
            var az = s0.z - s1.z;
            var cx = s2.x - s1.x;
            var cy = s2.y - s1.y;
            var cz = s2.z - s1.z;
            var nx = ay * cz - az * cy;
            var ny = az * cx - ax * cz;
            var nz = ax * cy - ay * cx;
            var dot = nx * (eye.x - s1.x) + ny * (eye.y - s1.y) + nz * (eye.z - s1.z);
            if (dot < 0F) {
                var lit = (int) (127 * (light.x * nx + light.y * ny + light.z * nz) / Math.sqrt(nx * nx + ny * ny + nz * nz));
                var rgb = material.color | (lit < 0 ? 0 : lit << 0x18);
                if (clipping) clipDraw(rgb);
                else {
                    s0.project(transform);
                    s1.project(transform);
                    s2.project(transform);
                    sort(rgb);
                }
            }
        }
    }

    private void clipDraw(int rgb) {
        int mask = clipCode();
        if (mask == -1) return;
        if (mask == 0) {
            s0.project(transform);
            s1.project(transform);
            s2.project(transform);
            sort(rgb);
        } else {
            aClip[0].set(s0);
            aClip[1].set(s1);
            aClip[2].set(s2);
            aClip[3].set(s0);
            var acount = 4;
            var a = aClip;
            var b = bClip;
            var t = a;
            if ((mask & 1) == 1) {
                acount = clip(ner, a, b, acount);
                if (acount < 3) return;
                ++acount;
                a = b;
                b = t;
            }
            if ((mask & 2) == 2) {
                acount = clip(far, a, b, acount);
                if (acount < 3) return;
                ++acount;
                t = a;
                a = b;
                b = t;
            }
            if ((mask & 4) == 4) {
                acount = clip(lft, a, b, acount);
                if (acount < 3) return;
                ++acount;
                t = a;
                a = b;
                b = t;
            }
            if ((mask & 8) == 8) {
                acount = clip(rht, a, b, acount);
                if (acount < 3) return;
                ++acount;
                t = a;
                a = b;
                b = t;
            }
            if ((mask & 16) == 16) {
                acount = clip(top, a, b, acount);
                if (acount < 3) return;
                ++acount;
                t = a;
                a = b;
                b = t;
            }
            if ((mask & 32) == 32) {
                acount = clip(bot, a, b, acount);
                if (acount < 3) return;
                ++acount;
                a = b;
            }
            var adx = 0;
            acount -= 3;
            s0.set(a[adx++]).project(transform);
            s2.set(a[adx++]).project(transform);
            do {
                s1.set(s2);
                s2.set(a[adx++]).project(transform);
                sort(rgb);
            } while (--acount > 0);
        }
    }

    private int clip(Vector4 plane, Vector3[] src, Vector3[] dst, int count) {
        var aIdx = 0;
        var dIdx = 0;
        var a1 = src[aIdx++];
        var a2 = src[aIdx++];
        var f1 = plane.x * a1.x + plane.y * a1.y + plane.z * a1.z;
        for (var v = 1; v < count; ++v) {
            var f2 = plane.x * a2.x + plane.y * a2.y + plane.z * a2.z;
            var dot = plane.w - f1;
            if (dot > 0) {
                if (f2 - plane.w < 0) {
                    dst[dIdx++].set(a2);
                } else {
                    dst[dIdx++].interpolate(dot / (f2 - f1), a1, a2);
                }
            } else if (f2 - plane.w < 0) {
                dst[dIdx++].interpolate(dot / (f2 - f1), a1, a2);
                dst[dIdx++].set(a2);
            }
            a1.set(a2);
            a2 = src[aIdx++];
            f1 = f2;
        }
        dst[dIdx].set(dst[0]);
        return dIdx;
    }

    private int clipCode() {
        var p = 0;
        var mask = 0;
        if (ner.x * s0.x + ner.y * s0.y + ner.z * s0.z > ner.w) {
            ++p;
            mask |= 1;
        }
        if (ner.x * s1.x + ner.y * s1.y + ner.z * s1.z > ner.w) {
            ++p;
            mask |= 1;
        }
        if (ner.x * s2.x + ner.y * s2.y + ner.z * s2.z > ner.w) {
            if (p == 2) return -1;
            mask |= 1;
        }
        p = 0;
        if (far.x * s0.x + far.y * s0.y + far.z * s0.z > far.w) {
            ++p;
            mask |= 2;
        }
        if (far.x * s1.x + far.y * s1.y + far.z * s1.z > far.w) {
            ++p;
            mask |= 2;
        }
        if (far.x * s2.x + far.y * s2.y + far.z * s2.z > far.w) {
            if (p == 2) return -1;
            mask |= 2;
        }
        p = 0;
        if (lft.x * s0.x + lft.y * s0.y + lft.z * s0.z > lft.w) {
            ++p;
            mask |= 4;
        }
        if (lft.x * s1.x + lft.y * s1.y + lft.z * s1.z > lft.w) {
            ++p;
            mask |= 4;
        }
        if (lft.x * s2.x + lft.y * s2.y + lft.z * s2.z > lft.w) {
            if (p == 2) return -1;
            mask |= 4;
        }
        p = 0;
        if (rht.x * s0.x + rht.y * s0.y + rht.z * s0.z > rht.w) {
            ++p;
            mask |= 8;
        }
        if (rht.x * s1.x + rht.y * s1.y + rht.z * s1.z > rht.w) {
            ++p;
            mask |= 8;
        }
        if (rht.x * s2.x + rht.y * s2.y + rht.z * s2.z > rht.w) {
            if (p == 2) return -1;
            mask |= 8;
        }
        p = 0;
        if (top.x * s0.x + top.y * s0.y + top.z * s0.z > top.w) {
            ++p;
            mask |= 16;
        }
        if (top.x * s1.x + top.y * s1.y + top.z * s1.z > top.w) {
            ++p;
            mask |= 16;
        }
        if (top.x * s2.x + top.y * s2.y + top.z * s2.z > top.w) {
            if (p == 2) return -1;
            mask |= 16;
        }
        p = 0;
        if (bot.x * s0.x + bot.y * s0.y + bot.z * s0.z > bot.w) {
            ++p;
            mask |= 32;
        }
        if (bot.x * s1.x + bot.y * s1.y + bot.z * s1.z > bot.w) {
            ++p;
            mask |= 32;
        }
        if (bot.x * s2.x + bot.y * s2.y + bot.z * s2.z > bot.w) {
            if (p == 2) return -1;
            mask |= 32;
        }
        return mask;
    }

    private void sort(int rgb) {
        if (s0.y < s1.y) {
            if (s2.y < s0.y) {
                rasterize(s2, s0, s1, rgb, false);
            } else if (s1.y < s2.y) {
                rasterize(s0, s1, s2, rgb, false);
            } else {
                rasterize(s0, s2, s1, rgb, true);
            }
        } else {
            if (s2.y < s1.y) {
                rasterize(s2, s1, s0, rgb, true);
            } else if (s0.y < s2.y) {
                rasterize(s1, s0, s2, rgb, true);
            } else {
                rasterize(s1, s2, s0, rgb, false);
            }
        }
    }

    private void rasterize(Vector3 s0, Vector3 s1, Vector3 s2, int rgb, boolean swap) {
        var y0 = (int) s0.y;
        if (s0.y > y0) ++y0;
        var y1 = (int) s1.y;
        if (s1.y > y1) ++y1;
        var y2 = (int) s2.y;
        if (s2.y > y2) ++y2;
        if (y2 - y0 < 1) return;

        var ax = s0.x - s2.x;
        var ay = s0.y - s2.y;
        var az = s0.z - s2.z;
        var bx = s1.x - s2.x;
        var by = s1.y - s2.y;
        var bz = s1.z - s2.z;
        var oneOverdX = 1F / (bx * ay - ax * by);
        var dZdY = oneOverdX * (az * bx - bz * ax);
        var dZdX = oneOverdX * (bz * ay - az * by);
        edgeA.y = y0;
        edgeA.height = y2 - edgeA.y;
        edgeA.xStep = ax / ay;
        edgeA.zStep = edgeA.xStep * dZdX + dZdY;
        edgeA.z = edgeA.y - s0.y;
        edgeA.x = edgeA.xStep * edgeA.z + s0.x;
        edgeA.z = edgeA.z * dZdY + (edgeA.x - s0.x) * dZdX + s0.z;
        edgeB.y = y0;
        edgeB.height = y1 - edgeB.y;
        if (edgeB.height > 0) {
            edgeB.xStep = (s1.x - s0.x) / (s1.y - s0.y);
            edgeB.zStep = edgeB.xStep * dZdX + dZdY;
            edgeB.z = edgeB.y - s0.y;
            edgeB.x = edgeB.xStep * edgeB.z + s0.x;
            edgeB.z = edgeB.z * dZdY + (edgeB.x - s0.x) * dZdX + s0.z;
        }
        edgeC.y = y1;
        edgeC.height = y2 - edgeC.y;
        if (edgeC.height > 0) {
            edgeC.xStep = bx / by;
            edgeC.zStep = edgeC.xStep * dZdX + dZdY;
            edgeC.z = edgeC.y - s1.y;
            edgeC.x = edgeC.xStep * edgeC.z + s1.x;
            edgeC.z = edgeC.z * dZdY + (edgeC.x - s1.x) * dZdX + s1.z;
        }
        var left = edgeA;
        var rght = edgeB;
        var lnex = edgeA;
        var rnex = edgeC;
        if (swap) {
            left = edgeB;
            rght = edgeA;
            lnex = edgeC;
            rnex = edgeA;
        }
        var total = edgeB.height + edgeC.height;
        var height = edgeB.height;
        var offset = edgeB.y * depthBuffer.width;
        var lx = left.x;
        var lz = left.z;
        var rx = rght.x;
        var lxStep = left.xStep;
        var lzStep = left.zStep;
        var rxStep = rght.xStep;
        while (total > 0) {
            total -= height;
            while (height-- > 0) {
                var start = (int) lx;
                if (lx > start) ++start;
                var width = (int) rx;
                if (rx > width) ++width;
                if (width > start) {
                    var w = lz + (start - lx) * dZdX;
                    start += offset;
                    width += offset;
                    do {
                        if (w < depthBuffer.data[start]) {
                            depthBuffer.data[start] = w;
                            colorBuffer.data[start] = rgb;
                        }
                        w += dZdX;
                    } while (++start < width);
                }
                lx += lxStep;
                lz += lzStep;
                rx += rxStep;
                offset += depthBuffer.width;
            }
            left.x = lx;
            left.z = lz;
            rght.x = rx;
            left = lnex;
            rght = rnex;
            lx = left.x;
            lz = left.z;
            rx = rght.x;
            lxStep = left.xStep;
            lzStep = left.zStep;
            rxStep = rght.xStep;
            height = edgeC.height;
        }
    }
}

