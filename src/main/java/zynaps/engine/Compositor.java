package zynaps.engine;

import javax.vecmath.Matrix4;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.Executors;
import java.util.concurrent.ForkJoinPool;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

final class Compositor {
    private final int[] specular = new int[128];
    private final List<Callable<Object>> tasks;
    private final Matrix4 shadow = new Matrix4();

    public Compositor(ColorBuffer colorBuffer, DepthBuffer depthBuffer, DepthBuffer shadowBuffer) {
        for (var i = 0; i < specular.length; i++) {
            var col = (int) (256.0 * Math.pow(i / 128.0, 11));
            specular[i] = col << 0x10 | col << 0x08 | col;
        }
        tasks = IntStream.range(0, colorBuffer.height).mapToObj(y -> Executors.callable(() -> {
            double e00 = shadow.m00;
            double e02 = shadow.m02;
            double e03 = shadow.m03;
            double e10 = shadow.m10;
            double e12 = shadow.m12;
            double e13 = shadow.m13;
            double e20 = shadow.m20;
            double e22 = shadow.m22;
            double e23 = shadow.m23;
            double e30 = shadow.m30;
            double e32 = shadow.m32;
            double e33 = shadow.m33;
            var e12y14 = shadow.m01 * y + e03;
            var e22y24 = shadow.m11 * y + e13;
            var e32y34 = shadow.m21 * y + e23;
            var e42y44 = shadow.m31 * y + e33;
            var mem1 = y * colorBuffer.width;
            var mem2 = mem1 + colorBuffer.width;
            while (mem1 < mem2) {
                double z = depthBuffer.data[mem1];
                var overW = 1 / (e42y44 + e32 * z);
                var tx = overW * (e12y14 + e02 * z);
                var ty = overW * (e22y24 + e12 * z);
                var tz = overW * (e32y34 + e22 * z) - 0.00005;
                var col = colorBuffer.data[mem1];
                var amb = 0x7f7f7f & col >> 1;
                if (tx > 1.0 && ty > 1.0 && tx < shadowBuffer.width - 1 && ty < shadowBuffer.height - 1) {
                    var smem1 = (int) ty * shadowBuffer.width + (int) tx - 1;
                    var smem0 = smem1 - shadowBuffer.width;
                    var smem2 = smem1 + shadowBuffer.width;
                    var acc = (tz < shadowBuffer.data[smem0] ? 1 : 0) + (tz < shadowBuffer.data[smem0 + 1] ? 1 : 0) + (tz < shadowBuffer.data[smem0 + 2] ? 1 : 0) + (tz < shadowBuffer.data[smem1] ? 1 : 0) + (tz < shadowBuffer.data[smem1 + 1] ? 1 : 0) + (tz < shadowBuffer.data[smem2] ? 1 : 0) + (tz < shadowBuffer.data[smem2 + 1] ? 1 : 0) + (tz < shadowBuffer.data[smem2 + 2] ? 1 : 0);
                    var lit = (0x00007f & col >> 0x18) * acc >> 3;
                    var dif = (col & 0xff00ff) * lit >> 0x7 & 0xff00ff | (col & 0x00ff00) * lit >> 0x7 & 0x00ff00;
                    var spe = specular[lit];
                    var red = (amb & 0xff0000) + (dif & 0xff0000) + (spe & 0xff0000);
                    var grn = (amb & 0x00ff00) + (dif & 0x00ff00) + (spe & 0x00ff00);
                    var blu = (amb & 0x0000ff) + (dif & 0x0000ff) + (spe & 0x0000ff);
                    colorBuffer.data[mem1] = Math.min(red, 0xff0000) | Math.min(grn, 0x00ff00) | Math.min(blu, 0x0000ff);
                } else {
                    colorBuffer.data[mem1] = amb;
                }
                e12y14 += e00;
                e22y24 += e10;
                e32y34 += e20;
                e42y44 += e30;
                ++mem1;
            }
        })).collect(Collectors.toList());
    }

    public void process(Matrix4 lightMatrix, Matrix4 viewMatrix) {
        shadow.mul(lightMatrix, shadow.set(viewMatrix).invert());
        ForkJoinPool.commonPool().invokeAll(tasks);
    }
}
