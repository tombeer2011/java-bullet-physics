package zynaps.engine;

public class GameLoop {
    private final Game game;
    private Thread thread;
    private boolean active;

    public GameLoop(Game game) {
        this.game = game;
        active = false;
    }

    public void start() {
        active = true;
        thread = new Thread(this::loop);
        thread.start();
    }

    public void stop() {
        active = false;
        if (thread != null) {
            thread = null;
        }
    }

    private void loop() {
        var t2 = System.nanoTime();
        while (active) {
            var t1 = t2;
            t2 = System.nanoTime();
            game.update((t2 - t1) / 1000000000.0);
            game.render();
        }
    }
}
