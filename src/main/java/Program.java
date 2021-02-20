import javax.swing.*;
import java.awt.*;

public class Program {
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            var panel = new BasicDemo();
            var frame = new JFrame("Java Bullet Physics");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.getContentPane().setLayout(new BorderLayout());
            frame.getContentPane().add(panel, BorderLayout.CENTER);
            frame.pack();
            frame.setIgnoreRepaint(true);
            frame.setResizable(false);
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
            frame.toFront();
            panel.initialise();
            panel.start();
            panel.requestFocus();
        });
    }
}
