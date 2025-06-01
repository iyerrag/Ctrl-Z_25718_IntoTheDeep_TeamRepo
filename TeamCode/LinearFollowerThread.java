public class LinearFollowerThread {
    private Thread movementThread;
    private boolean running = false;
    private Follower follower;
    private PathChain currentPath;

    public FollowerThread(HardwareMap hardwareMap){
        follower = new Follower(hardwareMap);

        movementThread = new Thread(() -> {
            while (true) {
                if (running && currentPath != null){

                }
            }
        })
    }
}
