package com.schneewittchen.automap;


import android.util.Log;

import com.schneewittchen.automap.loomoAdapters.LoomoAdapter;

/**
 * Interface between the java and the cpp part of the ros communication.
 */
public class CppInterface {

    /**
     * The enum Status.
     */
    public enum Status {
        /**
         * Disconnected status.
         */
        DISCONNECTED,
        /**
         * Connecting status.
         */
        CONNECTING,
        /**
         * Connected status.
         */
        CONNECTED};

    private static CppInterface instance = new CppInterface();
    private Status status;
    private ConnectionListener connectionListener;
    private ConnectionThread connectionThread;

    /**
     * Get the instance of this class as an singleton.
     *
     * @return Singleton instance
     */
    public static CppInterface getInstance() {
        return instance;
    }

    // Load native library
    static {
        System.loadLibrary("native-lib");
    }


    /**
     * Instantiates a new Cpp interface.
     */
    public CppInterface() {
        status = Status.DISCONNECTED;
    }


    /**
     * Start ros try.
     *
     * @param ip       the ip
     * @param masterIp the master ip
     */
    public void startRosTry(final String ip, final String masterIp){
        if(status == Status.CONNECTED){
            return;
        }

        // Kill current connect try
        if(connectionThread != null && connectionThread.running){
            System.out.println("Kill current Connection Thread");
            connectionThread.stopConnect();
        }

        status = Status.CONNECTING;
        connectionThread = new ConnectionThread(ip, masterIp);
        connectionThread.start();
    }


    private void setConnected(boolean running) {
        Status lastStatus = status;

        if(status == Status.CONNECTING && !running){
            return;
        }else if(running){
            status = Status.CONNECTED;
        }else{
            status = Status.DISCONNECTED;
        }


        if(connectionListener != null && lastStatus != status){
            if(isRosRunning()){
                connectionListener.onConnected();
            }else{
                connectionListener.onDisconnected();
            }
        }

    }

    /**
     * Is ros running boolean.
     *
     * @return the boolean
     */
    public boolean isRosRunning(){
        return status == Status.CONNECTED;
    }

    /**
     * Set connection listener.
     *
     * @param listener the listener
     */
    public void setConnectionListener(ConnectionListener listener){
        this.connectionListener = listener;
    }


    /**
     * Start the cpp ros part. Looking for master and connect to all subscribers.
     *
     * @param ip       Ip of the system
     * @param masterIP Ip of ros master
     */
    public native void startROS(String ip, String masterIP);


    /**
     * Move to.
     *
     * @param x the x
     * @param y the y
     */
    public native void moveTo(double x, double y);

    /**
     * Publish an image via ros.
     *
     * @param imageType  Type of the image
     * @param sequenceId Sequence
     * @param nanoStamp  Time stamp in nanoseconds
     * @param frameId    Camera frame
     * @param width      Width of the image
     * @param height     Height of the image
     * @param data       Image data
     * @param len        the len
     */
    public native void sendImage(String imageType, int sequenceId, long nanoStamp,
                                 String frameId, int width, int height, byte[] data, int len);


    /**
     * Publishing transformation data.
     *
     * @param sequenceID Sequence
     * @param nanoStamp  Time stamp in nanoseconds
     * @param sourceId   Source frame
     * @param targetId   Target frame
     * @param tx         Translation x-axis
     * @param ty         Translation y-axis
     * @param tz         Translation z-axis
     * @param qx         Quaternion x-axis
     * @param qy         Quaternion y-axis
     * @param qz         Quaternion z-axis
     * @param qw         Quaternion w-axis
     */
    public native void sendTf(int sequenceID, long nanoStamp, String sourceId,
                              String targetId, double tx, double ty, double tz,
                              double qx, double qy, double qz, double qw);

    /**
     * Publishing information about the camera.
     *
     * @param imageType  Type of the image
     * @param sequenceId Sequence
     * @param nanoStamp  Time stamp in nanoseconds
     * @param frameID    Frame
     * @param width      Width of the image
     * @param height     Height of the image
     * @param d          the d
     * @param k          Intrinsics of the camera
     * @param p          the p
     */
    public native void sendCameraInfo(String imageType, int sequenceId, long nanoStamp,
                                      String frameID, int width, int height, double[] d, double[] k, double[]p);

    /**
     * Publish the odometrie of the robot.
     *
     * @param headerFrameId Header Frame
     * @param childFrameId  Child Frame
     * @param sequenceID    Sequence
     * @param stampNanos    Time stamp in nanoseconds
     * @param vel           Velocity
     * @param angVel        Angular Speed
     * @param x             X position
     * @param y             Y position
     * @param theta         Rotation
     */
    public native void sendOdom(String headerFrameId, String childFrameId, int sequenceID, long stampNanos, double vel, double angVel, double x, double y, double theta);

    /**
     * Send goal string.
     *
     * @param goal the goal
     */
    public native void sendGoalString(String goal);

    /**
     * Controls the motion of the robot by speed and angular velocity.
     *
     * @param vel    Velocity
     * @param rotVel Angular velocity
     */
    public static void controlRobot(double vel, double rotVel) {
        LoomoAdapter.getInstance().control((float) vel, (float) rotVel);
    }


    /**
     * Updates the status of the ros connection
     *
     * @param running Stable Connection flag
     */
    public static void rosStatusUpdate(boolean running) {
        instance.setConnected(running);

        System.out.println("Ros status update: " + running);
    }


    /**
     * The interface Connection listener.
     */
    public interface ConnectionListener {

        /**
         * On connected.
         */
        void onConnected();

        /**
         * On disconnected.
         */
        void onDisconnected();
    }


    /**
     * The type Connection thread.
     */
    class ConnectionThread extends Thread {

        /**
         * The Ip.
         */
        String ip;
        /**
         * The Master ip.
         */
        String masterIp;
        /**
         * The Running.
         */
        volatile boolean running = true;

        /**
         * Instantiates a new Connection thread.
         *
         * @param ip       the ip
         * @param masterIp the master ip
         */
        public ConnectionThread(String ip, String masterIp){
            this.ip = ip;
            this.masterIp = masterIp;
        }

        public void run() {
            int counter = 0;

            while(!isRosRunning() && running){
                counter++;
                System.out.println("Try to connect to ros master #" + counter);
                startROS(ip, masterIp);

                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    e.printStackTrace();

                }
            }


            if(isRosRunning()) {
                System.out.println("Successful connected to ros master");
            }
        }

        /**
         * Stop connect.
         */
        public void stopConnect(){
            running = false;
        }
    }
}
