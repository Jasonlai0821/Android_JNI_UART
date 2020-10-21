package com.xinshiyun.utils;

import android.content.Context;
import android.content.Intent;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.util.Log;

import java.lang.ref.WeakReference;

public class CmdControlJNI {
    private static String TAG = CmdControlJNI.class.getSimpleName();

    private EventHandler mEventHandler;
    private Context mContext;
    private CmdControlCallback mCmdControlCallback = null;

    private long mNativeContext; // accessed by native methods
    private static final int 		CMD_MASTER_AIR			=0x20;
    private static final int 		CMD_MASTER_ADLIGHT		=0x21;
    private static final int 		CMD_MASTER_HEADLAMP		=0x22;
    private static final int 		CMD_MASTER_EXHAUSTFAN	=0x23;
    private static final int 		CMD_MASTER_COOLINGFAN	=0x24;
    private static final int 		CMD_MASTER_TOUCHSCREEN	=0x25;
    private static final int 		CMD_MASTER_TV			=0x26;
    private static final int 		CMD_MASTER_RKE			=0x27;
    private static final int 		CMD_MASTER_SMOKEALARM	=0x28;
    private static final int 		CMD_MASTER_HBS			=0x29;
    private static final int 		CMD_MASTER_TS			=0x2A;
    private static final int 		CMD_MASTER_HS			=0x2B;
    private static final int		CMD_SLAVE_RESET			=0x2C;
    private static final int		CMD_SLAVE_RESULT		=0x2D;

    private class EventHandler extends Handler {
        private CmdControlJNI mCmdControlJNI;

        public EventHandler(CmdControlJNI dc, Looper looper) {
            super(looper);
            mCmdControlJNI = dc;
        }

        @Override
        public void handleMessage(Message msg) {
            if (mCmdControlJNI.mNativeContext == 0) {
                Log.w(TAG, "Display Control went away with unhandled events");
                return;
            }

            Log.d(TAG, "msg.what = " + msg.what);

            switch (msg.what) {
                case CMD_MASTER_AIR:
                case CMD_MASTER_ADLIGHT:
                case CMD_MASTER_HEADLAMP:
                case CMD_MASTER_EXHAUSTFAN:
                case CMD_MASTER_COOLINGFAN:
                case CMD_MASTER_TOUCHSCREEN:
                case CMD_MASTER_TV:
                case CMD_MASTER_RKE:
                case CMD_MASTER_SMOKEALARM:
                case CMD_MASTER_HBS:
                case CMD_MASTER_TS:
                case CMD_MASTER_HS:
                    if(mCmdControlCallback != null){
                        mCmdControlCallback.onCmdExcuteResult(msg.what,msg.arg2);
                    }
                    break;
                case CMD_SLAVE_RESET:
                    rebootSystem();
                    break;
                case CMD_SLAVE_RESULT:
                    Log.d(TAG,"cmd Execute result:"+msg.arg2);
                    break;

            }
        }
    }

    static {
        System.loadLibrary("CmdControlJNI");
        native_init();
    }

    public CmdControlJNI(Context context) {
        Looper looper;
        if ((looper = Looper.myLooper()) != null) {
            mEventHandler = new EventHandler(this, looper);
        } else if ((looper = Looper.getMainLooper()) != null) {
            mEventHandler = new EventHandler(this, looper);
        } else {
            mEventHandler = null;
        }

        native_setup(new WeakReference<CmdControlJNI>(this));
        mContext =context;
    }

    /*
     * Called from native code when an interesting event happens. This method
     * just uses the EventHandler system to post the event back to the main app
     * thread. We use a weak reference to the original DisplayControl object so
     * that the native code is safe from the object disappearing from underneath
     * it. (This is the cookie passed to native_setup().)
     */
    @SuppressWarnings("rawtypes")
    public static void postEventFromNative(Object dc_ref, int what, int arg1,int arg2) {
        CmdControlJNI dc = (CmdControlJNI) ((WeakReference) dc_ref).get();
        if (dc == null) {
            //return 0;
        }
        if (dc.mEventHandler != null) {
            Message m = dc.mEventHandler.obtainMessage(what, arg1, arg2);
            dc.mEventHandler.sendMessage(m);
        }
        //return 0;
    }

    public void rebootSystem()
    {
        Intent intent = new Intent(Intent.ACTION_REBOOT);
        intent.putExtra("nowait", 1);
        intent.putExtra("interval", 1);
        intent.putExtra("window", 0);
        mContext.sendBroadcast(intent);
    }

    //串口指令的格式是Lead code + Length +cmd + data +crc
    //如下的cmd缺省crc，crc值由底层自动计算
    public String getValue(int value)
    {
        String val = null;
        val = Integer.toHexString(value);
        if(val.length() == 1){
            val = "0"+val;
        }
        val = val.toUpperCase();
        return val;
    }

    //onXXXSetParameter()函数的返回值，代表执行结果：1，成功，0：失败
    public int onAIRSetParameter(int value)
    {
        String cmd = "5A"+"06"+"20"+"0A"+getValue(value);
        return onExecuteCmd(cmd);
    }

    //onXXXGetParameter()函数的返回值，代表获取对应的参数值
    public int onAIRGetParameter()
    {
        String cmd = "5A"+"05"+"20"+"0B";
        return onExecuteCmd(cmd);
    }

    public int onADLightSetParameter(int value)
    {
        String cmd = "5A"+"06"+"21"+"0A"+getValue(value);
        return onExecuteCmd(cmd);
    }

    public int onADLightGetParameter()
    {
        String cmd = "5A"+"05"+"21"+"0B";
        return onExecuteCmd(cmd);
    }

    public int onHeadLampSetParameter(int value)
    {
        String cmd = "5A"+"06"+"22"+"0A"+getValue(value);
        return onExecuteCmd(cmd);
    }

    public int onHeadLampGetParameter()
    {
        String cmd = "5A"+"05"+"22"+"0B";
        return onExecuteCmd(cmd);
    }

    public int onExhaustFanSetParameter(int value)
    {
        String cmd = "5A"+"06"+"23"+"0A"+getValue(value);
        return onExecuteCmd(cmd);
    }

    public int onExhaustFanGetParameter()
    {
        String cmd = "5A"+"05"+"23"+"0B";
        return onExecuteCmd(cmd);
    }

    public int onCoolingFanSetParameter(int value)
    {
        String cmd = "5A"+"06"+"24"+"0A"+getValue(value);
        return onExecuteCmd(cmd);
    }

    public int onCoolingFanGetParameter()
    {
        String cmd = "5A"+"05"+"24"+"0B";
        return onExecuteCmd(cmd);
    }

    public int onTouchScreenSetParameter(int value)
    {
        String cmd = "5A"+"06"+"25"+"0A"+getValue(value);
        return onExecuteCmd(cmd);
    }

    public int onTouchScreenGetParameter()
    {
        String cmd = "5A"+"05"+"25"+"0B";
        return onExecuteCmd(cmd);
    }

    public int onTVSetParameter(int value)
    {
        String cmd = "5A"+"06"+"26"+"0A"+getValue(value);
        return onExecuteCmd(cmd);
    }

    public int onTVGetParameter()
    {
        String cmd = "5A"+"05"+"26"+"0B";
        return onExecuteCmd(cmd);
    }

    public int onRKESetParameter(int value)
    {
        String cmd = "5A"+"06"+"27"+"0A"+getValue(value);
        return onExecuteCmd(cmd);
    }

    public int onRKEGetParameter()
    {
        String cmd = "5A"+"05"+"27"+"0B";
        return onExecuteCmd(cmd);
    }

    public int onSmokeAlarmSetParameter(int value)
    {
        String cmd = "5A"+"06"+"28"+"0A"+getValue(value);
        return onExecuteCmd(cmd);
    }

    public int onSmokeAlarmGetParameter()
    {
        String cmd = "5A"+"05"+"28"+"0B";
        return onExecuteCmd(cmd);
    }

    public int onHBSSetParameter(int value)
    {
        String cmd = "5A"+"06"+"29"+"0A"+getValue(value);
        return onExecuteCmd(cmd);
    }

    public int onHBSGetParameter()
    {
        String cmd = "5A"+"05"+"29"+"0B";
        return onExecuteCmd(cmd);
    }

    public int onTSSetParameter(int value)
    {
        String cmd = "5A"+"06"+"2A"+"0A"+getValue(value);
        return onExecuteCmd(cmd);
    }

    public int onTSGetParameter()
    {
        String cmd = "5A"+"05"+"2A"+"0B";
        return onExecuteCmd(cmd);
    }

    public int onHSSetParameter(int value)
    {
        String cmd = "5A"+"06"+"2B"+"0A"+getValue(value);
        //需将value值打印出来确认是怎么样的，值范围为0x00-0xFF;若是1位，则6需用06表示，
        return onExecuteCmd(cmd);
    }

    public int onHSGetParameter()
    {
        String cmd = "5A"+"05"+"2B"+"0B";
        return onExecuteCmd(cmd);
    }

    private static native final void native_init();

    private native final void native_setup(Object dc_this);

    private native void native_finalize();

    public native int onExecuteCmd(String cmd);

    public void finalize() throws Throwable {
        // TODO Auto-generated method stub
        super.finalize();
        if (mNativeContext != 0) {
            native_finalize();
        }
        mNativeContext = 0L;
    }

    public void setCmdControlCallback(CmdControlCallback callback){
        mCmdControlCallback = callback;
    }

    public interface CmdControlCallback{
        void onCmdExcuteResult(int cmd_type,int value);
    }
}
