package com.xinshiyun.utils;

import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import androidx.appcompat.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity {
    private static String TAG = MainActivity.class.getSimpleName();

    CmdControlJNI mCmdControlJNI = null;

    public static Button mExcuteBtn;
    public static EditText mEditCmd;
    public static String mCmdString = null;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Log.d(TAG,"onCreate()");

        initView();
        initUartControl();
    }

    public void initView()
    {
        mExcuteBtn = (Button)findViewById(R.id.Excutebtn);
        mEditCmd = (EditText)findViewById(R.id.Cmdstring);

        mExcuteBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mCmdString = mEditCmd.getText().toString();

                if(mCmdString != null && mCmdString.length() !=0){
                    Log.d(TAG,"onClick() send mCmdString:"+mCmdString);
                    new Thread(){
                        public void run(){
                            mCmdControlJNI.onAIRSetParameter(1);
                            mCmdControlJNI.onAIRSetParameter(18);
                            mCmdControlJNI.onAIRGetParameter();
                            mCmdControlJNI.onADLightSetParameter(1);
                            mCmdControlJNI.onADLightGetParameter();
                            mCmdControlJNI.onHeadLampSetParameter(255);
                            mCmdControlJNI.onHeadLampSetParameter(10);
                            mCmdControlJNI.onHeadLampGetParameter();
                            mCmdControlJNI.onExhaustFanSetParameter(1);
                            mCmdControlJNI.onExhaustFanGetParameter();
                            mCmdControlJNI.onCoolingFanSetParameter(1);
                            mCmdControlJNI.onCoolingFanGetParameter();
                            mCmdControlJNI.onTouchScreenSetParameter(1);
                            mCmdControlJNI.onTouchScreenGetParameter();
                            mCmdControlJNI.onTVSetParameter(1);
                            mCmdControlJNI.onTVGetParameter();
                            mCmdControlJNI.onRKESetParameter(1);
                            mCmdControlJNI.onRKEGetParameter();
                            mCmdControlJNI.onSmokeAlarmSetParameter(1);
                            mCmdControlJNI.onSmokeAlarmGetParameter();
                            mCmdControlJNI.onHBSSetParameter(1);
                            mCmdControlJNI.onHBSGetParameter();
                            mCmdControlJNI.onTSSetParameter(255);
                            mCmdControlJNI.onTSSetParameter(50);
                            mCmdControlJNI.onTSGetParameter();
                            mCmdControlJNI.onHSSetParameter(255);
                            mCmdControlJNI.onHSSetParameter(100);
                            mCmdControlJNI.onHSGetParameter();

                            try {
                                sleep(1000);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }

                            mCmdControlJNI.onAIRSetParameter(0);
                            mCmdControlJNI.onAIRGetParameter();
                            mCmdControlJNI.onADLightSetParameter(0);
                            mCmdControlJNI.onADLightGetParameter();
                            mCmdControlJNI.onHeadLampSetParameter(0);
                            mCmdControlJNI.onHeadLampGetParameter();
                            mCmdControlJNI.onExhaustFanSetParameter(0);
                            mCmdControlJNI.onExhaustFanGetParameter();
                            mCmdControlJNI.onCoolingFanSetParameter(0);
                            mCmdControlJNI.onCoolingFanGetParameter();
                            mCmdControlJNI.onTouchScreenSetParameter(0);
                            mCmdControlJNI.onTouchScreenGetParameter();
                            mCmdControlJNI.onTVSetParameter(0);
                            mCmdControlJNI.onTVGetParameter();
                            mCmdControlJNI.onRKESetParameter(0);
                            mCmdControlJNI.onRKEGetParameter();
                            mCmdControlJNI.onSmokeAlarmSetParameter(0);
                            mCmdControlJNI.onSmokeAlarmGetParameter();
                            mCmdControlJNI.onHBSSetParameter(0);
                            mCmdControlJNI.onHBSGetParameter();
                            mCmdControlJNI.onTSSetParameter(240);
                            mCmdControlJNI.onTSGetParameter();
                            mCmdControlJNI.onHSSetParameter(240);
                            mCmdControlJNI.onHSGetParameter();
                        }
                    }.start();
                }

                Log.d(TAG,"onClick() send cmd");
            }
        });
    }

    public void initUartControl()
    {
        mCmdControlJNI = new CmdControlJNI(this);
        //setCmdControlCallback(callback);
    }

    @Override
    public void onDestroy() {                         //销毁
        Log.d(TAG, "onDestroy()");
        super.onDestroy();
        try {
            mCmdControlJNI.finalize();
        } catch (Throwable throwable) {
            throwable.printStackTrace();
        }
        mCmdControlJNI = null;
    }
}
