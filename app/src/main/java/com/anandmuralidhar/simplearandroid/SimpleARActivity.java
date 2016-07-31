/*
 *    Copyright 2016 Anand Muralidhar
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

package com.anandmuralidhar.simplearandroid;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.res.AssetManager;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;


public class SimpleARActivity extends Activity{

    private GLSurfaceView mGLView = null;
    private CameraClass mCameraObject;
    private boolean appIsExiting=false;
    private GestureClass mGestureObject;
    private SensorClass mSensorObject;

    private native void CreateObjectNative(AssetManager assetManager, String pathToInternalDir);
    private native void DeleteObjectNative();
    private native void SetCameraParamsNative(int previewWidth, int previewHeight, float cameraFOV);

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        AssetManager assetManager = getAssets();
        String pathToInternalDir = getFilesDir().getAbsolutePath();

        mCameraObject = new CameraClass();
        if(!mCameraObject.IsResolutionSupported()) {
            ShowExitDialog(this, getString(R.string.exit_no_resolution));
            return;
        }

        // call the native constructors to create an object
        CreateObjectNative(assetManager, pathToInternalDir);
        SetCameraParamsNative(mCameraObject.GetPreviewWidth(), mCameraObject.GetPreviewHeight(),
                mCameraObject.GetFOV());

        // layout has only two components, a GLSurfaceView and a TextView
        setContentView(R.layout.simplear_layout);
        mGLView = (MyGLSurfaceView) findViewById (R.id.gl_surface_view);

        // mGestureObject will handle touch gestures on the screen
        mGestureObject = new GestureClass(this);
        mGLView.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                mGestureObject.mTapDetector.onTouchEvent(event);
                return true;
            }
        });

        mSensorObject = new SensorClass(this, mGLView);
        if(!mSensorObject.isSensorsAvailable()) {
            ShowExitDialog(this, getResources().getString(R.string.exit_no_sensor));
            appIsExiting=true;
        }

    }

    @Override
    protected void onResume() {

        super.onResume();

        if(appIsExiting) {
            return;
        }

        // Android suggests that we call onResume on GLSurfaceView
        if (mGLView != null) {
            mGLView.onResume();
        }


        if(!mSensorObject.RegisterSensors()){
            ShowExitDialog(this, getResources().getString(R.string.exit_no_reg_sensor));
            appIsExiting=true;
            return;
        }

        // initialize the camera again in case activity was paused and resumed
        mCameraObject.InitializeCamera();
        mCameraObject.StartCamera();

    }

    @Override
    protected void onPause() {

        super.onPause();

        // Android suggests that we call onPause on GLSurfaceView
        if(mGLView != null) {
            mGLView.onPause();
        }

        mSensorObject.UnregisterSensors();

        mCameraObject.StopCamera();

    }

    @Override
    protected void onDestroy() {

        super.onDestroy();

        // We are exiting the activity, let's delete the native objects
        DeleteObjectNative();

    }

    public void ShowExitDialog(final Activity activity, String exitMessage){
        appIsExiting = true;
        AlertDialog.Builder alertDialogBuilder = new AlertDialog.Builder(activity);
        alertDialogBuilder.setMessage(exitMessage)
                .setCancelable(false)
                .setPositiveButton("OK", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface arg0, int arg1) {
                        activity.finish();
                    }
                });

        AlertDialog alertDialog = alertDialogBuilder.create();
        alertDialog.show();
    }

    /**
     * load libSimpleARNative.so since it has all the native functions
     */
    static {
        System.loadLibrary("SimpleARNative");
    }
}
