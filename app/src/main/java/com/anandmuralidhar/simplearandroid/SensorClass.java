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
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;
import android.view.Display;
import android.view.Surface;
import android.view.View;

public class SensorClass implements SensorEventListener {

    public SensorManager mSensorManager;
    private View mView;
    private int deviceRotation;

    // accelerometer variables
    Sensor mAccel;
    private float[] gravity = new float[3];
    private boolean isAccelSensorAvailable;

    private boolean isSensorsAvailable;

    public boolean isSensorsAvailable() {
        return isSensorsAvailable;
    }
    private native void SendGravityToNative(float gravityX, float gravityY, float gravityZ);

    public SensorClass(Activity mainActivity, View view) {

        mSensorManager = (SensorManager) mainActivity.getSystemService(Context.SENSOR_SERVICE);

        // fetch accel sensor
        mAccel = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

        // if sensor is not available, app will exit
        isSensorsAvailable = (mAccel != null);
        mView= view;

        // initialize gravity vector to 0 before starting to filter inputs
        gravity[0] = 0.0f;
        gravity[1] = 0.0f;
        gravity[2] = 0.0f;
    }

    public boolean RegisterSensors() {

        Log.d("SensorClass", "Registering sensors");
        if (isSensorsAvailable == false) {
            return false;
        }

        isAccelSensorAvailable = mSensorManager.registerListener(this, mAccel,
                SensorManager.SENSOR_DELAY_GAME);

        // if sensor is not registered, app will exit
        return isAccelSensorAvailable;
    }

    public void UnregisterSensors() {

        Log.d("SensorClass", "Unregistering sensor listener");
        mSensorManager.unregisterListener(this);

    }

    // IIR filter parameter for accelerometer filtering. range [0.0f -> 1.0f]
    // higher values -> more filtering
    private static final float ACCEL_ALPHA = 0.8f;

    // Isolate the force of gravity with a low-pass filter.
    private void CalculateGravityFromAccelerometer(float [] accel) {

        gravity[0] = ACCEL_ALPHA * gravity[0] + (1 - ACCEL_ALPHA) * accel[0];
        gravity[1] = ACCEL_ALPHA * gravity[1] + (1 - ACCEL_ALPHA) * accel[1];
        gravity[2] = ACCEL_ALPHA * gravity[2] + (1 - ACCEL_ALPHA) * accel[2];

    }

    @Override
    public void onSensorChanged(SensorEvent event) {

        // wait for display to be initialized since we need to fetch device rotation before
        // processing sensor data
        if(mView.getDisplay() == null) {
            return;
        }

        // coordinate system of device does not change with orientation
        // we check for device orientation and rearrange event values so that subsequent
        // calculations can be oblivious to change in orientation
        deviceRotation = mView.getDisplay().getRotation();
        float [] eventValues= new float[3];
        switch(deviceRotation) {

            case Surface.ROTATION_0:
                eventValues[0] = event.values[0];
                eventValues[1] = event.values[1];
                eventValues[2] = event.values[2];
                break;

            case Surface.ROTATION_90:
                eventValues[0] = -event.values[1];
                eventValues[1] = event.values[0];
                eventValues[2] = event.values[2];
                break;

            case Surface.ROTATION_180:
                eventValues[0] = -event.values[0];
                eventValues[1] = -event.values[1];
                eventValues[2] = event.values[2];
                break;

            case Surface.ROTATION_270:
                eventValues[0] = event.values[1];
                eventValues[1] = -event.values[0];
                eventValues[2] = event.values[2];
                break;

        }

        switch (event.sensor.getType()) {

            case Sensor.TYPE_ACCELEROMETER:
                CalculateGravityFromAccelerometer(eventValues);
                SendGravityToNative(gravity[0], gravity[1], gravity[2]);
                break;

        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Do something here if sensor accuracy changes.
    }

}


