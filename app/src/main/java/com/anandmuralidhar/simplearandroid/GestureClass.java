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
import android.support.v4.view.GestureDetectorCompat;
import android.view.GestureDetector;
import android.view.MotionEvent;

public class GestureClass {

    GestureDetectorCompat mTapDetector;
    private native void DoubleTapNative();

    public GestureClass(Activity activity) {

        // instantiate listener for detecting double-tap
        mTapDetector = new GestureDetectorCompat(activity, new MyTapListener());
    }

    // this class detects double-tap gesture
    class MyTapListener extends GestureDetector.SimpleOnGestureListener {

        public boolean onDoubleTap (MotionEvent event) {
            DoubleTapNative();
            return true;
        }
    }
}
