package org.mordraug.experimental.gyroemu;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.opengl.Matrix;
import android.test.suitebuilder.annotation.Suppress;
import android.util.Log;
import android.util.SparseArray;

import java.io.Console;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Array;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EventListener;

import static de.robv.android.xposed.XposedHelpers.findClass;

import de.robv.android.xposed.IXposedHookLoadPackage;
import de.robv.android.xposed.XC_MethodHook;
import de.robv.android.xposed.XposedBridge;
import de.robv.android.xposed.XposedHelpers;
import de.robv.android.xposed.callbacks.XC_LoadPackage;

/**
 * Created by Mordraug on 7/22/2016.
 */

public class GyroEmu implements IXposedHookLoadPackage {

    private Sensor vSensor;
    private SensorEventListener acc_listener;
    private SensorEventListener mag_listener;
    private SensorEventListener gyro_listener;
    private float[] last_acc={0,0,0};
    private float[] last_mag={0,0,0};
    private float[] g_vector = {0,0,0};
    private float[] rt0 = new float[16];
    private float[][] gyro_filter = new float[3][15];
    private int handle = 81;
    private long last_update=0;

    public GyroEmu(){
        //Creating sensor instance and setting its variables.
        try {
            Class<?> c = Class.forName("android.hardware.Sensor");
            Constructor<?> constructor = c.getDeclaredConstructor();
            constructor.setAccessible(true);
            vSensor = (Sensor)constructor.newInstance();
        } catch (ClassNotFoundException e) {
            Log.e("GyroEmu", "Error creating virtual Sensor: 00");
        }catch (NoSuchMethodException e) {
            Log.e("GyroEmu", "Error creating virtual Sensor: 01");
        }catch (IllegalAccessException e){
            Log.e("GyroEmu", "Error creating virtual Sensor: 02");
        }catch(InstantiationException e){
            Log.e("GyroEmu", "Error creating virtual Sensor: 03");
        }catch(InvocationTargetException e){
            Log.e("GyroEmu", "Error creating virtual Sensor: 04");
        }
        try {
            Field type = vSensor.getClass().getDeclaredField("mType");
            type.setAccessible(true);
            type.set(vSensor, Sensor.TYPE_GYROSCOPE);

            Field string_type = vSensor.getClass().getDeclaredField("mStringType");
            string_type.setAccessible(true);
            string_type.set(vSensor, Sensor.STRING_TYPE_GYROSCOPE);

            Field name = vSensor.getClass().getDeclaredField("mName");
            name.setAccessible(true);
            name.set(vSensor, "GyroEmu Virtual Sensor");

            Field vendor = vSensor.getClass().getDeclaredField("mVendor");
            vendor.setAccessible(true);
            vendor.set(vSensor, "Mordraug");

            Field permission = vSensor.getClass().getDeclaredField("mRequiredPermission");
            permission.setAccessible(true);
            permission.set(vSensor, "android.hardware.sensor.gyroscope");

            Field handle = vSensor.getClass().getDeclaredField("mHandle");
            handle.setAccessible(true);
            handle.set(vSensor, 81);

        }catch(NoSuchFieldException e){
            Log.e("GyroEmu", "Error creating virtual Sensor: 05");
        }catch(IllegalAccessException e){
            Log.e("GyroEmu", "Error creating virtual Sensor: 06");
        }

        //Accelerometers and Magnetometers Listeners.
        acc_listener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent event) {
                //Log.e("GyroEmu","acc_val:"+event.values[0]+";"+event.values[1]+";"+event.values[2]);
                last_acc=event.values;
                update(last_acc,last_mag);
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) {

            }
        };
        mag_listener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent event) {
                //Log.e("GyroEmu","mag_val:"+event.values[0]+";"+event.values[1]+";"+event.values[2]);
                last_mag=event.values;
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) {
            }
        };

    }

    //simple average filter.
    public float[] lowpass(float[] input, float[][] filter){
        float[] out = new float[input.length];
        for(int a=0;a<filter.length;a++){
            for(int b = filter[0].length-1;b>0;b--){
                filter[a][b]=filter[a][b-1];
            }
            filter[a][0]=input[a];
            for(int b = 0; b<filter.length;b++){
                out[a]+=filter[a][b];
            }
            out[a]/=filter[0].length;
            out[a]=(float)Math.floor(out[a]*10000)/10000f;
        }
        //Log.e("GyroEmu", Arrays.deepToString(filter));
        return out;
    }

    //vector normalize
    public float[] normalize(float[]vector){
        float[] out = new float[vector.length];
        float length = length(vector);
        for(int i = 0; i<vector.length; i++){
            out[i]=vector[i]/length;
        }
        return out;
    }

    //vector length
    public float length(float[] vector){
        float l = 0;
        for(float f : vector){
            l+=f*f;
        }
        return (float)Math.sqrt(l);
    }

    //roundup for vectors
    public float[] roundup(float[] input){
        float[] out = new float[input.length];
        for(int i=0; i<input.length;i++){
            out[i]=roundup(input[i]);
        }
        return out;
    }

    //It's actually a truncate
    public float roundup(float input){
        return (float)Math.floor(input*1000)/1000;
    }

    public void update(float[] acc_val, float[] mag_val){

        //Some shady algorithm I wrote at 5AM to get nice accelerometer output
        float gravity = 9.85f;
        float weight = Math.abs(1-(length(acc_val))/gravity)*0.9f+0.1f;
        float[] norm_acc = normalize(acc_val);
        float l = length(norm_acc);
        g_vector[0] = (1-weight)*g_vector[0]+weight*norm_acc[0];
        g_vector[1] = (1-weight)*g_vector[1]+weight*norm_acc[1];
        g_vector[2] = (1-weight)*g_vector[2]+weight*norm_acc[2];
        g_vector=roundup(normalize(g_vector));
        float[] mag_vector = roundup(normalize(mag_val));

        //Based on: https://github.com/memsindustrygroup/Open-Source-Sensor-Fusion/wiki/Virtual%20Gyro
        float[] momentum = new float[3];
        float[] rt1 = new float[16];

        SensorManager.getRotationMatrix(rt1,null,g_vector,mag_vector);
        if(last_update>0){
            float[] temp = new float[16];
            Matrix.multiplyMM(temp,0,rt1,0,rt0,0);
            double delta_t=(double)(System.nanoTime()-last_update)*0.000000002;
            momentum[0]=(float)(temp[2]-temp[8]/delta_t);
            momentum[1]=-(float)(temp[9]-temp[6]/delta_t);
            momentum[2]=(float)(temp[4]-temp[1]/delta_t);
            //Log.e("TGyro", ""+delta_t);
            momentum = roundup(lowpass(momentum,gyro_filter));
            for(int i=0;i<3;i++){
                if(Math.abs(momentum[i])<=0.01)
                    momentum[i]=0;
                momentum[i]*=1;
            }
        }
        Matrix.transposeM(rt0,0,rt1,0);
        last_update=System.nanoTime();

        //Creating SensorEvent instance and setting its variables, then calling onSensorChanged
        Constructor[] ctors = SensorEvent.class.getDeclaredConstructors();
        Constructor ctor = null;
        for (int i = 0; i < ctors.length; i++) {
            ctor = ctors[i];
            if (ctor.getGenericParameterTypes().length == 1)
                break;
        }
        try {
            ctor.setAccessible(true);
            SensorEvent event = (SensorEvent)ctor.newInstance(3);
            System.arraycopy(momentum,0,event.values,0,event.values.length);
            event.timestamp = System.nanoTime();
            event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH;
            event.sensor = vSensor;
            gyro_listener.onSensorChanged(event);
        }catch(IllegalAccessException e){
            Log.e("GyroEmu", "Error updating sensor: 01");
        }catch(InvocationTargetException e){
            Log.e("GyroEmu", "Error updating sensor: 02");
        }catch(InstantiationException e){
            Log.e("GyroEmu", "Error updating sensor: 03");
        }

    }

    //Bewaaare the HOOKS
    public void handleLoadPackage(XC_LoadPackage.LoadPackageParam lpparam) throws Throwable {
        try {
            final Class<?> sensorEQ = findClass(
                    "android.hardware.SystemSensorManager$SensorEventQueue",
                    lpparam.classLoader);
            final Class<?> sensorSMGR = findClass(
                    "android.hardware.SystemSensorManager",
                    lpparam.classLoader);
            final Class<?> sensorMGR = findClass(
                    "android.hardware.SensorMana" +
                            "ger",
                    lpparam.classLoader);

            XposedBridge.hookAllConstructors(sensorSMGR, new
                    XC_MethodHook() {
                        @SuppressWarnings("unchecked")
                        @Override
                        protected void afterHookedMethod(MethodHookParam param) throws
                                Throwable {
                                try {
                                    Field list = param.thisObject.getClass().getDeclaredField("mFullSensorsList");
                                    list.setAccessible(true);

                                    ArrayList<Sensor> list_inst = new ArrayList<Sensor>();
                                    Object list_val = list.get(param.thisObject);
                                    Method add_method = list_val.getClass().getDeclaredMethod("add", Object.class);
                                    add_method.invoke(list_val,vSensor);
                                    Log.e("GyroEmu", "Virtual Gyroscope injected!");
                                }catch(NoSuchFieldException e){
                                    Log.e("GyroEmu", "Error injecting Sensor: 01");
                                }catch(IllegalAccessException e){
                                    Log.e("GyroEmu", "Error injecting Sensor: 02");
                                }
                        }
                    });

            XposedBridge.hookAllMethods(sensorMGR, "registerListener", new
                    XC_MethodHook() {
                        @SuppressWarnings("unchecked")
                        @Override
                        protected void beforeHookedMethod(MethodHookParam param) throws
                                Throwable {
                            if(((Sensor)param.args[1]).getType()==Sensor.TYPE_GYROSCOPE||((Sensor)param.args[1]).getType()==Sensor.TYPE_GYROSCOPE_UNCALIBRATED){
                                gyro_listener = (SensorEventListener)param.args[0];
                                param.setResult(true);
                                ((SensorManager)param.thisObject).registerListener(mag_listener, ((SensorManager)param.thisObject).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), (int)param.args[2]);
                                ((SensorManager)param.thisObject).registerListener(acc_listener, ((SensorManager)param.thisObject).getDefaultSensor(Sensor.TYPE_ACCELEROMETER), (int)param.args[2]);
                            }
                        }
                    });
            XposedBridge.hookAllMethods(sensorMGR, "unregisterListener", new
                    XC_MethodHook() {
                        @SuppressWarnings("unchecked")
                        @Override
                        protected void afterHookedMethod(MethodHookParam param) throws
                                Throwable {
                            if(param.args.length==2 && param.args[1] instanceof Sensor && (((Sensor)param.args[1]).getType()==Sensor.TYPE_GYROSCOPE||((Sensor)param.args[1]).getType()==Sensor.TYPE_GYROSCOPE_UNCALIBRATED)){
                                ((SensorManager)param.thisObject).unregisterListener(mag_listener);
                                ((SensorManager)param.thisObject).unregisterListener(acc_listener);
                            }
                        }
                    });

        } catch (Throwable t) {
            Log.e("GyroEmu", "Exception in SystemSensorEvent hook: " + t.getMessage());
            // Do nothing
        }
    }
}
