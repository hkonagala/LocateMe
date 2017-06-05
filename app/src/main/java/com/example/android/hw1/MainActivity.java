package com.example.android.hw1;

import android.Manifest;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

public class MainActivity extends AppCompatActivity implements LocationListener, SensorEventListener {

    TextView tvLat1, tvLon1, tvLat2, tvLon2, tvDiff;
    LocationManager locationManager_;
    private Handler myHandler_ = new Handler();
    String newLat1, newLon1;
    double latestAngle;
    Sensor accelerometer_, gyroscope_, magnetometer_;
    List<AccelValue> accelReadings = new ArrayList<>();
    float[] g = new float[3];
    Location gpsLoc, netLoc, locActual;
    AccelValue pastAcceleration;
    Timer timer = new Timer();

    public static final int TIME_CONSTANT = 30;
    public static final float FILTER_COEFFICIENT = 0.98f;
    public static final float EPSILON = 0.000000001f;

    private float[] gyroscopes = new float[3];
    private float[] gyroAll = new float[9];
    private float[] gyroscopeAngle = new float[3];
    private float[] magnetValues = new float[3];
    private float[] aValues = new float[3];
    private float[] pastAValue = new float[3];
    private float[] magnetAngle = new float[3];
    private float[] newAngle = new float[3];
    private float[] rotationValues = new float[9];

    private static final float NS2S = 1.0f / 1000000000.0f;
    private float time;
    private boolean isStarted = true;



    final Handler h1 = new Handler(new Handler.Callback() {
        @Override
        public boolean handleMessage(Message message) {

            if (newLat1 != null && !newLat1.equals("")) {
                double distance = calcAccelDistance();
                double lastAngle = latestAngle;
                latestAngle = newAngle[0];
                double newPositiveAngle = (newAngle[0] < 0)?(360 + newAngle[0]) : newAngle[0];
                double lastPositiveAngle = (lastAngle < 0)?(360 + lastAngle) : lastAngle;
                MyLocation newLocation = getNewCoordinates(newLat1, newLon1, distance, newPositiveAngle - lastPositiveAngle);
                accelReadings = new ArrayList<>();
                pastAcceleration.setvX(0);
                pastAcceleration.setvY(0);
                pastAcceleration.setvZ(0);
                pastAcceleration.setdX(0);
                pastAcceleration.setdY(0);
                pastAcceleration.setdZ(0);
                synchronized (this) {
                    newLat1 = String.valueOf(newLocation.getLatitude());
                    newLon1 = String.valueOf(newLocation.getLongitude());
                    float absError = Float.valueOf(getError(String.valueOf(tvLat1.getText()), String.valueOf(tvLon1.getText()), newLat1, newLon1));
                    tvLat2.setText(newLat1);
                    tvLon2.setText(newLon1);
                    tvDiff.setText(String.valueOf(absError));
                }
            }

            return false;
        }
    });

    final Handler h2 = new Handler(new Handler.Callback() {
        @Override
        public boolean handleMessage(Message message) {
            float oneMinusCoeff = 1.0f - FILTER_COEFFICIENT;
            if (gyroscopeAngle[0] < -0.5 * Math.PI && magnetAngle[0] > 0.0) {
                newAngle[0] = (float) (FILTER_COEFFICIENT * (gyroscopeAngle[0] + 2.0 * Math.PI) + oneMinusCoeff * magnetAngle[0]);
                newAngle[0] -= (newAngle[0] > Math.PI) ? 2.0 * Math.PI : 0;
            } else if (magnetAngle[0] < -0.5 * Math.PI && gyroscopeAngle[0] > 0.0) {
                newAngle[0] = (float) (FILTER_COEFFICIENT * gyroscopeAngle[0] + oneMinusCoeff * (magnetAngle[0] + 2.0 * Math.PI));
                newAngle[0] -= (newAngle[0] > Math.PI) ? 2.0 * Math.PI : 0;
            } else {
                newAngle[0] = FILTER_COEFFICIENT * gyroscopeAngle[0] + oneMinusCoeff * magnetAngle[0];
            }
            if (gyroscopeAngle[1] < -0.5 * Math.PI && magnetAngle[1] > 0.0) {
                newAngle[1] = (float) (FILTER_COEFFICIENT * (gyroscopeAngle[1] + 2.0 * Math.PI) + oneMinusCoeff * magnetAngle[1]);
                newAngle[1] -= (newAngle[1] > Math.PI) ? 2.0 * Math.PI : 0;
            } else if (magnetAngle[1] < -0.5 * Math.PI && gyroscopeAngle[1] > 0.0) {
                newAngle[1] = (float) (FILTER_COEFFICIENT * gyroscopeAngle[1] + oneMinusCoeff * (magnetAngle[1] + 2.0 * Math.PI));
                newAngle[1] -= (newAngle[1] > Math.PI) ? 2.0 * Math.PI : 0;
            } else {
                newAngle[1] = FILTER_COEFFICIENT * gyroscopeAngle[1] + oneMinusCoeff * magnetAngle[1];
            }
            if (gyroscopeAngle[2] < -0.5 * Math.PI && magnetAngle[2] > 0.0) {
                newAngle[2] = (float) (FILTER_COEFFICIENT * (gyroscopeAngle[2] + 2.0 * Math.PI) + oneMinusCoeff * magnetAngle[2]);
                newAngle[2] -= (newAngle[2] > Math.PI) ? 2.0 * Math.PI : 0;
            } else if (magnetAngle[2] < -0.5 * Math.PI && gyroscopeAngle[2] > 0.0) {
                newAngle[2] = (float) (FILTER_COEFFICIENT * gyroscopeAngle[2] + oneMinusCoeff * (magnetAngle[2] + 2.0 * Math.PI));
                newAngle[2] -= (newAngle[2] > Math.PI) ? 2.0 * Math.PI : 0;
            } else {
                newAngle[2] = FILTER_COEFFICIENT * gyroscopeAngle[2] + oneMinusCoeff * magnetAngle[2];
            }

            gyroAll = getRotationMatrixFromOrientation(newAngle);
            System.arraycopy(newAngle, 0, gyroscopeAngle, 0, 3);

            return false;
        }
    });

    private String getError(String latitude, String longitude, String calcLatitudeValue, String calcLongitudeValue) {
        if (latitude == null || longitude == null || calcLatitudeValue == null || calcLongitudeValue == null
                || latitude.equals("") || longitude.equals("") || calcLatitudeValue.equals("") || calcLongitudeValue.equals("")){
            return "0";
        }
        float R = (float) 6371000;
        double dlat = Math.toRadians(Float.valueOf(calcLatitudeValue) - Float.valueOf(latitude));
        double dlon = Math.toRadians(Float.valueOf(calcLongitudeValue) - Float.valueOf(longitude));
        double a = Math.pow((Math.sin(dlat/2)),2) + Math.cos(Float.valueOf(latitude)) * Math.cos(Float.valueOf(calcLatitudeValue)) * Math.pow((Math.sin(dlon/2)), 2);
        double c = 2 * Math.atan2( Math.sqrt(a), Math.sqrt(1 - a) );
        double d = R * c;
        return String.valueOf(d);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        tvLat1 = (TextView) findViewById(R.id.lat1);
        tvLon1 = (TextView) findViewById(R.id.lon1);
        tvLat2 = (TextView) findViewById(R.id.lat2);
        tvLon2 = (TextView) findViewById(R.id.lon2);
        tvDiff = (TextView) findViewById(R.id.diff);
    }

    @Override
    protected void onResume() {
        super.onResume();
        newLat1 = null;
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED
                && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION}, 0);            return;
        }
        gyroscopeAngle[0] = 0.0f;
        gyroscopeAngle[1] = 0.0f;
        gyroscopeAngle[2] = 0.0f;
        gyroAll[0] = 1.0f; gyroAll[1] = 0.0f; gyroAll[2] = 0.0f;
        gyroAll[3] = 0.0f; gyroAll[4] = 1.0f; gyroAll[5] = 0.0f;
        gyroAll[6] = 0.0f; gyroAll[7] = 0.0f; gyroAll[8] = 1.0f;
        g[0] = 0;
        g[1] = 0;
        g[2] = 0;
        pastAcceleration = new AccelValue(0, 0, 0, 0);
        pastAcceleration.setvX(0);
        pastAcceleration.setvY(0);
        pastAcceleration.setvZ(0);
        pastAcceleration.setdX(0);
        pastAcceleration.setdY(0);
        pastAcceleration.setdZ(0);
        locationManager_ = (LocationManager) getSystemService(LOCATION_SERVICE);
        locationManager_.requestLocationUpdates(LocationManager.NETWORK_PROVIDER, 0, 0, this);
        locationManager_.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, this);

        SensorManager sensorManager_ = (SensorManager) getSystemService(SENSOR_SERVICE);
        accelerometer_ = sensorManager_.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyroscope_ = sensorManager_.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        magnetometer_ = sensorManager_.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        sensorManager_.registerListener(this, accelerometer_, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager_.registerListener(this, gyroscope_, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager_.registerListener(this, magnetometer_, SensorManager.SENSOR_DELAY_FASTEST);

        timer = new Timer();
        timer.schedule(new DistanceTask(), 1000, 2000);
        timer.schedule(new AzimuthTask(), 1000, TIME_CONSTANT);
    }

    @Override
    protected void onPause() {
        super.onPause();
        newLat1 = null;
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED
                && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION}, 0);
            return;
        }
        locationManager_.removeUpdates(this);
        timer.cancel();
        timer.purge();
    }

    private double calcAccelDistance(){
        double distance;
        synchronized (this) {
            float dT = 0;
            for (AccelValue val : accelReadings) {
                if (pastAcceleration.getTime() != 0) {
                    dT = (val.getTime() - pastAcceleration.getTime()) * NS2S;
                }
                val.setvX(pastAcceleration.getvX() + ((val.getaX() - pastAcceleration.getaX()) * dT));
                val.setvY(pastAcceleration.getvY() + ((val.getaY() - pastAcceleration.getaY()) * dT));
                val.setvZ(pastAcceleration.getvZ() + ((val.getaZ() - pastAcceleration.getaZ()) * dT));

                val.setdX(pastAcceleration.getdX() + ((val.getvX() - pastAcceleration.getvX()) * dT));
                val.setdY(pastAcceleration.getdY() + ((val.getvY() - pastAcceleration.getvY()) * dT));
                val.setdZ(pastAcceleration.getdZ() + ((val.getvZ() - pastAcceleration.getvZ()) * dT));

                pastAcceleration = val;
            }
            distance = Math.sqrt(Math.pow(pastAcceleration.getdX(), 2) + Math.pow(pastAcceleration.getdY(), 2) + Math.pow(pastAcceleration.getdZ(), 2));
        }
        return distance;
    }

    private void getRotationVectorFromGyro(float[] gyroValues,
                                           float[] deltaRotationVector,
                                           float timeFactor)
    {
        float[] normValues = new float[3];

        float omegaMagnitude =
                (float)Math.sqrt(gyroValues[0] * gyroValues[0] +
                        gyroValues[1] * gyroValues[1] +
                        gyroValues[2] * gyroValues[2]);
        if(omegaMagnitude > EPSILON) {
            normValues[0] = gyroValues[0] / omegaMagnitude;
            normValues[1] = gyroValues[1] / omegaMagnitude;
            normValues[2] = gyroValues[2] / omegaMagnitude;
        }

        float thetaOverTwo = omegaMagnitude * timeFactor;
        float sinThetaOverTwo = (float)Math.sin(thetaOverTwo);
        float cosThetaOverTwo = (float)Math.cos(thetaOverTwo);
        deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
        deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
        deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
        deltaRotationVector[3] = cosThetaOverTwo;
    }

    private float[] getRotationMatrixFromOrientation(float[] o) {
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];

        float sinX = (float)Math.sin(o[1]);
        float cosX = (float)Math.cos(o[1]);
        float sinY = (float)Math.sin(o[2]);
        float cosY = (float)Math.cos(o[2]);
        float sinZ = (float)Math.sin(o[0]);
        float cosZ = (float)Math.cos(o[0]);

        xM[0] = 1.0f; xM[1] = 0.0f; xM[2] = 0.0f;
        xM[3] = 0.0f; xM[4] = cosX; xM[5] = sinX;
        xM[6] = 0.0f; xM[7] = -sinX; xM[8] = cosX;
        yM[0] = cosY; yM[1] = 0.0f; yM[2] = sinY;
        yM[3] = 0.0f; yM[4] = 1.0f; yM[5] = 0.0f;
        yM[6] = -sinY; yM[7] = 0.0f; yM[8] = cosY;
        zM[0] = cosZ; zM[1] = sinZ; zM[2] = 0.0f;
        zM[3] = -sinZ; zM[4] = cosZ; zM[5] = 0.0f;
        zM[6] = 0.0f; zM[7] = 0.0f; zM[8] = 1.0f;

        float[] resultMatrix = matrixMultiplication(xM, yM);
        resultMatrix = matrixMultiplication(zM, resultMatrix);
        return resultMatrix;
    }

    private float[] matrixMultiplication(float[] A, float[] B) {
        float[] result = new float[9];

        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

        return result;
    }

    public MyLocation getNewCoordinates(String latitude, String longitude, double d, double azimuth){

        double dist = d/6371.0;
        double lat1 = Math.toRadians(Float.valueOf(latitude));
        double lon1 = Math.toRadians(Float.valueOf(longitude));
        double bearing = Math.toRadians(azimuth);

        double lat2 = Math.asin( Math.sin(lat1)*Math.cos(dist) + Math.cos(lat1)*Math.sin(dist)*Math.cos(bearing) );
        double a = Math.atan2(Math.sin(bearing)*Math.sin(dist)*Math.cos(lat1), Math.cos(dist)-Math.sin(lat1)*Math.sin(lat2));
        double lon2 = lon1 + a;
        lon2 = (lon2+ 3*Math.PI) % (2*Math.PI) - Math.PI;
        lat2 = Math.toDegrees(lat2);
        lon2 = Math.toDegrees(lon2);
        return new MyLocation(Double.valueOf(lat2).floatValue(), Double.valueOf(lon2).floatValue(), System.nanoTime());
    }

    // Private Classes

    private class AccelValue {
        float aX;
        float aY;
        float aZ;
        float vX;
        float vY;
        float vZ;
        float dX;
        float dY;
        float dZ;
        long time;

        float getvX() {
            return vX;
        }

        void setvX(float vX) {
            this.vX = vX;
        }

        float getvY() {
            return vY;
        }

        void setvY(float vY) {
            this.vY = vY;
        }

        float getvZ() {
            return vZ;
        }

        void setvZ(float vZ) {
            this.vZ = vZ;
        }

        float getdX() {
            return dX;
        }

        void setdX(float dX) {
            this.dX = dX;
        }

        float getdY() {
            return dY;
        }

        void setdY(float dY) {
            this.dY = dY;
        }

        float getdZ() {
            return dZ;
        }

        void setdZ(float dZ) {
            this.dZ = dZ;
        }

        AccelValue(float aX, float aY, float aZ, long time) {
            this.aX = aX;
            this.aY = aY;
            this.aZ = aZ;
            this.time = time;
        }
        float getaX() { return aX; }

        float getaY() { return aY; }

        float getaZ() { return aZ; }

        long getTime() { return time; }
    }

    private class MyLocation {
        float latitude, longitude;
        long time;

        MyLocation(float latitude, float longitude, long time) {
            this.latitude = latitude;
            this.longitude = longitude;
            this.time = time;
        }

        float getLatitude() {
            return latitude;
        }

        float getLongitude() {
            return longitude;
        }
    }

    private class RunLocation implements Runnable {
        private Location location;

        RunLocation(Location loc) {
            location = loc;
        }

        @Override
        public void run() {
            tvLat1.setText(String.valueOf(Double.valueOf(location.getLatitude())));
            tvLon1.setText(String.valueOf(Double.valueOf(location.getLongitude())));
        }
    }

    private class RunAccelValues implements Runnable {
        private SensorEvent event;

        RunAccelValues(SensorEvent event) {
            this.event = event;
        }

        @Override
        public void run() {

            System.arraycopy(event.values, 0, aValues, 0, 3);
            if(SensorManager.getRotationMatrix(rotationValues, null, aValues, magnetValues)) {
                SensorManager.getOrientation(rotationValues, magnetAngle);
            }

            final float alpha = (float) 0.8;
            g[0] = alpha * g[0] + (1 - alpha) * event.values[0];
            g[1] = alpha * g[1] + (1 - alpha) * event.values[1];
            g[2] = alpha * g[2] + (1 - alpha) * event.values[2];
            pastAValue[0] = event.values[0] - g[0];
            pastAValue[1] = event.values[1] - g[1];
            pastAValue[2] = event.values[2] - g[2];
            if (pastAValue[0] > 0.01 || pastAValue[1] > 0.01) {
                synchronized (this) {
                    accelReadings.add(new AccelValue(pastAValue[0], pastAValue[1], pastAValue[2], event.timestamp));
                }
            }
        }
    }

    private class GyroscopeTask implements Runnable {
        private SensorEvent event;

        GyroscopeTask(SensorEvent event) {
            this.event = event;
        }

        @Override
        public void run() {
            if (magnetAngle == null)
                return;
            if(isStarted) {
                float[] initMatrix;
                initMatrix = getRotationMatrixFromOrientation(magnetAngle);
                float[] test = new float[3];
                SensorManager.getOrientation(initMatrix, test);
                gyroAll = matrixMultiplication(gyroAll, initMatrix);
                isStarted = false;
            }

            float[] deltaVector = new float[4];
            if(time != 0) {
                final float dT = (event.timestamp - time) * NS2S;
                System.arraycopy(event.values, 0, gyroscopes, 0, 3);
                getRotationVectorFromGyro(gyroscopes, deltaVector, dT / 2.0f);
            }
            time = event.timestamp;
            float[] deltaMatrix = new float[9];
            SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);
            gyroAll = matrixMultiplication(gyroAll, deltaMatrix);
            SensorManager.getOrientation(gyroAll, gyroscopeAngle);
        }
    }

    private class AzimuthTask extends TimerTask {
        public void run() {
            h2.sendEmptyMessage(0);
        }
    }

    private class DistanceTask extends TimerTask {
        public void run() {
            h1.sendEmptyMessage(0);
        }
    }

    @Override
    public void onLocationChanged(Location location) {
        locActual = location;
        if (location.getProvider().equals(LocationManager.GPS_PROVIDER)){
            gpsLoc = location;
        } else {
            netLoc = location;
        }

        if (gpsLoc != null && netLoc != null) {
            if (gpsLoc.getTime() > netLoc.getTime()) {
                locActual = gpsLoc;
            } else{
                locActual = netLoc;
            }
        }

        if (newLat1 == null || newLat1.equals("")) {
            newLat1 = String.valueOf(Double.valueOf(location.getLatitude()));
            newLon1 = String.valueOf(Double.valueOf(location.getLongitude()));
        }
        RunLocation runLocation = new RunLocation(location);
        myHandler_.post(runLocation);
    }

    @Override
    public void onStatusChanged(String s, int i, Bundle bundle) {

    }

    @Override
    public void onProviderEnabled(String s) {

    }

    @Override
    public void onProviderDisabled(String s) {

    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        if (sensorEvent.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            RunAccelValues runAccelValues = new RunAccelValues(sensorEvent);
            myHandler_.post(runAccelValues);
        } else if (sensorEvent.sensor.getType() == Sensor.TYPE_GYROSCOPE){
            GyroscopeTask gyroscopeTask = new GyroscopeTask(sensorEvent);
            myHandler_.post(gyroscopeTask);
        } else if(sensorEvent.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            System.arraycopy(sensorEvent.values, 0, magnetValues, 0, 3);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

}
