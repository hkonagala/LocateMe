# LocateMe

 Android application that finds your location using two methods.

Method 1: Application uses the GPS unit to find your location outdoors and network triangulation to find your location indoors.

Method 2: Application uses the GPS unit to find your location outdoors and network triangulation to find your location indoors only ONCE when the app starts. From this point onwards the application uses the accelerometer and gyroscope to find your location. The location is in the form of <latitude,longitude> duples. 

Method 2 - Algorithm:

To execute the method 2, utilized 3 sensors: Accelerometer, Gyroscope and Magnetic Field. To start with, register every one of the sensors and calculate the Azimuth angle. To do that, get the rotation matrix from accelerometer data and compass. Then from the rotation matrix, calculate the orientation.Find the rotation vector from the gyroscope readings. Again calculate the orientation using the 9-axis gyroscope data. Then we get the azimuth angle. Store them.
Also, remove the gravity from the accelerometer data using a low pass filter. Then use the haversine algorithm to calculate the distance traveled, azimuth angle and the new latitudes, longtitudes for a certain time interval (in ms). Store the values continuosly. Thus, we obtained the latitude and longtitude required for the method 2, now using the following formulae. calculate the error in distance.

# Formula to calculate distance

A distance value (d) between the lat, long calculated from the two methods. The formula for
calculating d is the following.
dlon = lon2 - lon1 

dlat = lat2 - lat1 

a = (sin(dlat/2))^2 + cos(lat1) * cos(lat2) * (sin(dlon/2))^2

c = 2 * atan2( sqrt(a), sqrt(1-a) )

d = R * c (where R is the radius of the Earth).
Where lon1 and lat1 are the latitude and longitude from Method 1, and lon2 and lat2 are the latitude and longitude from Method 2.
