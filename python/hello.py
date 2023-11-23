import numpy as np
import math

# Define standard deviation for lat, long. 3.0 as per datasheet.
latlonSD = 6.0

altSD = 10.0
dest_lat = 3518.5730
dest_lat_dir = 'N'
dest_lon = 8044.5298
dest_lon_dir = 'W'
dest_alt = 199.80

# Earths gravity
GRAVITY = 9.80665
# Radius in meters
EARTH_RADIUS = 6371 * 1000.0
# Accelerometer standard deviation received after few readings from IMU.
accNSD = GRAVITY * 0.367107577589
accESD = GRAVITY * 0.313021102282
accUSD = GRAVITY * 0.376713286794


# /////////////////////////////////////////////////////////////////////////////

def RadiansToDegrees(rad):
    return float(rad * 180.0 / math.pi)


class Kalman():
    def __init__(self, initPos, initVel, gpsSD, accSD, time):
        # Store time.
        self.time = time
        # Matrix to store current state.
        self.currentstate = np.array([[initPos], [initVel]], dtype=np.float64)
        # Error variance matrix for accelerometer.
        self.Q = np.array([[float(accSD * accSD), 0], [0, float(accSD * accSD)]], dtype=np.float64)
        # Error variance matrix for gps.
        self.R = np.array([[float(gpsSD * gpsSD), 0], [0, float(gpsSD * gpsSD)]], dtype=np.float64)

        # transformation matrix for input data
        self.H = np.identity(2)
        # initial guess for covariance
        self.P = np.identity(2)
        # identity matrix
        self.I = np.identity(2)

        # accelerometer input matrix
        self.u = np.zeros([1, 1], dtype=np.float64)
        # gps input matrix
        self.z = np.zeros([2, 1], dtype=np.float64)
        # State transition matrix
        self.A = np.zeros([2, 2], dtype=np.float64)
        # Control matrix
        self.B = np.zeros([2, 1], dtype=np.float64)

    def predict(self, acc, currtime):
        dTime = self.time - currtime
        self.updateControlMatrix(dTime)
        self.updateStateMatrix(dTime)
        self.updateAccInputMatrix(acc)

        temp1 = np.dot(self.A, self.currentstate)
        temp2 = np.dot(self.B, self.u)
        self.currentstate = np.add(temp1, temp2)

        temp3 = np.dot(self.A, self.P)
        temp4 = np.dot(temp3, self.A.transpose())
        self.P = np.add(temp4, self.Q)

        self.updateTime(currtime)

    def update(self, pos, vel, posError, velError):
        self.z[0, 0] = float(pos)
        self.z[1, 0] = float(vel)

        if posError != None:
            self.R[0, 0] = posError * posError

        self.R[1, 1] = velError * velError

        y = np.subtract(self.z, self.currentstate)
        s = np.add(self.P, self.R)
        inv = np.linalg.inv(s)
        K = np.dot(self.P, inv)

        temp = np.dot(K, y)
        self.currentstate = np.add(self.currentstate, temp)

        temp2 = np.subtract(self.I, K)
        self.P = np.dot(temp2, self.P)

    def updateControlMatrix(self, dTime):
        Time = float(dTime)
        self.B[0, 0] = 0.5 * Time * Time
        self.B[1, 0] = Time

    def updateStateMatrix(self, dTime):
        self.A[0, 0] = 1.0
        self.A[0, 1] = float(dTime)
        self.A[1, 0] = 0.0
        self.A[1, 1] = 1.0

    def updateAccInputMatrix(self, acc):
        self.u[0, 0] = float(acc)

    def updateTime(self, currtime):
        self.time = currtime

    def getPredictedPosition(self):
        return self.currentstate[0, 0]

    def getPredictedVelocity(self):
        return self.currentstate[1, 0]


def degreeToRadians(deg):
    return float(deg * math.pi / 180.0)


def getDistanceM(lat1, lon1, lat2, lon2):
    dlon = degreeToRadians(lon2 - lon1)
    dlat = degreeToRadians(lat2 - lat1)

    a = math.pow(math.sin(dlat / 2.0), 2) + math.cos(degreeToRadians(lat1)) * math.cos(
        degreeToRadians(lat2)) * math.pow(math.sin(dlon / 2.0), 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS * c


# Convert lat and long points from degrees to meters using the haversine formula.
def LatLonToM(lat, lon):
    latdis = getDistanceM(lat, 0.0, 0.0, 0.0)
    londis = getDistanceM(0.0, lon, 0.0, 0.0)
    if lat < 0: latdis *= -1
    if lon < 0: londis *= -1
    return latdis, londis


# Based on the current lat, long, calculate point ahead at distance 'dist' and angle 'azimuth'
def getPointAhead(lat, lon, dist, azimuth):
    radiusFraction = float(dist / EARTH_RADIUS)

    bearing = degreeToRadians(azimuth)

    lat1 = degreeToRadians(lat)
    lng1 = degreeToRadians(lon)

    lat2_part1 = math.sin(lat1) * math.cos(radiusFraction)
    lat2_part2 = math.cos(lat1) * math.sin(radiusFraction) * math.cos(bearing)
    lat2 = math.asin(lat2_part1 + lat2_part2)

    lng2_part1 = math.sin(bearing) * math.sin(radiusFraction) * math.cos(lat1)
    lng2_part2 = math.cos(radiusFraction) - (math.sin(lat1) * math.sin(lat2))

    lng2 = lng1 + math.atan2(lng2_part1, lng2_part2)
    lng2 = (lng2 + 3 * math.pi) % (2 * math.pi) - math.pi

    return (RadiansToDegrees(lat2), RadiansToDegrees(lng2))


def metersToGeopoint(latM, lonM):
    lat, lon = 0.0, 0.0
    # Get point at East
    elat, elon = getPointAhead(lat, lon, lonM, 90.0)
    # Get point at NE
    nelat, nelon = getPointAhead(elat, elon, latM, 0.0)
    return nelat, nelon


def init_data(la, lo, time_gps):
    lat, lon = LatLonToM(la, lo)
    timestamp = float(time_gps[0]) * 3600 + float(time_gps[1]) * 60 + float(time_gps[2])
    latKalman = Kalman(lat, 0.0, latlonSD, accNSD, timestamp)
    lonKalman = Kalman(lon, 0.0, latlonSD, accESD, timestamp)
    print(timestamp)
    # print("py_time:",time[0]," ",time[1]," ",time[2],"\r\n")
    print("python", "lat:", lat, "lon:", lon, "\r\n")

    return latKalman, lonKalman


def main(pyv, la, lo, abs_north_acc, abs_east_acc, time_imu):
    # print("main:", pyv, "1:", abs_north_acc, "2:", abs_east_acc, "3:", pyv[4], "\r\n")

    # print("main_lat:", la, "main_lon:", lo, "\r\n")

    # print("py_time:", time_imu[0], ":", time_imu[1], ":", round(time_imu[2], 3), "\r")
    timestamp = float(time_imu[0]) * 3600 + float(time_imu[1]) * 60 + float(round(time_imu[2], 3))

    pyv[0].predict(GRAVITY * float(abs_north_acc), timestamp)  # ###
    pyv[1].predict(GRAVITY * float(abs_east_acc), timestamp)  # ###

    if la != 0.0:
        lat, lon = LatLonToM(la, lo)
        pyv[0].update(lat, 0.0, None, 0.0)
        pyv[1].update(lon, 0.0, None, 0.0)

    pLatM = pyv[0].getPredictedPosition()
    pLonM = pyv[1].getPredictedPosition()
    pLat, pLon = metersToGeopoint(pLatM, pLonM)

    if la != 0.0:
        print("out:", pLon, " ", pLat, "\r\n")
        f1 = open('/home/root123/project/cs/python/输出数据.txt', 'a')
        f1.write(str(pLon) + ',' + str(pLat) + ',' + str(lo) + ',' + str(la))
        f1.write("\n")
        f1.close()


def Hello():
    print("Hello, World!")
