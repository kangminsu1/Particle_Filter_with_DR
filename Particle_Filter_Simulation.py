import csv
import numpy as np
import time
import pymap3d as pm
import matplotlib.pyplot as plt
import matplotlib
from socket import *
from multiprocessing import Process, Value, Array, Manager
import math
from datetime import datetime
from sklearn.metrics import mean_squared_error

MAX_RANGE = 20.0  # maximum observation range. 높으면 높을수록 주변 ROI가 증가함

# Particle filter parameter
NP = 100  # Number of Particle
NTh = NP / 2.0  # Number of particle for re-sampling

# Estimation parameter of PF
Q = np.diag([0.2]) ** 2  # range error
R = np.diag([2.0, np.deg2rad(40.0)]) ** 2  # input error

show_animation = True

def motion_model(x, u, dt):

    # x, y, yaw, velocity
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[dt * math.cos(x[2, 0]), 0],# x
                  [dt * math.sin(x[2, 0]), 0],# y
                  [0.0, dt],# yaw
                  [1.0, 0.0]])# v

    x = F.dot(x) + B.dot(u)

    return x

def observation(dt, x_true, u, rf_id):
    z = np.zeros((0, 3))
    for i in range(len(rf_id[:, 0])):
        d = math.hypot(x_true[0, 0] - rf_id[i, 0], x_true[1, 0] - rf_id[i, 1]) # Pythagorean
        if d <= MAX_RANGE:
            zi = np.array([[d, rf_id[i, 0], rf_id[i, 1]]])
            z = np.vstack((z, zi))

    xd = motion_model(x_true, u, dt)

    return x_true, z, xd

def gauss_likelihood(x, sigma):
    p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2) * \
        math.exp(-x ** 2 / (2 * sigma ** 2))

    return p

def calc_covariance(x_est, px, pw):
    cov = np.zeros((3, 3))
    n_particle = px.shape[1]
    for i in range(n_particle):
        dx = (px[:, i:i + 1] - x_est)[0:3]
        cov += pw[0, i] * dx @ dx.T
    cov *= 1.0 / (1.0 - pw @ pw.T)

    return cov

def re_sampling(px, pw):
    w_cum = np.cumsum(pw) # 각 Weight 값을 누적합!
    base = np.arange(0.0, 1.0, 1 / NP) #[0 ... 1.0] <- 100개의 Value 생성
    re_sample_id = base + np.random.uniform(0, 1 / NP) # 난수 더하기
    indexes = []
    ind = 0
    for ip in range(NP):
        # 리생플링되는 Particle data가 누적합된 data보다 작은 것을 append
        while re_sample_id[ip] > w_cum[ind]: 
            ind += 1
        indexes.append(ind)

    px = px[:, indexes] # init particles
    pw = np.zeros((1, NP)) + 1.0 / NP  # init weight

    return px, pw

def pf_localization(dt, px, pw, z, u):
    for ip in range(NP):
        x = np.array([px[:, ip]]).T
        w = pw[0, ip] #particle weight를 하나씩 가져온다.
        #  Predict with random input sampling np.random.randn() -> 0~1 범위 내 실수 랜덤
        ud1 = u[0, 0] + np.random.randn() * R[0, 0] ** 0.5
        ud2 = u[1, 0] + np.random.randn() * R[1, 1] ** 0.5
        ud = np.array([[ud1, ud2]]).T
        # NP 번의 Prediction을 수행. 여기서 중요한건 R을 곱한 velocity, yaw rate를 사용한다는 점!
        x = motion_model(x, ud, dt) 

        # 분포가 많은 곳을 찾기 위해 Weight 정보들을 Maximun Likelihood 처리
        for i in range(len(z[:, 0])):
            pre_z = math.hypot(x[0, 0] - z[i, 1], x[1, 0] - z[i, 2])
            dz = pre_z - z[i, 0]
            w = w * gauss_likelihood(dz, math.sqrt(Q[0, 0]))

        px[:, ip] = x[:, 0]
        pw[0, ip] = w

    pw = pw / pw.sum()  # normalize

    x_est = px.dot(pw.T)
    p_est = calc_covariance(x_est, px, pw)
    N_eff = 1.0 / (pw.dot(pw.T))[0, 0]  # Effective particle number

    if N_eff < NTh:
        px, pw = re_sampling(px, pw)
    return x_est, p_est, px, pw

def imu_info_loading():
    first = False
    log_data = []
    with open('IMU.csv', 'r') as f:
        for line in f:
            if first == False:
                first = True
            elif first == True:
                data = line.split(',')
                data[-1] = data[-1].split('\n')[0]
                lists = np.array(list(map(np.float, np.array(data))))
                # time, lat, lon, h, speed, yaw, heading
                log_data.append([lists[0], lists[1], lists[2], lists[3], lists[7], lists[-2], lists[-1]])
    f.close()

    # Essential
    result_path = []
    for latlon in log_data:
        result_path.append([latlon[1], latlon[2], latlon[3]])
    result_path = np.array(result_path)

    center = sum(result_path) / len(result_path)  # Calculate ENU Center

    ENU_all = []
    for llh in result_path:
        e, n, u = pm.geodetic2enu(llh[0], llh[1], llh[2], center[0], center[1], center[2])
        ENU_all.append([e, n, u])
    ENU_all = np.array(ENU_all)

    info_all = []
    for i in range(len(log_data)):
        info_all.append([log_data[i][0], ENU_all[i][0], ENU_all[i][1], ENU_all[i][2], log_data[i][4], log_data[i][5], log_data[i][6]])
    return info_all

if __name__ == "__main__":
    matplotlib.use('TkAgg')
    # Loading IMU example Information .csv -> Lists
    info_all = imu_info_loading()

    # State Vector [x y yaw v]
    x_true = np.zeros((4, 1)) # True Measurement
    x_est = np.zeros((4, 1)) # Prediction

    px = np.zeros((4, NP))  # Particle store
    pw = np.zeros((1, NP)) + 1.0 / NP  # Particle weight

    # Results of History
    h_x_est = x_est
    h_x_true = x_true
    h_x_dr = x_true

    first = False
    start_time = datetime.now()
    # Objects.txt file open================================================= (2)
    rf_id = []
    temp_dt = 0.0
    with open('Objects.txt', 'r') as f:
        for line in f:
            data = line.split(',')
            data[-1] = data[-1].split('\n')[0]
            lists = np.array(list(map(np.float, np.array(data))))
            rf_id.append([lists[0], lists[1]])
    rf_id = np.array(rf_id)
    # Objects.txt file open================================================= (2)
    # For Create Surrounding Objects=========================================== (1)
    # objects = []
    # counts = 0
    # For Create Surrounding Objects=========================================== (1)
    toggle2 = False
    area = 50

    # For RMSE---------------------------------------
    all_time = []
    RMSE_Xp = []
    RMSE_Yp = []
    RMSE_Xd = []
    RMSE_Yd = []
    # For RMSE---------------------------------------
    for i in range(len(info_all)):
        # time.sleep(0.1)
        # For Create Surrounding Objects=========================================== (1)
        # counts += 1
        # if counts % 5 == 0:
        #     xx = round(np.random.uniform(lat.value - 10, lat.value + 10), 2)
        #     yy = round(np.random.uniform(lon.value - 10, lon.value + 10), 2)
        #     objects.append([xx,yy])
        # For Create Surrounding Objects=========================================== (1)

        dt = info_all[i][0]
        lat = info_all[i][1]
        lon = info_all[i][2]
        velocity = info_all[i][4] / 3.6 # m/s
        yaw = info_all[i][6] * math.pi / 180
        yaw_rate = info_all[i][5] # rad/s

        Real_dt = round(dt - temp_dt, 3)
        temp_dt = dt
        if Real_dt > 0.0:
            # Start Pariticle Filtering-----------------------------------
            px = np.zeros((4, NP)) + np.array([[lat, lon, yaw, 0.0]]).T
            x_true = np.array([[lat, lon, yaw, 0.0]]).T
            u = np.array([[velocity, yaw_rate]]).T

            x_true, z, x_dr = observation(Real_dt, x_true, u, rf_id)
            x_est, PEst, px, pw = pf_localization(Real_dt, px, pw, z, u)
            # Start Pariticle Filtering-----------------------------------

            # For RMSE------------------------------------------
            all_time.append(dt)
            MSE = mean_squared_error([x_true[0][0]], [x_est[0][0]])
            RMSE_Xp.append(np.sqrt(MSE))
            MSE = mean_squared_error([x_true[1][0]], [x_est[1][0]])
            RMSE_Yp.append(np.sqrt(MSE))
            MSE = mean_squared_error([x_true[0][0]], [x_dr[0][0]])
            RMSE_Xd.append(np.sqrt(MSE))
            MSE = mean_squared_error([x_true[1][0]], [x_dr[1][0]])
            RMSE_Yd.append(np.sqrt(MSE))
            # For RMSE------------------------------------------

            # Storing Data---------------------------------------
            if abs(lat) > 0:
                if first == False:
                    h_x_true = x_true
                    h_x_est = x_est
                    h_x_dr = x_dr
                    first = True
                else:
                    h_x_true = np.hstack((h_x_true, x_true))
                    h_x_est = np.hstack((h_x_est, x_est))
                    h_x_dr = np.hstack((h_x_dr, x_dr))
            # Storing Data---------------------------------------

            if show_animation:
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                for i in range(len(z[:, 0])):
                    plt.plot([x_true[0, 0], z[i, 1]], [x_true[1, 0], z[i, 2]], "--g")
                plt.plot(rf_id[:, 0], rf_id[:, 1], "*g")
                plt.plot(px[0, :], px[1, :], ".r")
                plt.plot(np.array(h_x_true[0, :]).flatten(),
                         np.array(h_x_true[1, :]).flatten(), "-b", label="Actual Trajectory")
                plt.plot(np.array(h_x_est[0, :]).flatten(),
                         np.array(h_x_est[1, :]).flatten(), "-r", label="Predicted Trajectory")
                plt.plot(np.array(h_x_dr[0, :]).flatten(),
                         np.array(h_x_dr[1, :]).flatten(), "-k", label="Dead Reckoning")

                # ROI
                # plt.xlim(lat - area, lat + area)
                # plt.ylim(lon - area, lon + area)

                end_time = datetime.now()
                DT = (end_time - start_time).seconds  # Second
                plt.title("Time(sec): " + str(DT))
                plt.legend()
                plt.grid(True)
                plt.pause(0.001)
    # For Create Surrounding Objects=========================================== (1)
    # ff = open('Objects.txt', 'w')
    # for i in range(len(objects)):
    #     strings = str(objects[i][0]) + ',' + str(objects[i][1]) + '\n'
    #     ff.write(strings)
    # ff.close()
    # objects = np.array(objects)
    # plt.cla()
    # plt.plot(objects[:, 0], objects[:, 1], "*r")
    # plt.show()
    # For Create Surrounding Objects=========================================== (1)

    # RMSE=Prediction=============================================
    plt.cla()
    plt.plot(all_time, RMSE_Xp, "ok", markersize=5, label="PF= " + str(np.round(np.mean(RMSE_Xp), 2)))
    plt.title("Root Mean Square Error")
    plt.xlabel("Time(sec)")
    plt.ylabel("Value(X)")
    plt.legend()
    plt.grid(True)
    plt.show()

    plt.cla()
    plt.plot(all_time, RMSE_Yp, "ok", markersize=5, label="PF= " + str(np.round(np.mean(RMSE_Yp), 2)))
    plt.title("Root Mean Square Error")
    plt.xlabel("Time(sec)")
    plt.ylabel("Value(Y)")
    plt.legend()
    plt.grid(True)
    plt.show()

    # RMSE=Dead Reckoning=============================================
    plt.cla()
    plt.plot(all_time, RMSE_Xd, "ok", markersize=5, label="DR= " + str(np.round(np.mean(RMSE_Xd), 2)))
    plt.title("Root Mean Square Error")
    plt.xlabel("Time(sec)")
    plt.ylabel("Value(X)")
    plt.legend()
    plt.grid(True)
    plt.show()

    plt.cla()
    plt.plot(all_time, RMSE_Yd, "ok", markersize=5, label="DR= " + str(np.round(np.mean(RMSE_Yd), 2)))
    plt.title("Root Mean Square Error")
    plt.xlabel("Time(sec)")
    plt.ylabel("Value(Y)")
    plt.legend()
    plt.grid(True)
    plt.show()

    print("main Clear")

