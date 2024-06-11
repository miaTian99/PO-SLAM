import numpy as np
import math
from scipy.spatial.transform import Rotation

def Trans(traj, R_input, t_input, Transform, flag):
    Rot_base = Rotation.from_matrix(R_input)
    M_base = Rot_base.as_matrix()
    t_base = np.array(t_input)
    frames = traj.shape[0]
    print(frames)

    for i in range(0, frames):
        t = np.array([0.000, 0.000, 0.000])
        r = [[0, 0, 1], [0, 1, 0], [0, 0, 1]]
        t[0], t[1], t[2] = traj[i][3], traj[i][7], traj[i][11]
        r[0][0], r[0][1], r[0][2] = traj[i][0], traj[i][1], traj[i][2]
        r[1][0], r[1][1], r[1][2] = traj[i][4], traj[i][5], traj[i][6]
        r[2][0], r[2][1] ,r[2][2] = traj[i][8], traj[i][9], traj[i][10]
        Rot = Rotation.from_matrix(r)
        M = Rot.as_matrix()
        print(t,21)

        if flag == 1: # 处理Tbo_gt = Twb(traj).inv*Two
            T1 = np.column_stack((M, t))
            T1 = np.vstack((T1, np.array([0, 0, 0, 1])))
            T2 = np.column_stack((R_input, t_input))
            T2 = np.vstack((T2, np.array([0, 0, 0, 1])))
            Tbo_gt = np.linalg.inv(T1).dot(T2) 
            M = Tbo_gt[:3,:3]
            t = Tbo_gt[:3,3]
            traj[i][0], traj[i][1], traj[i][2] = M[0][0], M[0][1], M[0][2]
            traj[i][4], traj[i][5], traj[i][6] = M[1][0], M[1][1], M[1][2]
            traj[i][8], traj[i][9], traj[i][10] = M[2][0], M[2][1], M[2][2]
            
        if flag == 2: # 处理Tbo_obj = Transform_airsim*Tco(traj)
            t = Transform.dot(t)
            M = Transform.dot(M)
            traj[i][0], traj[i][1], traj[i][2] = M[0][2], M[0][0], M[0][1] # ZXY
            traj[i][4], traj[i][5], traj[i][6] = M[1][2], M[1][0], M[1][1]
            traj[i][8], traj[i][9], traj[i][10] = M[2][2], M[2][0], M[2][1]
        
        traj[i][3], traj[i][7], traj[i][11] = t[0], t[1], t[2]
        print(t,38)
    return traj

def Calc_error(gt_file, pred_file):
    gt_traj = np.loadtxt(gt_file)
    pre_traj = np.loadtxt(pred_file) 
    frames = gt_traj.shape[0]
    error = np.zeros((frames,12))
    for i in range(0, frames):
        # print("pre_traj"+str(i)+":", [pre_traj[i][3], pre_traj[i][7], pre_traj[i][11]])
        # print("gt_traj"+str(i)+":", [gt_traj[i][3], gt_traj[i][7], gt_traj[i][11]])
        error[i][3] = pre_traj[i][3] - gt_traj[i][3]
        error[i][7] = pre_traj[i][7] - gt_traj[i][7]
        error[i][11] = pre_traj[i][11] - gt_traj[i][11]
        # error[i][3] = abs(pre_traj[i][3] - gt_traj[i][3])
        # error[i][7] = abs(pre_traj[i][7] - gt_traj[i][7])
        # error[i][11] = abs(pre_traj[i][11] - gt_traj[i][11])
        # 朝向位姿用来占位
        error[i][0], error[i][1], error[i][2] = 0, 0, 0
        error[i][4], error[i][5], error[i][6] = 0, 0, 0
        error[i][8], error[i][9], error[i][10] = 0, 0, 0
        print("error"+str(i)+":", [error[i][3], error[i][7], error[i][11]])
    return error

if __name__ == '__main__':
    # obj的位姿真值 w.r.t Airsim世界坐标系下的物体位姿真值
    obj_input = [91.226303, -61.946350, -5.531830] # absolute xyz of rock1
    path_gt = 'SePT01/data_cropped'
    traj_rover_gt = np.loadtxt(path_gt + '.kitti')    # Airsim世界坐标系下的小车轨迹位姿真值 
    path_poses = 'SePT01/SePT01_ObjectRelativePoses'
    traj_obj = np.loadtxt(path_poses + '.txt')        # SLAM世界坐标系下相对于物体的轨迹位姿预测值
    save_poses = path_poses + '.kitti'       # Airsim世界坐标系下的相对于物体的轨迹位姿真值 
    save_gt = 'SePT01/gt_obj_rela.kitti'
    save_error = 'SePT01/error_obj_rela.kitti'

    no_Transform = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
    Transform_airsim = np.array([[0, 0, 1],[1, 0, 0],[0, 1, 0]]) # Airsim(SLAM世界坐标系->AirSim世界坐标系)
    
    # Step1: 新建一个4维正交矩阵Two，并将obj_input赋值给Two第四列
    # 物体位姿真值 Two = [I, obj_input]
    R_input_pred = [[1.000000000, 0.000000000, 0.000000000], 
		 [0.000000000, 1.000000000, 0.000000000], 
		 [0.000000000, 0.000000000, 1.000000000]] # 3-D Euclidean space
    t_input_pred = [0.000000000,0.000000000,0.000000000] # x, y, z
    t_input_pred2 = np.array(t_input_pred) + np.array(obj_input) 

    # Step2: 对真值traj_cam_gt的每一项都点乘以Two
    # Tbo = Tbw*Two
    Tbo_gt = Trans(traj_rover_gt, R_input_pred, t_input_pred2, Transform_airsim, flag=1) 
    np.savetxt(save_gt, Tbo_gt)
    print("<<<<<<<<<<<<<<<<<<<<<<< gt finish >>>>>>>>>>>>>>>>>>>>>>>>")

    # Step3: 对预测值的traj_obj每一项都做转系 (转到airsim的世界坐标下)
    Tbo_obj = Trans(traj_obj, R_input_pred, R_input_pred, Transform_airsim, flag=2)
    np.savetxt(save_poses, Tbo_obj)
    print("<<<<<<<<<<<<<<<<<<<<<<< pred finish >>>>>>>>>>>>>>>>>>>>>>>>")

    # Step5: 计算相对轨迹误差
    error = Calc_error(save_gt, save_poses)
    np.savetxt(save_error, error)	



