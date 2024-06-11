import numpy as np
import math
from scipy.spatial.transform import Rotation

# # first_frame的位姿真值 w.r.t first frame
# first_frame_input = [0.0000442, -0.000319982, 0.256115] # relative xyz
# # obj的位姿真值 w.r.t first frame
# obj_input = [57.717701, 21.956001, -7.200460] # relative xyz 
# # 和PointPosWorld几乎一致，但是和输出的变量差距较大


def Trans(traj, R_input, t_input, Transform, savepath, flag):
    Rot_base = Rotation.from_matrix(R_input)
    M_base = Rot_base.as_matrix()
    t_base = np.array(t_input)
    frames = traj.shape[0]
    print(frames)

    for i in range(0, frames):
        t = np.array([0.000, 0.000, 0.000])
        t[0], t[1], t[2] = traj[i][3], traj[i][7], traj[i][11]
        
        r = [[0, 0, 1], [0, 1, 0], [0, 0, 1]]
        r[0][0] = traj[i][0]
        r[0][1] = traj[i][1]
        r[0][2] = traj[i][2]
        r[1][0] = traj[i][4]
        r[1][1] = traj[i][5]
        r[1][2] = traj[i][6]
        r[2][0] = traj[i][8]
        r[2][1] = traj[i][9]
        r[2][2] = traj[i][10]
        Rot = Rotation.from_matrix(r)
        M = Rot.as_matrix()
        print(t,35)
        
        t = Transform.dot(t)
        M = Transform.dot(M)
        
        t = M_base.dot(t) + t_base
        M = M_base.dot(M)
        
        traj[i][3] = t[0]
        traj[i][7] = t[1]
        traj[i][11] = t[2]
        
        # Rot = Rotation.from_matrix(M)
        euler=Rot.as_euler("xyz",degrees=True)
        print(t,49)
        if flag == 1: # 写入真值，不做变形0
            traj[i][0], traj[i][1], traj[i][2] = M[0][0], M[0][1], M[0][2]
            traj[i][4], traj[i][5], traj[i][6] = M[1][0], M[1][1], M[1][2]
            traj[i][8], traj[i][9], traj[i][10] = M[2][0], M[2][1], M[2][2]
        if flag ==2: # 写入预测值，需要做对齐
            traj[i][0], traj[i][1], traj[i][2] = M[0][2], M[0][0], M[0][1] # ZXY
            traj[i][4], traj[i][5], traj[i][6] = M[1][2], M[1][0], M[1][1]
            traj[i][8], traj[i][9], traj[i][10] = M[2][2], M[2][0], M[2][1]
        if flag ==3: # 写入真值，需要做对齐
            traj[i][0], traj[i][1], traj[i][2] = M[0][1], M[0][2], M[0][0] # YZX
            traj[i][4], traj[i][5], traj[i][6] = M[1][1], M[1][2], M[1][0]
            traj[i][8], traj[i][9], traj[i][10] = M[2][1], M[2][2], M[2][0]
    np.savetxt(savepath, traj)

def Calc_error(gt_file, pred_file, savepath):
    gt_traj = np.loadtxt(gt_file)
    pre_traj = np.loadtxt(pred_file) 
    frames = gt_traj.shape[0]
    error = np.zeros((frames,12))
    for i in range(0, frames):
        error[i][3] = pre_traj[i][3] - gt_traj[i][3]
        error[i][7] = pre_traj[i][7] - gt_traj[i][7]
        error[i][11] = pre_traj[i][11] - gt_traj[i][11]
        # 朝向位姿用来占位
        error[i][0], error[i][1], error[i][2] = pre_traj[i][0] - gt_traj[i][0], pre_traj[i][1] -gt_traj[i][1], pre_traj[i][2] - gt_traj[i][2]
        error[i][4], error[i][5], error[i][6] = pre_traj[i][4] - gt_traj[i][4], pre_traj[i][5] -gt_traj[i][5], pre_traj[i][6] - gt_traj[i][6]
        error[i][8], error[i][9], error[i][10] = pre_traj[i][8] - gt_traj[i][8], pre_traj[i][9] - gt_traj[i][9], pre_traj[i][10] - gt_traj[i][10]
    np.savetxt(savepath, error)	

no_Transform = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
Transform_airsim = np.array([[0, 0, 1],[1, 0, 0],[0, 1, 0]]) # Airsim
# Transform_blender = np.array([[1, 0, 0],[0, -1, 0],[0, 0, -1]]) # blender

# first_frame relative pose
# 填入traj预测值第一行数据 
R_input_pred = [[1.000000000, 0.000000000, 0.000000000], 
		 [0.000000000, 1.000000000, 0.000000000], 
		 [0.000000000, 0.000000000, 1.000000000]] # 3-D Euclidean space
t_input_pred = [0.000000000,0.000000000,0.000000000] # x, y, z

# 填入traj真值第一行数据
R_input_gt = [[9.997913370276019229e-01,1.578318298970633185e-03,-2.036642619379407104e-02],
	       [-1.019778333341974296e-03,9.996238691431097445e-01,2.740584411234222098e-02],
	       [2.040202089771925298e-02,-2.737935628728862050e-02,9.994168941900986036e-01]]
t_input_gt = [-8.901740000000000000e-03,-2.690090000000000016e-03,5.365529999999999466e-01]

path_pred = 'SePT01/SePT01_CameraTrajectory'
traj_pred = np.loadtxt(path_pred + '.txt') # pred_res.txt
path_gt = 'SePT01/data_cropped'
traj_gt = np.loadtxt(path_gt + '.kitti')
save_trans = 'SePT01/traj_trans.kitti'           # SLAM to Airsim (预测值，用于评价相机轨迹结果)
save_trans_obj = 'SePT01/traj_trans_obj.kitti'   # Arisim to SLAM (真值，用于评价相对于物体的轨迹结果)
# 将SLAM预测值转到Airsim系并做rpy转系(to ZXY)
Trans(traj_pred, R_input_gt, t_input_gt, Transform_airsim, save_trans, flag=2) # flag=2转系ZXY(相机为中心)
print("<<<<<<<<<<<<<< traj_trans.kitti >>>>>>>>>>>>>>>>")
# # 将Airsim真值转到SLAM系(to YZX)
# Transform_airsim = np.transpose(Transform_airsim)
# R_input_pred = np.transpose(R_input_pred)
# t_input_pred = np.transpose(t_input_pred)
# Trans(traj_gt, R_input_pred, t_input_pred, Transform_airsim, save_trans_obj, flag=3) # flag=3转系YZX(目标为中心)
# print("<<<<<<<<<<<<<< traj_trans_obj.kitti >>>>>>>>>>>>>>>>")
