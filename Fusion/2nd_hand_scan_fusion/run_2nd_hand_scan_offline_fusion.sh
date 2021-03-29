#!/bin/bash

##usage: pi_map_fusion.py [-h] [-th THRESHOLD_SONAR]
##                        [-in_map_mode CARTOGRAPHER_MAP_MODE] [-inpath IN_PATH]
##                        [-outpath OUT_PATH] [-lfile LIDAR_FILE_STR]
##                        [-sfile SONAR_FILE_STR] [-pfile LIDAR_POSE_STR]
##
##optional arguments:
##  -h, --help            show this help message and exit
##  -th THRESHOLD_SONAR   threshold for sonar filter, quantification for
##                        OccupancyGrid. Required: > 0.5
##  -in_map_mode CARTOGRAPHER_MAP_MODE
##                        the mode of input image, 1: from cartographer map
##                        saver. 0: from ros map saver
##  -inpath IN_PATH       path name of input files
##  -outpath OUT_PATH     path name of output files
##  -lfile LIDAR_FILE_STR
##                        file name of lidar source data and original position,
##                        .pgm and .yaml
##  -sfile SONAR_FILE_STR
##                        file name of sonar source data, .txt
##  -pfile LIDAR_POSE_STR
##                        file name of lidar pose, .txt


# not use this one
#TAG_NAME="0807_SZU_L6F11_1m3_01"
#VERSION="0820_SZU_L6F11_01_V5_LJ_offline"

########### 1
# filtered the first small room, and need chop raw sonar scan and raw sonar pose files
#TAG_NAME="0807_SZU_L6F11_1m3_02"
#VERSION="0913_SZU_L6F11_02_NY_FILTER_offline"
# bagfile: 0807_SZU_L6F11_1m3_02_2019-08-07-15-46-00_1st_room_filter.bag

########### 2
#TAG_NAME="0807_SZU_L6F11_1m3_06"
#VERSION="0824_SZU_L6F11_06_V3_LJ_offline"

########### 3
#TAG_NAME="0807_SZU_L6F02_1m3_08"
#VERSION="0827_SZU_L6F02_08_V4_LJ_offline"

########### 4
TAG_NAME="0809_A4_F2_08"
VERSION="0819_A4_F2_08_V2_LJ_offline"

########### 5
#TAG_NAME="0806_pi_office_1m3_07_tf_inter"
#VERSION="0914_pi_office_07_NY_offline"

########### 6
# actually this is what I called L6F11_1m3_00
#TAG_NAME="0806_SZU_L6F11_1m3"
#VERSION="0901_SZU_L6F11_00_V1_NY_offline"
#bagfile: 0807_SZU_L6F11_1m3_2019-08-07-14-55-24.bag

########### 7
#TAG_NAME="0805_line_Lobby_02"
#VERSION="0902_line_Lobby_02_NY_offline"
# bagfile: 2019-08-05-16-56-36_0805_line_Lobby_02.bag

########### 8
#TAG_NAME="0805_line_Lobby_03"
#VERSION="0903_line_Lobby_03_NY_offline"
# bagfile: 2019-08-05-18-00-50_0805_line_Lobby_03.bag






# example 1:
#python pi_map_fusion.py -th 0.6 -in_map_mode 0 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/ -lfile 0311_corido -sfile 0311_corido_sonar_2d_ogm -pfile 0311_corido_our_lidar_data -outpath ./outputs/

# example 2:
#python pi_map_fusion.py -th 0.6 -in_map_mode 0 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/ -lfile office_0319_0m8_large_t2 -sfile office_0319_0m8_large_sonar_2d_ogm -pfile office_0319_0m8_large_our_lidar_data -outpath ./outputs/

#0408
#python pi_map_sonar_display.py -th 0.6 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0408_pioneer_sonar_wheel_pose/ -sfile 0408_sonar_2d_ogm_p3dx_pi_office_1m5  -outpath ./outputs/

#0409
#python pi_map_sonar_display.py -th 0.6 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0409_pioneer_sonar_wheel_pose/ -sfile 0409_corridor_sonar_2d_ogm_1m3_longpath -outpath ./outputs/

#0410
#python pi_map_sonar_display.py -th 0.6 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0410_pioneer_sonar_zte/ -sfile 0410_zte_straitline_sonar_2d_ogm -outpath ./outputs/

#0411
#python pi_map_fusion.py -th 0.6 -in_map_mode 0 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0412_pioneer_lidar_sonar/ -lfile 0412_pi_hallway_02_t2  -sfile 0412_pi_hallway_02_t2_lidarpose_sonar_2d_ogm -pfile offic -outpath ./outputs/

#0415
#python pi_map_fusion.py -th 0.6 -in_map_mode 0 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0415_pioneer_lidar_sonar/ -lfile 0415_A4_lobby_short_t2_03  -sfile 0415_A4_lobby_short_t2_03_sonar_2d_ogm  -pfile 0415_A4_lobby_short_t2_03_our_lidar_data  -outpath ./outputs/

#0430
#python pi_map_fusion.py -th 0.99 -in_map_mode 0 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0430_pioneer_lidar_sonar/ -lfile 0430_pi_office_bayesian  -sfile 0430_pi_office_bayesian_sonar_2d_ogm  -pfile 0430_pi_office_bayesian_our_lidar_data  -outpath ./outputs/

#0507 
#python pi_map_fusion.py -th 0.6 -in_map_mode 0 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0507_pioneer_bayes/ -lfile 0507_corridor_bayesian_1m3_19  -sfile 0507_corridor_bayesian_1m3_19_sonar_2d_ogm  -pfile 0507_corridor_bayesian_1m3_19_our_lidar_data  -outpath ./outputs/

#0517
#python pi_map_fusion.py -th 0.6 -in_map_mode 0 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0517_pioneer_DS/ -lfile 0517_F2_direct_line_1m3_04  -sfile 0517_F2_direct_line_1m3_04_sonar_2d_ogm  -pfile 0517_F2_direct_line_1m3_04_our_lidar_data  -outpath ./outputs/

#0519
#python pi_map_fusion.py -th 0.6 -in_map_mode 0 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0519_from_May/ -lfile 0519_F2_DS_1m4_18  -sfile 0519_F2_DS_1m4_18_sonar_2d_ogm  -pfile 0519_F2_DS_1m4_18_our_lidar_data  -outpath ./outputs/

#0522
#python pi_map_fusion.py -th 0.6 -in_map_mode 0 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0519_from_May/ -lfile 0519_F2_DS_1m4_18  -sfile 0519_F2_DS_1m4_18_sonar_2d_ogm  -pfile 0519_F2_DS_1m4_18_our_lidar_data  -outpath ./outputs/

#python 2nd_hand_scan_offline_fusion.py -inpath /home/ubuntu/Documents/Tools/scripts/good_outputs/ -lfile 0519_F2_direct_line_1m4_10_lidar_raw -sfile 0519_F2_direct_line_1m4_10_sonar_filter -pfile 0519_F2_direct_line_1m4_10_our_lidar_data -yfile 0519_F2_direct_line_1m4_10 -mfile hd_maps -outpath ./outputs/

#0730

#0805
#python 2nd_hand_scan_offline_fusion.py -inpath /home/ubuntu/Documents/Tools/scripts/outputs_July/ -inpath2 /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0805_line_Lobby_02_lidar_raw -sfile 0805_line_Lobby_02_sonar_filter -pfile 0805_line_Lobby_02_our_lidar_data -yfile 0805_line_Lobby_02  -mfile hd_maps -outpath ./outputs/


#0808
#python 2nd_hand_scan_offline_fusion.py -inpath /home/ubuntu/Documents/Tools/scripts/outputs_July/ -inpath2 /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0807_SZU_L6F02_1m3_08_lidar_raw -sfile 0807_SZU_L6F02_1m3_08_sonar_2d_ogm_offline3_sonar_filter -pfile 0807_SZU_L6F02_1m3_08_Final_GloPo_TX -yfile 0807_SZU_L6F02_1m3_08 -mfile hd_maps -outpath ./outputs/ -traj_mode 0

#python 2nd_hand_scan_offline_fusion.py -inpath /home/ubuntu/Documents/Tools/scripts/outputs_July/ -inpath2 /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0807_SZU_L6F02_1m3_08_lidar_raw -sfile 0807_SZU_L6F02_1m3_08_sonar_2d_ogm_sonar_filter -pfile 0807_SZU_L6F02_1m3_08_our_lidar_data -yfile 0807_SZU_L6F02_1m3_08  -mfile hd_maps -outpath ./outputs/


# traj_mode: 
# 0: original local pose
# 1: final node pose
# 2: final node interpolated pose
#python 2nd_hand_scan_offline_fusion.py -inpath /home/ubuntu/Documents/Tools/scripts/outputs_July/ -inpath2 /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0807_SZU_L6F02_1m3_08_lidar_raw -sfile 0807_SZU_L6F02_1m3_08_sonar_2d_ogm_offline3_sonar_filter -pfile 0807_SZU_L6F02_1m3_08_Final_GloPo_TX -yfile 0807_SZU_L6F02_1m3_08 -mfile hd_maps -outpath ./outputs/ -traj_mode 0


# traj_mode: 
# 0: original local pose
# 1: final node pose
# 2: final node interpolated pose
#python2 2nd_hand_scan_offline_fusion.py -inpath ~/Documents/Tools/scripts/outputs_July/ -inpath2 ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0820_SZU_L6F11_01_V5_LJ_offline_lidar_raw -sfile 0820_SZU_L6F11_01_V5_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_Bay_sonar_filter -pfile 0820_SZU_L6F11_01_V5_LJ_offline_Final_GloPo_Interpo -yfile 0807_SZU_L6F11_1m3_01 -mfile hd_maps -outpath ./outputs/ -traj_mode 2


###########################################################################
#--------------------------------------------------------------------------
# From here, These are really important runnings, go to the paper.
#--------------------------------------------------------------------------
# traj_mode: 
# 0: original local pose
# 1: final node pose
# 2: final node interpolated pose
#python2 2nd_hand_scan_offline_fusion.py -inpath ~/Documents/Tools/scripts/outputs_July/ -inpath2 ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0824_SZU_L6F11_06_V3_LJ_offline_lidar_raw -sfile 0824_SZU_L6F11_06_V3_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_Bay_sonar_filter -pfile 0824_SZU_L6F11_06_V3_LJ_offline_Final_GloPo_Interpo -yfile 0824_SZU_L6F11_06_V3_LJ_offline -mfile hd_maps -outpath ./outputs/ -traj_mode 2



# traj_mode: 
# 0: original local pose
# 1: final node pose
# 2: final node interpolated pose
#python2 2nd_hand_scan_offline_fusion.py -inpath ~/Documents/Tools/scripts/outputs_July/ -inpath2 ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0824_SZU_L6F11_06_V3_LJ_offline_lidar_raw -sfile 0824_SZU_L6F11_06_V3_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_Bay_2S_sonar_filter -pfile 0824_SZU_L6F11_06_V3_LJ_offline_Final_GloPo_Interpo -yfile 0824_SZU_L6F11_06_V3_LJ_offline -mfile hd_maps -outpath ./outputs/ -traj_mode 2


#0906 report

# traj_mode: 
# 0: original local pose
# 1: final node pose
# 2: final node interpolated pose
#python2 2nd_hand_scan_offline_fusion.py -inpath ~/Documents/Tools/scripts/outputs_July/ -inpath2 ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0820_SZU_L6F11_01_V5_LJ_offline_lidar_raw -sfile xxxxxxxL6F11_06_V3_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_Bay_sonar_filter -pfile 0824_SZU_L6F11_06_V3_LJ_offline_Final_GloPo_Interpo -yfile 0824_SZU_L6F11_06_V3_LJ_offline -mfile hd_maps -outpath ./outputs/ -traj_mode 2


#0912
#TAG_NAME="0807_SZU_L6F02_1m3_08"
#VERSION="0827_SZU_L6F02_08_V4_LJ_offline"

#TAG_NAME="0809_A4_F2_08"
#VERSION="0819_A4_F2_08_V2_LJ_offline"

###### type: 
###### "LPF": local pose,
###### "PGPF": partial global pose, 
###### "FGPF": final global pose,
###### "FGPFINTP": final global pose interpolation. Extra file will be gen

###### "CLD" -- "DIRECT_CENTRAL_LINE_ALGORITHM"
###### "CoD" -- "DIRECT_CONE_ALGORITHM"
###### "Bay" -- "BAYESIAN_ALGORITHM"
###### "DS"  -- "DS_ALGORITHM"
# traj_mode: 
# 0: original local pose
# 1: final node pose
# 2: final node interpolated pose
#python2 2nd_hand_scan_offline_fusion.py -inpath ~/Documents/Tools/scripts/outputs_July/ -inpath2 ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile ${VERSION}_lidar_raw -sfile ${VERSION}_sonar_2d_ogm_offline_FGPFINTP_Bay_sonar_filter -pfile ${VERSION}_Final_GloPo_Interpo -yfile ${VERSION} -mfile hd_maps -outpath ./outputs/ -traj_mode 2



python2 RVS_2nd_hand_scan_offline_fusion_video_2_plt_backup.py -inpath ~/Documents/Tools/scripts/outputs_July/ -inpath2 ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile ${VERSION}_lidar_raw -sfile ${VERSION}_sonar_2d_ogm_offline_FGPFINTP_Bay_sonar_filter -pfile ${VERSION}_Final_GloPo_Interpo -yfile ${VERSION} -mfile hd_maps -outpath ./outputs/ -traj_mode 2

