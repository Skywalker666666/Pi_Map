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
TAG_NAME="0807_SZU_L6F02_1m3_08"
VERSION="0827_SZU_L6F02_08_V4_LJ_offline"

########### 4
#TAG_NAME="0809_A4_F2_08"
#VERSION="0819_A4_F2_08_V2_LJ_offline"

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

#0730
#python pi_map_fusion.py -th 0.6 -in_map_mode 0 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/   -lfile 0730_line_loop_closure_02  -sfile 0730_line_loop_closure_02_sonar_2d_ogm  -pfile 0730_line_loop_closure_02_our_lidar_data  -outpath ./outputs_July/

#python pi_map_fusion.py -th 0.6 -in_map_mode 0 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/   -lfile 0730_line_loop_closure_03  -sfile 0730_line_loop_closure_03_sonar_2d_ogm  -pfile 0730_line_loop_closure_03_our_lidar_data -outpath ./outputs_July/


#0801
#python pi_map_fusion.py -th 0.6 -in_map_mode 0 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0801_line_6Hz_lidar  -sfile 0801_line_6Hz_lidar_sonar_2d_ogm  -pfile 0801_line_6Hz_lidar_our_lidar_data -outpath ./outputs_July/

#python pi_map_fusion.py -th 0.6 -in_map_mode 0 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_online_lidar_cmp/ -lfile 0801_line_6Hz_lidar_offline  -sfile 0801_line_6Hz_lidar_sonar_2d_ogm  -pfile 0801_line_6Hz_lidar_our_lidar_data -outpath ./outputs_July/

#0803
#python pi_lidar_maps_overlapping.py -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_online_lidar_cmp/ -lfile1 0801_line_6Hz_lidar -lfile2 0801_line_6Hz_lidar_offline


#0803
#python pi_trajectory_eval.py -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -outpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0801_line_6Hz_lidar -orig_pfile Orig_LocPo_TX_1564763022495  -opti_pfile Opti_GloPo_TX_1564763022495  -fina_pfile Final_GloPo_TX_1564763022495


#python pi_trajectory_eval.py -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -outpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0730_line_loop_closure_03 -orig_pfile Orig_LocPo_TX_1564763022495  -opti_pfile Opti_GloPo_TX_1564763022495  -fina_pfile Final_GloPo_TX_1564763022495

#python pi_trajectory_eval.py -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -outpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0805_line_Lobby_01  -orig_pfile Orig_LocPo_TX  -opti_pfile  Opti_GloPo_TX -fina_pfile  Final_GloPo_TX


#0805
#rosrun rosaria_client sonar_mapping_3dx_offline -scan_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0730_line_loop_closure_03_scan_raw_data_sonar.txt" -pose_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0730_line_loop_closure_03_pose_raw_data.txt" -sonar_2d_ogm_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/sonar_2d_ogm_offline.txt"

# offline sonar mapping run
#rosrun rosaria_client sonar_mapping_3dx_offline -scan_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/scan_raw_data_sonar.txt" -pose_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/pose_raw_data.txt" -sonar_2d_ogm_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/test_sonar_2d_ogm_offline.txt"

#rosrun rosaria_client sonar_mapping_3dx_offline -scan_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0805_line_Lobby_03_scan_raw_data_sonar.txt" -pose_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0805_line_Lobby_03_pose_raw_data.txt" -sonar_2d_ogm_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/0805_line_Lobby_03_sonar_2d_ogm_offline.txt"


#python pi_map_fusion.py -th 0.6 -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0805_line_Lobby_03 -sfile 0805_line_Lobby_03_sonar_2d_ogm  -pfile 0805_line_Lobby_03_our_lidar_data -outpath ./outputs_July/

# trajectory evaluation
#python pi_trajectory_eval.py -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -outpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0805_line_Lobby_03  -orig_pfile Orig_LocPo_TX  -opti_pfile  Opti_GloPo_TX -fina_pfile  Final_GloPo_TX

# 0806
#python pi_map_fusion.py -th 0.6 -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0806_pi_office_1m3_08_tf_inter -sfile 0806_pi_office_1m3_08_tf_inter_sonar_2d_ogm  -pfile 0806_pi_office_1m3_08_tf_inter_our_lidar_data -outpath ./outputs_July/

#python 4_pi_trajectory_eval.py -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -outpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0806_pi_office_1m3_08_tf_inter -orig_pfile Orig_LocPo_TX  -opti_pfile  Opti_GloPo_TX -fina_pfile  Final_GloPo_TX

#rosrun rosaria_client sonar_mapping_3dx_offline -scan_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0806_pi_office_1m3_08_tf_inter_scan_raw_data_sonar.txt" -pose_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0806_pi_office_1m3_08_tf_inter_pose_raw_data.txt" -sonar_2d_ogm_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/0806_pi_office_1m3_08_tf_inter_sonar_2d_ogm_offline.txt"

# 0807
#python pi_map_fusion.py -th 0.6 -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0807_SZU_L6F02_1m3_08 -sfile 0807_SZU_L6F02_1m3_08_sonar_2d_ogm  -pfile 0807_SZU_L6F02_1m3_08_our_lidar_data -outpath ./outputs_July/

#python 4_pi_trajectory_eval.py -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -outpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0807_SZU_L6F02_1m3_08 -orig_pfile Orig_LocPo_TX  -opti_pfile  Opti_GloPo_TX -fina_pfile  Final_GloPo_TX

#rosrun rosaria_client sonar_mapping_3dx_offline -scan_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0807_SZU_L6F02_1m3_08_scan_raw_data_sonar.txt" -pose_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0807_SZU_L6F02_1m3_08_pose_raw_data.txt" -sonar_2d_ogm_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/0807_SZU_L6F02_1m3_08_sonar_2d_ogm_offline.txt" -final_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0807_SZU_L6F02_1m3_08_Final_GloPo_TX.txt"


#0808
# offline sonar result
#python pi_map_fusion.py -th 0.6 -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -inpath2 /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/ -lfile 0807_SZU_L6F02_1m3_08 -sfile 0807_SZU_L6F02_1m3_08_sonar_2d_ogm_offline3  -pfile 0807_SZU_L6F02_1m3_08_our_lidar_data -outpath ./outputs_July/ -sonar_src_mode 0

# src: 0: offline newly generated sonar map  1: online saved sonar map
#python pi_map_fusion.py -th 0.6 -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -inpath2 /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/ -lfile 0807_SZU_L6F11_1m3_03 -sfile 0807_SZU_L6F11_1m3_03_sonar_2d_ogm  -pfile 0807_SZU_L6F11_1m3_03_our_lidar_data -outpath ./outputs_July/ -sonar_src_mode 1


#0809
# src: 0: offline newly generated sonar map  1: online saved sonar map
#python pi_map_fusion.py -th 0.6 -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -inpath2 /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/ -lfile 0814_SZU_F11_06_test_at_LJ -sfile 0814_SZU_F11_06_test_at_LJ_sonar_2d_ogm  -pfile 0814_SZU_F11_06_test_at_LJ_our_lidar_data -outpath ./outputs_July/ -sonar_src_mode 1

#python 4_pi_trajectory_eval.py -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -outpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0809_A4_F2_03 -orig_pfile Orig_LocPo_TX  -opti_pfile  Opti_GloPo_TX -fina_pfile  Final_GloPo_TX

#rosrun rosaria_client sonar_mapping_3dx_offline -scan_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0809_A4_F2_01_scan_raw_data_sonar.txt" -pose_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0809_A4_F2_01_pose_raw_data.txt" -sonar_2d_ogm_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/0809_A4_F2_01_sonar_2d_ogm_offline.txt" -final_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0809_A4_F2_01_Final_GloPo_TX.txt"

#0816
#map evaluation
#python 6_pi_IQ_evaluation.py -inpath /home/ubuntu/Documents/Tools/scripts/outputs_July/ -inpath2 /home/ubuntu/Documents/Tools/scripts/outputs_July/  -gtfile 0807_SZU_L6F02_1m3_08_lidar_raw -efile 0807_SZU_L6F02_1m3_08_fusion

#python 6_pi_IQ_evaluation.py -inpath /home/ubuntu/Documents/Tools/scripts/outputs_July/ -inpath2 /home/ubuntu/Documents/Tools/scripts/outputs_July/ -gtfile 0807_SZU_L6F02_1m3_08_lidar_raw -efile 0807_SZU_L6F02_1m3_08_0807_SZU_L6F02_1m3_08_sonar_2d_ogm_offline3_fusion

#python 6_pi_IQ_evaluation.py -inpath /home/ubuntu/Documents/Tools/scripts/outputs_July/ -inpath2 /home/ubuntu/Documents/Tools/scripts/outputs_July/ -gtfile 0807_SZU_L6F02_1m3_08_lidar_raw -efile 0807_SZU_L6F02_1m3_08_0807_SZU_L6F02_1m3_08_sonar_2d_ogm_fusion

#python 6_pi_IQ_evaluation.py -inpath /home/ubuntu/Documents/Tools/scripts/outputs_July/ -inpath2 /home/ubuntu/Documents/Tools/scripts/outputs_July/ -gtfile 0807_SZU_L6F02_1m3_08_lidar_raw -efile 0807_SZU_L6F02_1m3_08_lidar_raw



#0820
# src: 0: offline newly generated sonar map  1: online saved sonar map
# Input Lidar Quantification mode: 0: 3 values  1: multiple values
# python pi_map_fusion.py -th 0.6 -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -inpath2 /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/ -lfile xxx -sfile xxx_sonar_2d_ogm -pfile xxx_our_lidar_data -outpath ./outputs_July/ -sonar_src_mode 1

#python 4_pi_trajectory_eval.py -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -outpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile xxx -orig_pfile Orig_LocPo_TX  -opti_pfile  Opti_GloPo_TX -fina_pfile  Final_GloPo_TX

###### type: 
###### "LPF": local pose,
###### "PGPF": partial global pose, 
###### "FGPF": final global pose

###### "CLD"
###### "CoD"
###### "Bay"
###### "DS"
#rosrun rosaria_client sonar_mapping_3dx_offline -scan_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0807_SZU_L6F11_1m3_06_scan_raw_data_sonar.txt" -pose_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0807_SZU_L6F11_1m3_06_pose_raw_data.txt" -sonar_2d_ogm_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/0824_SZU_L6F11_06_V3_LJ_offline_sonar_2d_ogm_offline_FGPF_CLD.txt"  -orig_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0824_SZU_L6F11_06_V3_LJ_offline_Orig_LocPo_TX.txt"  -opti_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0824_SZU_L6F11_06_V3_LJ_offline_Opti_GloPo_TX.txt" -final_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0824_SZU_L6F11_06_V3_LJ_offline_Final_GloPo_TX.txt" -pose_type="FGPF"

###### src mode: 0: offline newly generated sonar map  1: online saved sonar map
###### Input Lidar Quantification mode: 0: 3 values  1: multiple values
###### our_lidar_data here is fake one. In new version, we use pose info from cartographer generator
#python pi_map_fusion.py -th 0.6 -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -inpath2 /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/ -lfile 0824_SZU_L6F11_06_V3_LJ_offline -sfile 0824_SZU_L6F11_06_V3_LJ_offline_sonar_2d_ogm_offline_FGPF_CLD -pfile 0824_SZU_L6F11_06_V3_LJ_offline_our_lidar_data -outpath ./outputs_July/ -sonar_src_mode 0

#python 4_pi_trajectory_eval.py -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -outpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0824_SZU_L6F11_06_V3_LJ_offline -orig_pfile Orig_LocPo_TX  -opti_pfile  Opti_GloPo_TX -fina_pfile  Final_GloPo_TX




#0825

###### type: 
###### "LPF": local pose,
###### "PGPF": partial global pose, 
###### "FGPF": final global pose
###### "FGPFINTP": final global pose interpolation.

###### "CLD" -- "DIRECT_CENTRAL_LINE_ALGORITHM"
###### "CoD" -- "DIRECT_CONE_ALGORITHM"
###### "Bay" -- "BAYESIAN_ALGORITHM"
###### "DS"  -- "DS_ALGORITHM"
#rosrun rosaria_client sonar_mapping_3dx_offline -scan_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0807_SZU_L6F11_1m3_01_scan_raw_data_sonar.txt" -pose_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0807_SZU_L6F11_1m3_01_pose_raw_data.txt" -sonar_2d_ogm_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/0820_SZU_L6F11_01_V5_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_Bay.txt"  -orig_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0820_SZU_L6F11_01_V5_LJ_offline_Orig_LocPo_TX.txt"  -opti_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0820_SZU_L6F11_01_V5_LJ_offline_Opti_GloPo_TX.txt" -final_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0820_SZU_L6F11_01_V5_LJ_offline_Final_GloPo_TX.txt" -pose_type="FGPFINTP" -sonar_map_algo_option="Bay" -final_node_pose_interpo_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0820_SZU_L6F11_01_V5_LJ_offline_Final_GloPo_Interpo.txt"

###### src mode: 0: offline newly generated sonar map  1: online saved sonar map
###### Input Lidar Quantification mode: 0: 3 values  1: multiple values
###### our_lidar_data here is fake one. In new version, we use pose info from cartographer generator
#python pi_map_fusion.py -th 0.6 -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -inpath2 /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/ -lfile 0820_SZU_L6F11_01_V5_LJ_offline -sfile 0820_SZU_L6F11_01_V5_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_Bay -pfile 0820_SZU_L6F11_01_V5_LJ_offline_our_lidar_data -outpath ./outputs_July/ -sonar_src_mode 0

#python 4_pi_trajectory_eval.py -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -outpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0824_SZU_L6F11_06_V3_LJ_offline -orig_pfile Orig_LocPo_TX  -opti_pfile  Opti_GloPo_TX -fina_pfile  Final_GloPo_TX


#python 4_pi_trajectory_eval.py -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -outpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile  0827_SZU_L6F11_01_V7_LJ_offline -orig_pfile Orig_LocPo_TX  -opti_pfile  Opti_GloPo_TX -fina_pfile  Final_GloPo_TX


###### type: 
###### "LPF": local pose,
###### "PGPF": partial global pose, 
###### "FGPF": final global pose,
###### "FGPFINTP": final global pose interpolation.

###### "CLD" -- "DIRECT_CENTRAL_LINE_ALGORITHM"
###### "CoD" -- "DIRECT_CONE_ALGORITHM"
###### "Bay" -- "BAYESIAN_ALGORITHM"
###### "DS"  -- "DS_ALGORITHM"
#rosrun rosaria_client sonar_mapping_3dx_offline -scan_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0807_SZU_L6F11_1m3_06_scan_raw_data_sonar.txt" -pose_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0807_SZU_L6F11_1m3_06_pose_raw_data.txt" -sonar_2d_ogm_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/0824_SZU_L6F11_06_V3_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_Bay.txt"  -orig_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0824_SZU_L6F11_06_V3_LJ_offline_Orig_LocPo_TX.txt"  -opti_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0824_SZU_L6F11_06_V3_LJ_offline_Opti_GloPo_TX.txt" -final_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0824_SZU_L6F11_06_V3_LJ_offline_Final_GloPo_TX.txt" -pose_type="FGPFINTP" -sonar_map_algo_option="Bay"

###### src mode: 0: offline newly generated sonar map  1: online saved sonar map
###### Input Lidar Quantification mode: 0: 3 values  1: multiple values
###### our_lidar_data here is fake one. In new version, we use pose info from cartographer generator
#python pi_map_fusion.py -th 0.6 -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -inpath2 /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/ -lfile 0824_SZU_L6F11_06_V3_LJ_offline -sfile 0824_SZU_L6F11_06_V3_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_CLD -pfile 0824_SZU_L6F11_06_V3_LJ_offline_our_lidar_data -outpath ./outputs_July/ -sonar_src_mode 0

#python 4_pi_trajectory_eval.py -in_map_mode 1 -inpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -outpath /home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile 0820_SZU_L6F11_01_V5_LJ_offline -orig_pfile Orig_LocPo_TX  -opti_pfile  Opti_GloPo_TX -fina_pfile  Final_GloPo_TX



#roslaunch cartographer_ros pi_robot_1_2.launch bag_filename:=/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/bagfiles/0807_SZU_L6F11_1m3_06_2019-08-07-18-50-04.bag start_sec:=0 pub_rate:=1.0



###### "CLD" -- "DIRECT_CENTRAL_LINE_ALGORITHM"
###### "CoD" -- "DIRECT_CONE_ALGORITHM"
###### "Bay" -- "BAYESIAN_ALGORITHM"
###### "DS"  -- "DS_ALGORITHM"
#rosrun rosaria_client sonar_mapping_3dx_offline -scan_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0807_SZU_L6F11_1m3_01_scan_raw_data_sonar.txt" -pose_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0807_SZU_L6F11_1m3_01_pose_raw_data.txt" -sonar_2d_ogm_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/0827_SZU_L6F11_01_V7_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_Bay.txt"  -orig_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0827_SZU_L6F11_01_V7_LJ_offline_Orig_LocPo_TX.txt"  -opti_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0827_SZU_L6F11_01_V7_LJ_offline_Opti_GloPo_TX.txt" -final_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0827_SZU_L6F11_01_V7_LJ_offline_Final_GloPo_TX.txt" -pose_type="FGPFINTP" -sonar_map_algo_option="Bay" -final_node_pose_interpo_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/0827_SZU_L6F11_01_V7_LJ_offline_Final_GloPo_Interpo.txt"



#####0819_SZU_L6F02_08_V4_LJ_offline
#TAG_NAME="0807_SZU_L6F02_1m3_08"
#VERSION="0827_SZU_L6F02_08_V4_LJ_offline"

#####0820_SZU_L6F11_01_V5_LJ_offline
#TAG_NAME="0807_SZU_L6F11_1m3_01"
#VERSION="0820_SZU_L6F11_01_V5_LJ_offline"

#TAG_NAME="0807_SZU_L6F11_1m3_06"
#VERSION="0824_SZU_L6F11_06_V3_LJ_offline"

###### type: 
###### "LPF": local pose,
###### "PGPF": partial global pose, 
###### "FGPF": final global pose,
###### "FGPFINTP": final global pose interpolation. Extra file will be gen

###### "CLD" -- "DIRECT_CENTRAL_LINE_ALGORITHM"
###### "CoD" -- "DIRECT_CONE_ALGORITHM"
###### "Bay" -- "BAYESIAN_ALGORITHM"
###### "DS"  -- "DS_ALGORITHM"
#rosrun rosaria_client sonar_mapping_3dx_offline -scan_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${TAG_NAME}_scan_raw_data_sonar.txt" -pose_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${TAG_NAME}_pose_raw_data.txt" -sonar_2d_ogm_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/${VERSION}_sonar_2d_ogm_offline_FGPFINTP_Bay_2S_1m0.txt"  -orig_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Orig_LocPo_TX.txt"  -opti_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Opti_GloPo_TX.txt" -final_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Final_GloPo_TX.txt" -pose_type="FGPFINTP" -sonar_map_algo_option="Bay" -final_node_pose_interpo_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Final_GloPo_Interpo.txt"

###### src mode: 0: offline newly generated sonar map  1: online saved sonar map
###### Input Lidar Quantification mode: 0: 3 values  1: multiple values
###### our_lidar_data here is fake one. In new version, we use pose info from cartographer generator
#python2 pi_map_fusion.py -th 0.6 -in_map_mode 1 -inpath ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -inpath2 ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/ -lfile ${VERSION} -sfile ${VERSION}_sonar_2d_ogm_offline_FGPFINTP_Bay_2S -pfile ${VERSION}_our_lidar_data -outpath ./outputs_July/ -sonar_src_mode 0

#python2 4_pi_trajectory_eval.py -in_map_mode 1 -inpath ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -outpath ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile ${VERSION} -orig_pfile Orig_LocPo_TX  -opti_pfile  Opti_GloPo_TX -fina_pfile  Final_GloPo_TX

#GROUND_TRUTH_MAP_STR file name of ground truth map, .png
#EVALUATION_MAP_STR file name of evaluation map, .png

#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "0824_SZU_L6F11_06_V3_LJ_offline_lidar_raw_gt" -efile 0824_SZU_L6F11_06_V3_LJ_offline_0824_SZU_L6F11_06_V3_LJ_offline_sonar_2d_ogm_offline_LPF_CLD_fusion 

#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "0824_SZU_L6F11_06_V3_LJ_offline_lidar_raw_gt" -efile 0824_SZU_L6F11_06_V3_LJ_offline_0824_SZU_L6F11_06_V3_LJ_offline_sonar_2d_ogm_offline_PGPF_CLD_fusion

#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "0824_SZU_L6F11_06_V3_LJ_offline_lidar_raw_gt" -efile 0824_SZU_L6F11_06_V3_LJ_offline_0824_SZU_L6F11_06_V3_LJ_offline_sonar_2d_ogm_offline_FGPF_CLD_fusion


#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "0824_SZU_L6F11_06_V3_LJ_offline_lidar_raw_gt" -efile 0824_SZU_L6F11_06_V3_LJ_offline_0824_SZU_L6F11_06_V3_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_CLD_fusion


#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/2nd_hand_scan_fusion/outputs/ -gtfile "0824_SZU_L6F11_06_V3_LJ_offline_lidar_raw_gt" -efile 0824_SZU_L6F11_06_V3_LJ_offline_lidar_raw0824_SZU_L6F11_06_V3_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_Bay_2S_sonar_filter_2nd_FUSION

#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "0824_SZU_L6F11_06_V3_LJ_offline_lidar_raw_gt" -efile 0824_SZU_L6F11_06_V3_LJ_offline_lidar_raw


# 0905 try run
#TAG_NAME="0807_SZU_L6F11_1m3_01"
#VERSION="0820_SZU_L6F11_01_V5_LJ_offline"
#TAG_NAME="0807_SZU_L6F11_1m3_06"
#VERSION="0824_SZU_L6F11_06_V3_LJ_offline"

###### type: 
###### "LPF": local pose,
###### "PGPF": partial global pose, 
###### "FGPF": final global pose,
###### "FGPFINTP": final global pose interpolation. Extra file will be gen

###### "CLD" -- "DIRECT_CENTRAL_LINE_ALGORITHM"
###### "CoD" -- "DIRECT_CONE_ALGORITHM"
###### "Bay" -- "BAYESIAN_ALGORITHM"
###### "DS"  -- "DS_ALGORITHM"
#rosrun rosaria_client sonar_mapping_3dx_offline -scan_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${TAG_NAME}_scan_raw_data_sonar.txt" -pose_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${TAG_NAME}_pose_raw_data.txt" -sonar_2d_ogm_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/${VERSION}_sonar_2d_ogm_offline_FGPFINTP_CLD.txt"  -orig_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Orig_LocPo_TX.txt"  -opti_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Opti_GloPo_TX.txt" -final_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Final_GloPo_TX.txt" -pose_type="FGPFINTP" -sonar_map_algo_option="CLD" -final_node_pose_interpo_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Final_GloPo_Interpo.txt"

# 0906 report
#TAG_NAME="0807_SZU_L6F11_1m3_01"
#VERSION="0820_SZU_L6F11_01_V5_LJ_offline"


#python2 pi_map_fusion.py -th 0.6 -in_map_mode 1 -inpath ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -inpath2 ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/ -lfile ${VERSION} -sfile ${VERSION}_sonar_2d_ogm_offline_FGPFINTP_CLD -pfile ${VERSION}_our_lidar_data -outpath ./outputs_July/ -sonar_src_mode 0

#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "0820_SZU_L6F11_01_V5_LJ_offline_lidar_raw_FUSION_gt" -efile 0820_SZU_L6F11_01_V5_LJ_offline_0820_SZU_L6F11_01_V5_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_CLD_fusion

#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "0820_SZU_L6F11_01_V5_LJ_offline_lidar_raw_FUSION_gt" -efile 0820_SZU_L6F11_01_V5_LJ_offline_sonar_2d_ogm_offline_FGPF_CLD_sonar_filter



# 0910
#TAG_NAME="0807_SZU_L6F02_1m3_08"
#VERSION="0827_SZU_L6F02_08_V4_LJ_offline"


#TAG_NAME="0809_A4_F2_08"
#VERSION="0819_A4_F2_08_V2_LJ_offline"
# traj_shape_mode: 0 : dots
# traj_shape_mode: 1 : lines
#python2 4_pi_trajectory_eval.py -in_map_mode 1 -inpath ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -outpath ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -lfile ${VERSION} -orig_pfile Orig_LocPo_TX  -opti_pfile  Opti_GloPo_TX -fina_pfile  Final_GloPo_TX -traj_shape_mode 0

###### type: 
###### "LPF": local pose,
###### "PGPF": partial global pose, 
###### "FGPF": final global pose,
###### "FGPFINTP": final global pose interpolation. Extra file will be gen

###### "CLD" -- "DIRECT_CENTRAL_LINE_ALGORITHM"
###### "CoD" -- "DIRECT_CONE_ALGORITHM"
###### "Bay" -- "BAYESIAN_ALGORITHM"
###### "DS"  -- "DS_ALGORITHM"
rosrun rosaria_client sonar_mapping_3dx_offline -scan_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${TAG_NAME}_scan_raw_data_sonar.txt" -pose_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${TAG_NAME}_pose_raw_data.txt" -sonar_2d_ogm_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/${VERSION}_sonar_2d_ogm_offline_PGPF_Bay.txt"  -orig_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Orig_LocPo_TX.txt"  -opti_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Opti_GloPo_TX.txt" -final_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Final_GloPo_TX.txt" -pose_type="PGPF" -sonar_map_algo_option="Bay" -final_node_pose_interpo_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Final_GloPo_Interpo.txt"


# src: 0: offline newly generated sonar map  1: online saved sonar map
#python2 pi_map_fusion.py -th 0.55 -in_map_mode 1 -inpath ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -inpath2 ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/ -lfile ${VERSION} -sfile ${VERSION}_sonar_2d_ogm_offline_FGPFINTP_Bay -pfile ${VERSION}_our_lidar_data -outpath ./outputs_July/ -sonar_src_mode 0


#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile 0827_SZU_L6F02_08_V4_LJ_offline_lidar_raw
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile 0827_SZU_L6F02_08_V4_LJ_offline_0827_SZU_L6F02_08_V4_LJ_offline_sonar_2d_ogm_offline_LPF_Bay_fusion
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile 0827_SZU_L6F02_08_V4_LJ_offline_0827_SZU_L6F02_08_V4_LJ_offline_sonar_2d_ogm_offline_PGPF_Bay_fusion
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile 0827_SZU_L6F02_08_V4_LJ_offline_0827_SZU_L6F02_08_V4_LJ_offline_sonar_2d_ogm_offline_FGPF_Bay_fusion
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile 0827_SZU_L6F02_08_V4_LJ_offline_0827_SZU_L6F02_08_V4_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_Bay_fusion
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/2nd_hand_scan_fusion/outputs/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile 0827_SZU_L6F02_08_V4_LJ_offline_lidar_raw0827_SZU_L6F02_08_V4_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_Bay_sonar_filter_2nd_FUSION
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/2nd_hand_scan_fusion/outputs/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile 0827_SZU_L6F02_08_V4_LJ_offline_lidar_raw0827_SZU_L6F02_08_V4_LJ_offline_sonar_2d_ogm_offline_FGPFINTP_Bay_sonar_filter_2nd_FUSION_occ

#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_lidar_raw
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_${VERSION}_sonar_2d_ogm_offline_LPF_Bay_fusion
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_${VERSION}_sonar_2d_ogm_offline_PGPF_Bay_fusion
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_${VERSION}_sonar_2d_ogm_offline_FGPF_Bay_fusion
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_${VERSION}_sonar_2d_ogm_offline_FGPFINTP_Bay_fusion
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/2nd_hand_scan_fusion/outputs/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_lidar_raw${VERSION}_sonar_2d_ogm_offline_FGPFINTP_Bay_sonar_filter_2nd_FUSION
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/2nd_hand_scan_fusion/outputs/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_lidar_raw${VERSION}_sonar_2d_ogm_offline_FGPFINTP_Bay_sonar_filter_2nd_FUSION_occ
#


#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_${VERSION}_sonar_2d_ogm_offline_FGPFINTP_CLD_fusion
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_${VERSION}_sonar_2d_ogm_offline_FGPFINTP_CoD_fusion
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_${VERSION}_sonar_2d_ogm_offline_FGPFINTP_Bay_fusion
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_${VERSION}_sonar_2d_ogm_offline_FGPFINTP_DS_fusion


#0913
#python2 pi_map_fusion.py -th 0.55 -in_map_mode 1 -inpath ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/ -inpath2 ~/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/ -lfile ${VERSION} -sfile ${VERSION}_sonar_2d_ogm_offline_FGPFINTP_Bay_2S_2m5 -pfile ${VERSION}_our_lidar_data -outpath ./outputs_July/ -sonar_src_mode 0


#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_lidar_raw
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_${VERSION}_sonar_2d_ogm_offline_LPF_Bay_fusion
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/outputs_July/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_${VERSION}_sonar_2d_ogm_offline_PGPF_Bay_fusion
#python2 6_pi_IQ_evaluation.py -inpath ~/Documents/Tools/scripts/ground_truth/ -inpath2 ~/Documents/Tools/scripts/2nd_hand_scan_fusion/outputs/ -gtfile "${VERSION}_2nd_FUSION_gt" -efile ${VERSION}_lidar_raw${VERSION}_sonar_2d_ogm_offline_FGPFINTP_Bay_sonar_filter_2nd_FUSION
#


#python2 7_pi_lidar_glass_percentage.py -inpath ~/Documents/Tools/scripts/ground_truth/  -gtfile "${VERSION}_2nd_FUSION_gt" 
#python2 7_pi_lidar_glass_percentage.py -inpath ~/Documents/Tools/scripts/outputs_July/  -gtfile "${VERSION}_lidar_raw"








