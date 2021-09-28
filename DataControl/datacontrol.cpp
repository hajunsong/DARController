#include "datacontrol.h"


DataControl::DataControl()
{
    memset(&ClientToServer, 0, sizeof(ClientToServer));
    memset(&ServerToClient, 0, sizeof(ServerToClient));
    memset(&RobotData, 0, sizeof(RobotData));
    config_check = false;
    memset(&PathData, 0, sizeof(PathData));
    tablet_mode = false;
    section_indx = 0;
    trayInfor.section1 = 0;
    trayInfor.section2 = 0;
    trayInfor.section3 = 0;
    trayInfor.section4 = 0;
    trayInfor.section5 = 0;
    trayInfor.section1_cnt = 0;
    trayInfor.section2_cnt = 0;
    trayInfor.section3_cnt = 0;
    trayInfor.section4_cnt = 0;
    trayInfor.section5_cnt = 0;

    KITECHData.camera_request = false;
    KITECHData.camera_request_old = KITECHData.camera_request;
    KITECHData.tablet_check = false;
    KITECHData.tablet_check_old = KITECHData.tablet_check;
    KITECHData.interface_cmd = 0;
    KITECHData.interface_sub = 0;
    KITECHData.tablet_connect = false;

    memset(joint_offset, 0, sizeof(int32_t)*6);
    memset(tool_offset, 0, sizeof(double)*3);

    key_value = 0;
    feeding_start = false;
    joint_path_index = 0;
    logging_enable = false;
    current_state_print = false;
    dining_delay = 6000;
    dining_delay_cnt = 0;
    dining_speed = 1.0;
    dining_speed_default = 5.0;
    KITECHData.dining_speed = 1;
    KITECHData.dining_time = 6;
    select_speed = 3.0;

    PathData.teaching_pose[0] = -0.217032;
    PathData.teaching_pose[1] = -0.035785;
    PathData.teaching_pose[2] =  0.231491;
    PathData.teaching_pose[3] =  1.572425;
    PathData.teaching_pose[4] = -0.001626;
    PathData.teaching_pose[5] = -1.567730;
    PathData.teaching_beta = 0;
    PathData.teaching_d7 = 0;

    offset[0] = 0;
    offset[1] = 0.07;
    offset[2] = -0.02;

    char readBuffer[65536];
    FILE* fp = fopen("/mnt/mtd5/daincube/KETI/config.json", "r");
    FileReadStream frs(fp, readBuffer, sizeof(readBuffer));
    Document document;
    document.ParseStream(frs);

    const Value& tool_offset_json = document["TOOL_OFFSET"];
    assert(tool_offset_json.IsArray());
    for (SizeType i = 0; i < tool_offset_json.Size(); i++){
        printf("tool_offset[%d] = %f\n", i, tool_offset_json[i].GetDouble());
        tool_offset[i] = tool_offset_json[i].GetDouble();
    }
    const Value& camera_joint_json = document["CAMERA_JOINT"];
    assert(camera_joint_json.IsArray());
    for (SizeType i = 0; i < camera_joint_json.Size(); i++){
        printf("camera_joint[%d] = %f\n", i, camera_joint_json[i].GetDouble());
        operateCameraReadyJoint[i] = camera_joint_json[i].GetDouble();
    }
    const Value& module_type_json = document["MODULE_TYPE"];
    assert(module_type_json.IsNumber());
    MODULE_TYPE = static_cast<uint8_t>(module_type_json.GetUint());
    printf("module type version : %d\n", MODULE_TYPE);

    const Value& module_dir_json = document["MODULE_DIR"];
    assert(module_dir_json.IsArray());
    for (SizeType i = 0; i < module_dir_json.Size(); i++){
        printf("module dir[%d] = %d\n", i, module_dir_json[i].GetInt());
        module_dir[i] = module_dir_json[i].GetInt();
    }

    const Value& joint_offset_json = document["JOINT_OFFSET"];
    assert(joint_offset_json.IsArray());
    for (SizeType i = 0; i < joint_offset_json.Size(); i++){
        printf("joint_offset[%d] = %d\n", i, joint_offset_json[i].GetInt());
        joint_offset[i] = joint_offset_json[i].GetInt();
    }

    const Value& feeding_joint_json = document["READY_JOINT"];
    assert(feeding_joint_json.IsArray());
    for (SizeType i = 0; i < feeding_joint_json.Size(); i++){
        printf("feeding_joint[%d] = %f\n", i, feeding_joint_json[i].GetDouble());
        feedingReadyJoint[i] = feeding_joint_json[i].GetDouble();
    }

    const Value& logging_enable_json = document["LOGGING_ENABLE"];
    assert(logging_enable_json.IsBool());
    logging_enable = logging_enable_json.GetBool();
    printf("logging enable : %s\n", logging_enable ? "true" : "false");

    const Value& current_state_print_json = document["CURRENT_STATE_PRINT"];
    assert(current_state_print_json.IsBool());
    current_state_print = current_state_print_json.GetBool();
    printf("current state print : %s\n", current_state_print ? "true" : "false");
}

DataControl::~DataControl()
{
    PathData.readyPath.path_x.clear();
    PathData.readyPath.path_y.clear();
    PathData.readyPath.path_z.clear();
    for(uint i = 0; i < PathData.row; i++){
        PathData.movePath[i].path_x.clear();
        PathData.movePath[i].path_y.clear();
        PathData.movePath[i].path_z.clear();
        PathData.movePath[i].path_theta.clear();
    }
    PathData.point_px.clear();
    PathData.point_py.clear();
    PathData.point_pz.clear();
    PathData.point_rx.clear();
    PathData.point_ry.clear();
    PathData.point_rz.clear();
    PathData.point_px_home.clear();
    PathData.point_py_home.clear();
    PathData.point_pz_home.clear();
    PathData.point_rx_home.clear();
    PathData.point_ry_home.clear();
    PathData.point_rz_home.clear();
    PathData.acc_time.clear();
    PathData.total_time.clear();


    for(unsigned int i = 0; i < wp_rice.size(); i++){
        wp_rice[i].wp.clear();
    }
    for(unsigned int i = 0; i < wp_side1.size(); i++){
        wp_side1[i].wp.clear();
    }
    for(unsigned int i = 0; i < wp_side2.size(); i++){
        wp_side2[i].wp.clear();
    }
    for(unsigned int i = 0; i < wp_side3.size(); i++){
        wp_side3[i].wp.clear();
    }

    wp_rice.clear();
    wp_side1.clear();
    wp_side2.clear();
    wp_side3.clear();
    wp_soup.clear();
}

void DataControl::DataReset()
{
    memset(&ClientToServer, 0, sizeof(ClientToServer));
    memset(&ServerToClient, 0, sizeof(ServerToClient));
    memset(&RobotData, 0, sizeof(RobotData));
    memset(&PathData, 0, sizeof(PathData));

    section_indx = 0;
    trayInfor.section1 = 0;
    trayInfor.section2 = 0;
    trayInfor.section3 = 0;
    trayInfor.section4 = 0;
    trayInfor.section5 = 0;

    KITECHData.camera_request = false;
    KITECHData.camera_request_old = KITECHData.camera_request;
    KITECHData.tablet_check = false;
    KITECHData.tablet_check_old = KITECHData.tablet_check;
    KITECHData.interface_cmd = 0;
    KITECHData.interface_sub = 0;

    key_value = 0;
    feeding_start = false;
    joint_path_index = 0;
}

void DataControl::jointPositionENC2DEG(int32_t pos_enc[], double pos_deg[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_deg[i] = (pos_enc[i] - joint_offset[i])*ENC2DEG;
    }
}

void DataControl::jointPositionENC2RAD(int32_t pos_enc[], double pos_rad[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_rad[i] = (pos_enc[i] - joint_offset[i])*ENC2DEG*DEG2RAD;
    }
}

void DataControl::jointPositionENC2DEG(long pos_enc[], double pos_deg[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_deg[i] = (pos_enc[i] - joint_offset[i])*ENC2DEG;
    }
}

void DataControl::jointPositionENC2RAD(long pos_enc[], double pos_rad[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_rad[i] = (pos_enc[i] - joint_offset[i])*ENC2DEG*DEG2RAD;
    }
}

void DataControl::cartesianPoseScaleUp(double pose_small[], double pose_big[])
{
    for(int i = 0; i < NUM_DOF; i++){
        if (i < 3){
            pose_big[i] = pose_small[i]*1000;
        }
        else{
            pose_big[i] = pose_small[i]*RAD2DEG;
        }
    }
}

void DataControl::cartesianPoseScaleDown(double pose_big[], double pose_small[])
{
    for(int i = 0; i < NUM_DOF; i++){
        if (i < 3){
            pose_small[i] = pose_big[i]*0.001;
        }
        else{
            pose_small[i] = pose_big[i]*DEG2RAD;
        }
    }
}

void DataControl::jointPositionRAD2ENC(double pos_rad[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_rad[i]*RAD2DEG*DEG2ENC) + joint_offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(double pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + joint_offset[i];
    }
}

void DataControl::jointPositionRAD2ENC(double pos_rad[], long pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_rad[i]*RAD2DEG*DEG2ENC) + joint_offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(double pos_deg[], long pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + joint_offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(const double pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + joint_offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(const int32_t pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + joint_offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(int32_t pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + joint_offset[i];
    }
}

void DataControl::jointVelocityENC2RPM(int32_t vel_enc[], double vel_rpm[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        vel_rpm[i] = vel_enc[i]*ENC2RPM;
    }
}

void DataControl::jointVelocityENC2RAD(int32_t vel_enc[], double vel_rad[]){
    for(int i = 0; i < NUM_JOINT; i++){
        vel_rad[i] = vel_enc[i]*ENC2RPM*RPM2DEG*DEG2RAD;
    }
}

void DataControl::jointVelocityENC2RAD(long vel_enc[], double vel_rad[]){
    for(int i = 0; i < NUM_JOINT; i++){
        vel_rad[i] = vel_enc[i]*ENC2RPM*RPM2DEG*DEG2RAD;
    }
}

void DataControl::jointCurrentRAW2mA(int16_t cur_raw[], double cur_mA[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        cur_mA[i] = cur_raw[i]*RAW2mA;
    }
}

void DataControl::jointCurrentmA2RAW(double cur_mA[], int16_t cur_raw[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        cur_raw[i] = static_cast<int16_t>(cur_mA[i]*mA2RAW);
    }
}

void DataControl::jointPositionRAD2DEG(double pos_rad[], double pos_deg[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_deg[i] = pos_rad[i]*RAD2DEG;
    }
}

void DataControl::jointCurrentRAW2Torque(int16_t cur_raw[], double cur_torque[])
{
    if(MODULE_TYPE == 2){

    }
    else if(MODULE_TYPE == DataControl::FAR_V3){
#ifdef DYNAMIXELLIB_H
        cur_torque[0] = cur_raw[0]*RAW2mA*0.001*TORQUE_CONSTANT_W270;
        cur_torque[1] = cur_raw[1]*RAW2mA*0.001*TORQUE_CONSTANT_W270;
        cur_torque[2] = cur_raw[2]*RAW2mA*0.001*TORQUE_CONSTANT_W270;
        cur_torque[3] = cur_raw[3]*RAW2mA*0.001*TORQUE_CONSTANT_W350;
        cur_torque[4] = cur_raw[4]*RAW2mA*0.001*TORQUE_CONSTANT_W350;
        cur_torque[5] = cur_raw[5]*RAW2mA*0.001*TORQUE_CONSTANT_W350;
#endif
    }
    else if(MODULE_TYPE == DataControl::FAR_V4){

    }
}
