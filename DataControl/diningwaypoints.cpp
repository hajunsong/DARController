#include "DataControl/datacontrol.h"

DiningWaypoints::DiningWaypoints(){
    for(int i = 0; i < 5; i++){
        diningSection[i].x.clear();
        diningSection[i].y.clear();
        diningSection[i].z.clear();
        diningSection[i].beta.clear();
        diningSection[i].d7.clear();
        diningSection[i].option.clear();
    }

    spoon = -25;
    chop = 25 + 3;

    y_offset[0] = 0.003;
    y_offset[1] = 0.007;
    z_offset_ref = 0.168 - 0.019;
    z_offset[0] = z_offset_ref - spoon*0.001;
    z_offset[1] = z_offset_ref + chop*0.001;

    ref_s1[0] = -0.200594; ref_s1[1] = 0.189727; ref_s1[2] = -0.039;
    ref_s2[0] = -0.299594; ref_s2[1] = 0.189727; ref_s2[2] = -0.039;
    ref_s3[0] = -0.101594; ref_s3[1] = 0.189727; ref_s3[2] = -0.039;

    ref_s4[0] = -0.122694; ref_s4[1] = 0.115945; ref_s4[2] = -0.039;

    ref_s5[0] = -0.325316; ref_s5[1] = 0.062945; ref_s5[2] = -0.039;

//    ref_s5_x[0] = -0.325316; ref_s5_x[1] = -0.289094; ref_s5_x[2] = -0.253094; ref_s5_x[3] = -0.216871;
//    ref_s5_y[0] =  0.062945; ref_s5_y[1] =  0.089945; ref_s5_y[2] =  0.129945; ref_s5_z = -0.039;

    side1_offset[0] = -0.019;   side1_offset[1] =  0.018;
    side1_offset[2] =  0.019;   side1_offset[3] =  0.018;
    side1_offset[4] = -0.019;   side1_offset[5] = -0.014;
    side1_offset[6] =  0.019;   side1_offset[7] = -0.014;

    side2_offset[0] = -0.019;   side2_offset[1] = 0.018;
    side2_offset[2] =  0.019;   side2_offset[3] = 0.018;

    side3_offset[0] = -0.019;   side3_offset[1] =  0.018;
    side3_offset[2] =  0.019;   side3_offset[3] =  0.018;
    side3_offset[4] = -0.019;   side3_offset[5] = -0.014;
    side3_offset[6] =  0.019;   side3_offset[7] = -0.014;

    side5_offset_x[0] = 0;
    side5_offset_x[1] = 0.036*1;
    side5_offset_x[2] = 0.036*2;
    side5_offset_x[3] = 0.036*3;

    side5_offset_y[0] = 0;
    side5_offset_y[1] = 0.027*1;
    side5_offset_y[2] = 0.027*2;

    diningSection[0].x.push_back(ref_s2[0]); diningSection[0].y.push_back(ref_s2[1] + y_offset[1]); diningSection[0].z.push_back(ref_s2[2] + z_offset[1] + 0.05); diningSection[0].beta.push_back(0);   diningSection[0].d7.push_back(0);    diningSection[0].option.push_back(0);
    diningSection[0].x.push_back(ref_s2[0]); diningSection[0].y.push_back(ref_s2[1] + y_offset[1]); diningSection[0].z.push_back(ref_s2[2] + z_offset[1] + 0.05); diningSection[0].beta.push_back(-90); diningSection[0].d7.push_back(0);    diningSection[0].option.push_back(0);
    diningSection[0].x.push_back(ref_s2[0]); diningSection[0].y.push_back(ref_s2[1] + y_offset[1]); diningSection[0].z.push_back(ref_s2[2] + z_offset[1] + 0.023); diningSection[0].beta.push_back(-90); diningSection[0].d7.push_back(0);    diningSection[0].option.push_back(0);
    diningSection[0].x.push_back(ref_s2[0]); diningSection[0].y.push_back(ref_s2[1] + y_offset[1]); diningSection[0].z.push_back(ref_s2[2] + z_offset[1] + 0.023); diningSection[0].beta.push_back(-90); diningSection[0].d7.push_back(chop); diningSection[0].option.push_back(0);
    diningSection[0].x.push_back(ref_s2[0]); diningSection[0].y.push_back(ref_s2[1] + y_offset[1]); diningSection[0].z.push_back(ref_s2[2] + z_offset[1] + 0.05); diningSection[0].beta.push_back(-90); diningSection[0].d7.push_back(chop); diningSection[0].option.push_back(0);
    diningSection[0].x.push_back(ref_s2[0]); diningSection[0].y.push_back(ref_s2[1] + y_offset[1]); diningSection[0].z.push_back(ref_s2[2] + z_offset[1] + 0.05); diningSection[0].beta.push_back(0); diningSection[0].d7.push_back(chop+2); diningSection[0].option.push_back(0);

    diningSection[1].x.push_back(ref_s1[0]); diningSection[1].y.push_back(ref_s1[1] - y_offset[0]);                                      diningSection[1].z.push_back(ref_s1[2] + z_offset[0] + 0.05);                  diningSection[1].beta.push_back(0);    diningSection[1].d7.push_back(0);     diningSection[1].option.push_back(0);
    diningSection[1].x.push_back(ref_s1[0]); diningSection[1].y.push_back(ref_s1[1] - y_offset[0] + z_offset[0]*sin(30*DEG2RAD));        diningSection[1].z.push_back(ref_s1[2] + z_offset[0]*cos(30*DEG2RAD) + 0.03);  diningSection[1].beta.push_back(-60);  diningSection[1].d7.push_back(spoon); diningSection[1].option.push_back(1);
    diningSection[1].x.push_back(ref_s1[0]); diningSection[1].y.push_back(ref_s1[1] - y_offset[0] + z_offset[0]*sin(40*DEG2RAD) - 0.028);diningSection[1].z.push_back(ref_s1[2] + z_offset[0]*cos(40*DEG2RAD) - 0.01);  diningSection[1].beta.push_back(-50);  diningSection[1].d7.push_back(spoon); diningSection[1].option.push_back(0);
    diningSection[1].x.push_back(ref_s1[0]); diningSection[1].y.push_back(ref_s1[1] - y_offset[0] + z_offset[0]*sin(40*DEG2RAD) + 0.02); diningSection[1].z.push_back(ref_s1[2] + z_offset[0]*cos(40*DEG2RAD) - 0.02);  diningSection[1].beta.push_back(-15);  diningSection[1].d7.push_back(spoon); diningSection[1].option.push_back(0);

//    for(int i = 0; i < NUM_JOINT; i++){
//        diningSection[2].joint_path_h[i].clear();
//        diningSection[2].joint_path_d[i].clear();
//        diningSection[2].joint_path_l[i].clear();
//    }
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q1_h.txt", &diningSection[2].joint_path_h[0], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q2_h.txt", &diningSection[2].joint_path_h[1], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q3_h.txt", &diningSection[2].joint_path_h[2], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q4_h.txt", &diningSection[2].joint_path_h[3], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q5_h.txt", &diningSection[2].joint_path_h[4], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q6_h.txt", &diningSection[2].joint_path_h[5], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q7_h.txt", &diningSection[2].joint_path_h[6], "\n");

//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q1_d.txt", &diningSection[2].joint_path_d[0], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q2_d.txt", &diningSection[2].joint_path_d[1], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q3_d.txt", &diningSection[2].joint_path_d[2], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q4_d.txt", &diningSection[2].joint_path_d[3], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q5_d.txt", &diningSection[2].joint_path_d[4], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q6_d.txt", &diningSection[2].joint_path_d[5], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q7_d.txt", &diningSection[2].joint_path_d[6], "\n");

//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q1_l.txt", &diningSection[2].joint_path_l[0], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q2_l.txt", &diningSection[2].joint_path_l[1], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q3_l.txt", &diningSection[2].joint_path_l[2], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q4_l.txt", &diningSection[2].joint_path_l[3], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q5_l.txt", &diningSection[2].joint_path_l[4], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q6_l.txt", &diningSection[2].joint_path_l[5], "\n");
//    load_data("/mnt/mtd5/daincube/KETI/joint_data/q7_l.txt", &diningSection[2].joint_path_l[6], "\n");

//    for(int i = 0; i < NUM_JOINT; i++){
//        printf("q%d_h size : %d\n", i, diningSection[2].joint_path_h->size());
//        printf("q%d_d size : %d\n", i, diningSection[2].joint_path_d->size());
//        printf("q%d_l size : %d\n", i, diningSection[2].joint_path_l->size());
//    }

    diningSection[2].x.push_back(ref_s3[0]); diningSection[2].y.push_back(ref_s3[1] + y_offset[1]); diningSection[2].z.push_back(ref_s3[2] + z_offset[1] + 0.05);  diningSection[2].beta.push_back(0);   diningSection[2].d7.push_back(0);    diningSection[2].option.push_back(0);
    diningSection[2].x.push_back(ref_s3[0]); diningSection[2].y.push_back(ref_s3[1] + y_offset[1]); diningSection[2].z.push_back(ref_s3[2] + z_offset[1] + 0.05);  diningSection[2].beta.push_back(-90); diningSection[2].d7.push_back(0);    diningSection[2].option.push_back(0);
    diningSection[2].x.push_back(ref_s3[0]); diningSection[2].y.push_back(ref_s3[1] + y_offset[1]); diningSection[2].z.push_back(ref_s3[2] + z_offset[1] + 0.023); diningSection[2].beta.push_back(-90); diningSection[2].d7.push_back(0);    diningSection[2].option.push_back(0);
    diningSection[2].x.push_back(ref_s3[0]); diningSection[2].y.push_back(ref_s3[1] + y_offset[1]); diningSection[2].z.push_back(ref_s3[2] + z_offset[1] + 0.023); diningSection[2].beta.push_back(-90); diningSection[2].d7.push_back(chop); diningSection[2].option.push_back(0);
    diningSection[2].x.push_back(ref_s3[0]); diningSection[2].y.push_back(ref_s3[1] + y_offset[1]); diningSection[2].z.push_back(ref_s3[2] + z_offset[1] + 0.05);  diningSection[2].beta.push_back(-90); diningSection[2].d7.push_back(chop); diningSection[2].option.push_back(0);

    diningSection[3].x.push_back(ref_s4[0]); diningSection[3].y.push_back(ref_s4[1] - y_offset[0]);                                      diningSection[3].z.push_back(ref_s4[2] + z_offset[0] + 0.05);                  diningSection[3].beta.push_back(0);    diningSection[3].d7.push_back(0);     diningSection[3].option.push_back(0);
    diningSection[3].x.push_back(ref_s4[0]); diningSection[3].y.push_back(ref_s4[1] - y_offset[0] + z_offset[0]*sin(30*DEG2RAD));        diningSection[3].z.push_back(ref_s4[2] + z_offset[0]*cos(30*DEG2RAD) + 0.020);  diningSection[3].beta.push_back(-60);  diningSection[3].d7.push_back(spoon); diningSection[3].option.push_back(1);
    diningSection[3].x.push_back(ref_s4[0]); diningSection[3].y.push_back(ref_s4[1] - y_offset[0] + z_offset[0]*sin(30*DEG2RAD) - 0.02); diningSection[3].z.push_back(ref_s4[2] + z_offset[0]*cos(30*DEG2RAD) - 0.04);  diningSection[3].beta.push_back(-40);  diningSection[3].d7.push_back(spoon); diningSection[3].option.push_back(0);
    diningSection[3].x.push_back(ref_s4[0]); diningSection[3].y.push_back(ref_s4[1] - y_offset[0] + z_offset[0]*sin(30*DEG2RAD) + 0.04); diningSection[3].z.push_back(ref_s4[2] + z_offset[0]*cos(30*DEG2RAD) - 0.045);  diningSection[3].beta.push_back(-15);  diningSection[3].d7.push_back(spoon); diningSection[3].option.push_back(0);

    diningSection[4].x.push_back(ref_s5[0]); diningSection[4].y.push_back(ref_s5[1] - y_offset[0]);                                      diningSection[4].z.push_back(ref_s5[2] + z_offset[0] + 0.1);                   diningSection[4].beta.push_back(0);    diningSection[4].d7.push_back(0);     diningSection[4].option.push_back(0);
//    diningSection[4].x.push_back(ref_s5[0]); diningSection[4].y.push_back(ref_s5[0] - y_offset[0] - z_offset[0]*sin(30*DEG2RAD));        diningSection[4].z.push_back(ref_s5[2] + z_offset[0] + 0.1);                   diningSection[4].beta.push_back(-120); diningSection[4].d7.push_back(spoon); diningSection[4].option.push_back(0);
//    diningSection[4].x.push_back(ref_s5[0]); diningSection[4].y.push_back(ref_s5[0] - y_offset[0] - z_offset[0]*sin(30*DEG2RAD));        diningSection[4].z.push_back(ref_s5[2] + z_offset[0]*cos(30*DEG2RAD));         diningSection[4].beta.push_back(-120); diningSection[4].d7.push_back(spoon); diningSection[4].option.push_back(0);
//    diningSection[4].x.push_back(ref_s5[0]); diningSection[4].y.push_back(ref_s5[0] - y_offset[0] - z_offset[0]*sin(30*DEG2RAD));        diningSection[4].z.push_back(ref_s5[2] + z_offset[0]*cos(30*DEG2RAD));         diningSection[4].beta.push_back(-120); diningSection[4].d7.push_back(spoon); diningSection[4].option.push_back(0);
    diningSection[4].x.push_back(ref_s5[0]); diningSection[4].y.push_back(ref_s5[1] - y_offset[0] + z_offset[0]*sin(15*DEG2RAD) + 0.01); diningSection[4].z.push_back(ref_s5[2] + z_offset[0]*cos(15*DEG2RAD) + 0.04);  diningSection[4].beta.push_back(-75);  diningSection[4].d7.push_back(spoon); diningSection[4].option.push_back(0);
    diningSection[4].x.push_back(ref_s5[0]); diningSection[4].y.push_back(ref_s5[1] - y_offset[0] + z_offset[0]*sin(15*DEG2RAD) - 0.015); diningSection[4].z.push_back(ref_s5[2] + z_offset[0]*cos(15*DEG2RAD) - 0.00);  diningSection[4].beta.push_back(-75);  diningSection[4].d7.push_back(spoon); diningSection[4].option.push_back(0);
    diningSection[4].x.push_back(ref_s5[0]); diningSection[4].y.push_back(ref_s5[1] - y_offset[0] + z_offset[0]*sin(45*DEG2RAD) + 0.04); diningSection[4].z.push_back(ref_s5[2] + z_offset[0]*cos(45*DEG2RAD) - 0.01);  diningSection[4].beta.push_back(-15);  diningSection[4].d7.push_back(spoon); diningSection[4].option.push_back(0);
}

void DiningWaypoints::reset(){
}
