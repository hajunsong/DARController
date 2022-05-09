#include "DataControl/datacontrol.h"



DiningWaypoints::DiningWaypoints(){
    for(int i = 0; i < 6; i++){
        diningSection1[i].x.clear();
        diningSection1[i].y.clear();
        diningSection1[i].z.clear();
        diningSection1[i].q6.clear();
        diningSection1[i].q7.clear();
        diningSection1[i].time.clear();
        diningSection1[i].file_data.clear();
    }

    for(int i = 0; i < 6; i++){
        diningSection2[i].x.clear();
        diningSection2[i].y.clear();
        diningSection2[i].z.clear();
        diningSection2[i].q6.clear();
        diningSection2[i].q7.clear();
        diningSection2[i].time.clear();
        diningSection2[i].file_data.clear();
    }

    diningSection3.x.clear();
    diningSection3.y.clear();
    diningSection3.z.clear();
    diningSection3.q6.clear();
    diningSection3.q7.clear();
    diningSection3.time.clear();
    diningSection3.file_data.clear();

    diningSectionSoup.x.clear();
    diningSectionSoup.y.clear();
    diningSectionSoup.z.clear();
    diningSectionSoup.q6.clear();
    diningSectionSoup.q7.clear();
    diningSectionSoup.time.clear();
    diningSectionSoup.file_data.clear();

    for(int i = 0; i < 12; i++){
        diningSectionRice[i].x.clear();
        diningSectionRice[i].y.clear();
        diningSectionRice[i].z.clear();
        diningSectionRice[i].q6.clear();
        diningSectionRice[i].q7.clear();
        diningSectionRice[i].time.clear();
        diningSectionRice[i].file_data.clear();
    }

    double s1_offset[3] = {0.001, 0.015, -0.01};
    double s2_offset[3] = {0.0, 0.008, -0.015};
    double s3_offset[3] = {-0.01, 0.012, 0};
    double s4_offset[3] = {0.027, -0.014, -0.018};
    double s5_offset[3] = {-0.003, 0.010, -0.01};
    double alpha = 0.7;

    char file_name[255];
    for(int i = 0; i < 6; i++){
        memset(file_name, 0, sizeof(char)*255);
        sprintf(file_name, "/mnt/mtd5/daincube/KETI/path_data/tray1_summary_edit%d.csv", i+1);
        load_data(file_name, &diningSection1[i].file_data, ",");
        printf("tray 1-%d wp size : %d\n", i+1, diningSection1[i].file_data.size());
        for(unsigned int j = 0; j < diningSection1[i].file_data.size()/6; j++){
            diningSection1[i].time.push_back(diningSection1[i].file_data[j*6 + 0]*alpha);
            diningSection1[i].x.push_back(diningSection1[i].file_data[j*6 + 1]*0.001 + s1_offset[0]);
            diningSection1[i].y.push_back(diningSection1[i].file_data[j*6 + 2]*0.001 + s1_offset[1]);
            diningSection1[i].z.push_back(diningSection1[i].file_data[j*6 + 3]*0.001 + s1_offset[2]);
            diningSection1[i].q6.push_back(diningSection1[i].file_data[j*6 + 4]);
            diningSection1[i].q7.push_back(diningSection1[i].file_data[j*6 + 5]);
//                printf("%f, %f, %f, %f, %f, %f\n", diningSection1[i].time[j], diningSection1[i].x[j], diningSection1[i].y[j], diningSection1[i].z[j], diningSection1[i].q6[j], diningSection1[i].q7[j]);
        }
//            printf("\n");
    }

    for(int i = 0; i < 6; i++){
        memset(file_name, 0, sizeof(char)*255);
        sprintf(file_name, "/mnt/mtd5/daincube/KETI/path_data/tray2_summary_edit%d.csv", i+1);
        load_data(file_name, &diningSection2[i].file_data, ",");
        printf("tray 2-%d wp size : %d\n", i+1, diningSection2[i].file_data.size());
        for(unsigned int j = 0; j < diningSection2[i].file_data.size()/6; j++){
            diningSection2[i].time.push_back(diningSection2[i].file_data[j*6 + 0]*alpha);
            diningSection2[i].x.push_back(diningSection2[i].file_data[j*6 + 1]*0.001 + s2_offset[0]);
            diningSection2[i].y.push_back(diningSection2[i].file_data[j*6 + 2]*0.001 + s2_offset[1]);
            diningSection2[i].z.push_back(diningSection2[i].file_data[j*6 + 3]*0.001 + s2_offset[2]);
            diningSection2[i].q6.push_back(diningSection2[i].file_data[j*6 + 4]);
            diningSection2[i].q7.push_back(diningSection2[i].file_data[j*6 + 5]);
//            printf("%f, %f, %f, %f, %f, %f\n", diningSection2[i].time[j], diningSection2[i].x[j], diningSection2[i].y[j], diningSection2[i].z[j], diningSection2[i].q6[j], diningSection2[i].q7[j]);
        }
//        printf("\n");
    }

    load_data("/mnt/mtd5/daincube/KETI/path_data/tray3_summary_edit.csv", &diningSection3.file_data, ",");
    printf("tray 3 wp size : %d\n", diningSection3.file_data.size()/6);
    for(unsigned int i = 0; i < diningSection3.file_data.size()/6; i++){
        diningSection3.time.push_back(diningSection3.file_data[i*6 + 0]*alpha);
        diningSection3.x.push_back(diningSection3.file_data[i*6 + 1]*0.001 + s3_offset[0]);
        diningSection3.y.push_back(diningSection3.file_data[i*6 + 2]*0.001 + s3_offset[1]);
        diningSection3.z.push_back(diningSection3.file_data[i*6 + 3]*0.001 + s3_offset[2]);
        diningSection3.q6.push_back(diningSection3.file_data[i*6 + 4]);
        diningSection3.q7.push_back(diningSection3.file_data[i*6 + 5]);
//            printf("%f, %f, %f, %f, %f, %f\n", diningSection3.time[i], diningSection3.x[i], diningSection3.y[i], diningSection3.z[i], diningSection3.q6[i], diningSection3.q7[i]);
    }
//        printf("\n");

    load_data("/mnt/mtd5/daincube/KETI/path_data/tray5_soup_summary_edit.csv", &diningSectionSoup.file_data, ",");
    printf("tray 4 wp size : %d\n", diningSectionSoup.file_data.size()/6);
    for(unsigned int i = 0; i < diningSectionSoup.file_data.size()/6; i++){
        diningSectionSoup.time.push_back(diningSectionSoup.file_data[i*6 + 0]*alpha);
        diningSectionSoup.x.push_back(diningSectionSoup.file_data[i*6 + 1]*0.001 + s4_offset[0]);
        diningSectionSoup.y.push_back(diningSectionSoup.file_data[i*6 + 2]*0.001 + s4_offset[1]);
        diningSectionSoup.z.push_back(diningSectionSoup.file_data[i*6 + 3]*0.001 + s4_offset[2]);
        diningSectionSoup.q6.push_back(diningSectionSoup.file_data[i*6 + 4]);
        diningSectionSoup.q7.push_back(diningSectionSoup.file_data[i*6 + 5]);
//            printf("%f, %f, %f, %f, %f, %f\n", diningSectionSoup.time[i], diningSectionSoup.x[i], diningSectionSoup.y[i], diningSectionSoup.z[i], diningSectionSoup.q6[i], diningSectionSoup.q7[i]);
    }
//        printf("\n");

    for(int i = 0; i < 12; i++){
        memset(file_name, 0, sizeof(char)*255);
        sprintf(file_name, "/mnt/mtd5/daincube/KETI/path_data/tray4_rice_summary_edit%d.csv", i+1);
        load_data(file_name, &diningSectionRice[i].file_data, ",");
        printf("tray 5-%d wp size : %d\n", i+1, diningSectionRice[i].file_data.size());
        for(unsigned int j = 0; j < diningSectionRice[i].file_data.size()/6; j++){
            diningSectionRice[i].time.push_back(diningSectionRice[i].file_data[j*6 + 0]*alpha);
            diningSectionRice[i].x.push_back(diningSectionRice[i].file_data[j*6 + 1]*0.001 + s5_offset[0]);
            diningSectionRice[i].y.push_back(diningSectionRice[i].file_data[j*6 + 2]*0.001 + s5_offset[1]);
            diningSectionRice[i].z.push_back(diningSectionRice[i].file_data[j*6 + 3]*0.001 + s5_offset[2]);
            diningSectionRice[i].q6.push_back(diningSectionRice[i].file_data[j*6 + 4]);
            diningSectionRice[i].q7.push_back(diningSectionRice[i].file_data[j*6 + 5]);
//                printf("%f, %f, %f, %f, %f, %f\n", diningSectionRice[i].time[j], diningSectionRice[i].x[j], diningSectionRice[i].y[j], diningSectionRice[i].z[j], diningSectionRice[i].q6[j], diningSectionRice[i].q7[j]);
        }
//            printf("\n");
    }

    diningPath.clear();
    diningPath.push_back(diningSection1);
    diningPath.push_back(diningSection2);
    diningPath.push_back(&diningSection3);
    diningPath.push_back(&diningSectionSoup);
    diningPath.push_back(diningSectionRice);
}

void DiningWaypoints::reset(){
}
