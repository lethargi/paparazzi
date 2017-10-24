#include "modules/my_visrl/simsoft.h"

#ifdef VISRL_NPS
#include "modules/my_visrl/vis_nps.h"
#endif
// #else
#ifdef VISRL_AP
#include "modules/my_visrl/vis_ap.h"
#endif
//Copied all includes from visrl.c
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// #include <limits.h>
#include <float.h>

// think this contains state information from that file
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "autopilot.h"

#define NAV_C
#include "generated/flight_plan.h"

#include "subsystems/datalink/telemetry.h"


// FILE LOCATIONS DURING NPS RUNS
#ifdef VISRL_NPS
char save_location[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/";
char copy_location[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/__LastSaves/";
char qdict_txt_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/qdict.txt";
char qdictkeys_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/qdict_keys.dat";
char qdictvalues_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/qdict_values.dat";
char statevisitsvalues_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/statevisits_values.dat";
char statevisits_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/statevisits.txt";
char log_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/log.txt";
char epi_log_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/epi_log.txt";
#else
// FILE LOCATION WHEN RUNNING IN THE UAV
// char save_location[] = "/home/default/";
char copy_location[] = "/home/default/__LastSaves"; // Have to create this folder in UAV
char qdict_txt_file_addrs[] = "/home/default/qdict.txt";
char qdictkeys_file_addrs[] = "/home/default/qdict_keys.dat";
char qdictvalues_file_addrs[] = "/home/default/qdict_values.dat";
char statevisitsvalues_file_addrs[] = "/home/defaultstatevisits_values.dat";
char statevisits_file_addrs[] = "/home/default/statevisits.txt";
char log_file_addrs[] = "/home/default/log.txt";
char epi_log_file_addrs[] = "/home/default/epi_log.txt";
// savelocation commented out for UAV implementation; may need fixing
// char savelocation[] = "/home/default/_Study/AE9999_Thesis/playground/SavedQtabs/";
#endif


FILE *qdict_txt_file;
FILE *qdictkeys_file;
FILE *qdictvalues_file;
FILE *statevisitsvalues_file;
FILE *statevisits_txt_file;
FILE *log_file;
FILE *epi_log_file;


// need to control over runs; Need to control over episodes; 
// Need to control run factor leves;
// Need to make folders of appropriate labels
// Need to copy and store relevant data from run in folders
// ?? fault detection and restart
// consider shell script to run simulation 

void simsoft_init(void)
{
    log_file = fopen(log_file_addrs,"w"); //create or reset logfile
    epi_log_file = fopen(epi_log_file_addrs,"w"); //create or reset epi logfile
    fclose(log_file);
    fclose(epi_log_file);
}

uint8_t rl_write_episode_log(void)
{
    // need to include error checks
    printf("===============\n");
    printf("\nWriting to file;");
    log_file = fopen(log_file_addrs,"a");
    fprintf(log_file,"%u %u %.1f %.1f %u %d\n",episodes_simulated,steps_taken,
            sum_dQ,episode_rewards,ll_qdict->length,rl_eps);
    // fwrite(qtab,sizeof(float),sizeof(qtab)/sizeof(float),qtab_file);
    fclose(log_file);
    printf("Done\n");
    printf("===============\n");
    return 0;
}

uint8_t rl_write_step_log(void)
{
    // need to include error checks
    epi_log_file = fopen(epi_log_file_addrs,"a");
    // Save the RL data for the step
    fprintf(epi_log_file,"%3u %4u %s %d %s % 3.0f", episodes_simulated, steps_taken,
            cur_sta, cur_act, act_type, cur_rew);
    // Save absolute state data
    fprintf(epi_log_file," % 05.2f % 04.3f % 04.3f", DegOfRad(GetCurHeading()), GetPosX(),
            GetPosY());
    // save comamand data
    fprintf(epi_log_file," % 05.2f % 04.3f % 04.3f",
            DegOfRad(ANGLE_FLOAT_OF_BFP(nav_heading)), WaypointX(WP_GOAL),
            WaypointY(WP_GOAL));
    fprintf(epi_log_file,"\n");
    fclose(epi_log_file);
    return 0;
}

// functions to read and write Q-tables; can be improved overall
uint8_t print_qdict(void)
{
    // printf("\n====== Writing to txt file =========\n");
    md_export_to_text(ll_qdict, qdict_txt_file_addrs, statevisits_file_addrs);
    return 0;
}

uint8_t write_qdict(void)
{
    // printf("\n====== Writing to dat file =========\n");
    md_export_to_dat(ll_qdict, qdictkeys_file_addrs, qdictvalues_file_addrs,
            statevisitsvalues_file_addrs);
    // printf("======= Done ==========\n");
    return 0;
}

uint8_t load_qdict(void)
{
    //safety feature about testing if "myqdict" already exists is unimplemented
    // printf("\n ===Loading QDICT from dat=== \n");
    md_import_from_dat(ll_qdict, qdictkeys_file_addrs, qdictvalues_file_addrs,
            statevisitsvalues_file_addrs);
    // printf("\n ===Done=== \n");
    return 0;
}

uint8_t load_qdict_fromtxt(void)
{
    // printf("\n ===Loading QDICT from txt=== \n");
    md_import_from_text(ll_qdict, qdict_txt_file_addrs,
            statevisits_file_addrs);
    // printf("\n ===Done=== \n");
    return 0;
}

uint8_t copy_file(char *old_filename, char  *new_filename)
{
    FILE  *ptr_old, *ptr_new;
    int  a;

    ptr_old = fopen(old_filename,"rb");
    ptr_new = fopen(new_filename,"wb");

    while(1)
    {
        a  =  fgetc(ptr_old);

        if(!feof(ptr_old))
            fputc(a, ptr_new);
        else
            break;
    }

    fclose(ptr_new);
    fclose(ptr_old);
    return  0;
}

uint8_t copy_qdict(void)
{
    char acopyloc[200];
    //https://stackoverflow.com/questions/8257714/how-to-convert-an-int-to-string-in-c
    int length = snprintf( NULL, 0, "%s%d/", copy_location, episodes_simulated);
    char* path = malloc( length + 1 );
    snprintf( path, length + 1, "%s%d/", copy_location, episodes_simulated);
    mkdir(path,0777);

    strcpy(acopyloc,path);
    strcat(acopyloc,"statevisits.txt");
    copy_file(statevisits_file_addrs,acopyloc);

    strcpy(acopyloc,path);
    strcat(acopyloc,"qdict.txt");
    copy_file(qdict_txt_file_addrs,acopyloc);

    strcpy(acopyloc,path);
    strcat(acopyloc,"qdict_keys.dat");
    copy_file(qdictkeys_file_addrs,acopyloc);

    strcpy(acopyloc,path);
    strcat(acopyloc,"qdict_values.dat");
    copy_file(qdictvalues_file_addrs,acopyloc);

    strcpy(acopyloc,path);
    strcat(acopyloc,"statevisits.dat");
    copy_file(statevisitsvalues_file_addrs,acopyloc);

    free(path);
    return FALSE;


}

uint8_t copy_logs(void)
{
    char acopyloc[200];

    strcpy(acopyloc,copy_location);
    strcat(acopyloc,"epi_log_cp.txt");
    copy_file(epi_log_file_addrs,acopyloc);

    strcpy(acopyloc,copy_location);
    strcat(acopyloc,"log_cp.txt");
    copy_file(log_file_addrs,acopyloc);

    return FALSE;
}


