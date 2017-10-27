#include "modules/my_visrl/simsoft.h"
#include "modules/my_visrl/visrl.h"

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
#include <time.h>

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
#else
char save_location[] = "/home/default/SavedQtabs/";
#endif
// char runname[40], sessname[40], sessfold[200], runfold[200];
// char copy_location[200];
uint8_t runnum;
uint8_t endrun;
uint8_t rl_maxruns = 10;
int8_t printerror;
char *runname, *sessname, *sessfold, *runfold, *copy_location;
// char copy_location[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/__LastSaves/";
char *qd_addrs, *sv_addrs, *log_addrs, *eplog_addrs, *runinfo_addrs;
// char qdict_txt_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/qdict.txt";
// char statevisits_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/statevisits.txt";
// char log_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/log.txt";
// char epi_log_file_addrs[] = "/home/alaj/_Study/AE9999_Thesis/playground/SavedQtabs/epi_log.txt";
// FILE LOCATION WHEN RUNNING IN THE UAV
// char save_location[] = "/home/default/";
// char copy_location[] = "/home/default/__LastSaves"; // Have to create this folder in UAV
// char qdict_txt_file_addrs[] = "/home/default/qdict.txt";
// char statevisits_file_addrs[] = "/home/default/statevisits.txt";
// char log_file_addrs[] = "/home/default/log.txt";
// char epi_log_file_addrs[] = "/home/default/epi_log.txt";
// savelocation commented out for UAV implementation; may need fixing
// char savelocation[] = "/home/default/_Study/AE9999_Thesis/playground/SavedQtabs/";



FILE *qdict_txt_file;
FILE *statevisits_txt_file;
FILE *log_file;
FILE *epi_log_file;
FILE *runinfo_file;

struct tm runstart_tm;
struct tm runend_tm;
// need to control over runs; Need to control over episodes;
// Need to control run factor leves;
// Need to make folders of appropriate labels
// Need to copy and store relevant data from run in folders
// ?? fault detection and restart
// consider shell script to run simulation

void simsoft_init(void)
{
    runname = malloc( sizeof(char)*30);
    sessname = malloc( sizeof(char)*30);
    sessfold = malloc( sizeof(char)*100);
    runfold = malloc( sizeof(char)*100);
    copy_location = malloc( sizeof(char)*100);

    qd_addrs = malloc(sizeof(char)*120);
    sv_addrs  = malloc(sizeof(char)*120);
    log_addrs = malloc(sizeof(char)*120);
    eplog_addrs = malloc(sizeof(char)*120);

    runnum = 1;

    setup_sess_fold();
    setup_run_fold();

    log_file = fopen(log_addrs,"w"); //create or reset logfile
    epi_log_file = fopen(eplog_addrs,"w"); //create or reset epi logfile
    fclose(log_file);
    fclose(epi_log_file);

//     char runname[40], sessname[40], sessfold[200], runfold[200];
//     char copy_location[200];
}

uint8_t rl_addruncounter(void)
{
    ++runnum;
    return 0;
}

uint8_t rl_write_episode_log(void)
{
    // need to include error checks
    printf("===============\n");
    printf("\nWriting to file;");
    log_file = fopen(log_addrs,"a");
    fprintf(log_file,"%u %u %.1f %.1f %u %d\n",epinum,steps_taken,
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
    epi_log_file = fopen(eplog_addrs,"a");
    // Save the RL data for the step
    fprintf(epi_log_file,"%3u %4u %s %d %s % 3.0f", epinum, steps_taken,
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
    md_export_to_text(ll_qdict, qd_addrs, sv_addrs);
    return 0;
}

uint8_t load_qdict_fromtxt(void)
{
    md_import_from_text(ll_qdict, qd_addrs, sv_addrs);
    printf("Loaded %d states \n",ll_qdict->length);
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
    int length = snprintf( NULL, 0, "%s%d/", copy_location, epinum);
    char* path = malloc( length + 1 );
    snprintf( path, length + 1, "%s%d/", copy_location, epinum);
    mkdir(path,0777);

    strcpy(acopyloc,path);
    strcat(acopyloc,"statevisits.txt");
    copy_file(sv_addrs,acopyloc);

    strcpy(acopyloc,path);
    strcat(acopyloc,"qdict.txt");
    copy_file(qd_addrs,acopyloc);

    free(path);
    return FALSE;


}

uint8_t copy_logs(void)
{
    char acopyloc[200];

    strcpy(acopyloc,copy_location);
    strcat(acopyloc,"epi_log_cp.txt");
    copy_file(eplog_addrs,acopyloc);

    strcpy(acopyloc,copy_location);
    strcat(acopyloc,"log_cp.txt");
    copy_file(log_addrs,acopyloc);

    return FALSE;
}

uint8_t snprint_fail(int8_t errorcode)
{
    if (errorcode < 0) {
        printf("Failed to make path folder; sprintf failed\n");
        return 1;
    }
    else {
        return 0;
    }
}

uint8_t setup_sess_fold(void)
{
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);

    printerror = sprintf(sessname,"%d%d%d_%d%d",tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min);
    if (snprint_fail(printerror)){ return 0; }
    printerror = sprintf(sessfold,"%s%s/",save_location,sessname);
    if (snprint_fail(printerror)){ return 0; }

    mkdir(sessfold,0777);

    return 0;
}

uint8_t rl_resetrun(void)
{
    rl_curmaxeps = rl_initmaxeps;
    md_free_list(ll_qdict);
    ll_qdict = md_init_linkedlist();
    epinum=0;
    return 0;
}

uint8_t setup_run_fold(void)
{
    char *learntype = malloc(sizeof(char)*20);
    char *tasktype = malloc(sizeof(char)*20);

    time_t t = time(NULL);
    runstart_tm = *localtime(&t);

    endrun = 0;

#ifdef VISRL_SARSA
    strcpy(learntype,"SARSA");
#else
    strcpy(learntype,"QLearn");
#endif

#ifdef VISRL_2GOALS
    strcpy(tasktype,"2Goals");
#else
    strcpy(tasktype,"TurnToGoal");
#endif

    printerror = sprintf(runname,"%s_%s",learntype,tasktype);
    if (snprint_fail(printerror)){ return 0; }

    printerror = sprintf(runfold,"%s%s%02d/",sessfold,runname,runnum);
    if (snprint_fail(printerror)){ return 0; }
    printerror = sprintf(copy_location,"%s%s/",runfold,"__LastSaves");
    if (snprint_fail(printerror)){ return 0; }

    printerror = sprintf(qd_addrs,"%s%s",runfold,"qdict.txt");
    if (snprint_fail(printerror)){ return 0; }
    printerror = sprintf(sv_addrs,"%s%s",runfold,"statevisits.txt");
    if (snprint_fail(printerror)){ return 0; }
    printerror = sprintf(log_addrs,"%s%s",runfold,"log.txt");
    if (snprint_fail(printerror)){ return 0; }
    printerror = sprintf(eplog_addrs,"%s%s",runfold,"epi_log.txt");
    if (snprint_fail(printerror)){ return 0; }

    printf("\nRun:%d :: SavedAt:%s\n",runnum,runfold);
    mkdir(runfold,0777);
    mkdir(copy_location,0777);

    free(learntype);
    free(tasktype);

    return 0;
}

uint8_t save_run_metadata(void)
{
    time_t t = time(NULL);
    runend_tm = *localtime(&t);

    printerror = sprintf(runinfo_addrs,"%s%s",runfold,"qdict.txt");
    if (snprint_fail(printerror)){ return 0; }

    runinfo_file = fopen(runinfo_addrs,"w"); //create or reset logfile
    fprintf(runinfo_file,"StartTime:%d%d%d_%d%d%d\n",runstart_tm.tm_year + 1900,
            runstart_tm.tm_mon + 1, runstart_tm.tm_mday, runstart_tm.tm_hour,
            runstart_tm.tm_min, runstart_tm.tm_sec);
    fprintf(runinfo_file,"EndTime:%d%d%d_%d%d%d\n",runend_tm.tm_year + 1900,
            runend_tm.tm_mon + 1, runend_tm.tm_mday, runend_tm.tm_hour,
            runend_tm.tm_min, runend_tm.tm_sec);
    fprintf(runinfo_file,"EndTime:%d%d%d_%d%d%d\n",runend_tm.tm_year + 1900,
            runend_tm.tm_mon + 1, runend_tm.tm_mday, runend_tm.tm_hour,
            runend_tm.tm_min, runend_tm.tm_sec);



#ifdef VISRL_SARSA
    fprintf(runinfo_file,"LearnType:SARSA\n");
#else
    fprintf(runinfo_file,"LearnType:Qlearn\n");
#endif

#ifdef VISRL_2GOALS
    fprintf(runinfo_file,"TaskType:2Goals\n");
#else
    fprintf(runinfo_file,"TaskType:TurnToGoal\n");
#endif
    fprintf(runinfo_file,"Episodes:%d\n",epinum);
    fprintf(runinfo_file,"PixelCountThresh:: Red:%f Blue:%f\n",red_thresh,blue_thresh);
    fprintf(runinfo_file,"GoalReachThresh:: Red:%d, Blue:%d",min_pix_thresh);
    fprintf(runinfo_file,"MinimumPixelThresh:%d",min_pix_thresh);


//
    fclose(runinfo_file);
    return 0;
}

uint8_t simsoft_cleanup(void)
{
    free(runname);
    free(sessname);
    free(sessfold);
    free(runfold);
    free(copy_location);

    free(qd_addrs);
    free(sv_addrs);
    free(log_addrs);
    free(eplog_addrs);

    return 0;
}
