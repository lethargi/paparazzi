/*
#include <time.h>
#include <math.h>

char qdict_txt_file_addrs[] = "qdict.txt";
char qdictkeys_file_addrs[] = "qdict_keys.dat";
char qdictvalues_file_addrs[] = "qdict_values.dat";
char statevisitsvalues_file_addrs[] = "statevisits_values.dat";
char statevisits_file_addrs[] = "statevisits.txt";
*/
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <float.h>
#include <unistd.h>
#include <string.h>

#include "modules/manan_test/mydict.h"


md_linkedlist *md_init_linkedlist(void)
{
    md_linkedlist *mylist = (md_linkedlist*)malloc(sizeof(md_linkedlist));
    mylist->elemsize = sizeof(md_node);
    mylist->length = 0;
    // char *mykey = "First";
    mylist->head = NULL;
    return mylist;
}

void md_node_best_action(md_node *cursor)
{
    float cur_best_val = -FLT_MAX;
    // uint8_t cur_best_act;
    if (cursor->f > cur_best_val) {
        cur_best_val = cursor->f;
        cursor->best_act = 0;
    }
    if (cursor->l > cur_best_val) {
        cur_best_val = cursor->l;
        cursor->best_act = 1;
    }
    if (cursor->r > cur_best_val) {
        cur_best_val = cursor->r;
        cursor->best_act = 2;
    }
    // use this when using the turn extended action
    /*
    if (cursor->T > cur_best_val) {
        cur_best_val = cursor->T;
        cursor->best_act = 3;
    }
    */
}

md_node* md_create(char *key, md_node *next)
{
    md_node *new_node = (md_node*)malloc(sizeof(md_node));
    if(new_node == NULL)
    {
        printf("Error creating node\n");
        return 0;
        // exit(0);
    }
    new_node->key = key;

    // value functions initialized at 0
    new_node->f = 0;
    new_node->l = 0;
    new_node->r = 0;
    new_node->T = 0;

    // state visits intialization
    new_node->f_v = 0;
    new_node->l_v = 0;
    new_node->r_v = 0;
    new_node->T_v = 0;

    new_node->next = next;

    return new_node;
}

md_node* md_prepend_list(md_linkedlist* mylist, char *key)
{
    md_node* new_node = md_create(key, mylist->head);
    mylist->head = new_node;
    mylist->length++;
    // head = new_node;
    return new_node;
}

md_node* search(md_linkedlist* alist, char *key)
{
    md_node *cursor = alist->head;
    while(cursor != NULL)
    {
        if(strcmp(cursor->key,key) == 0) {
            return cursor;
        }
        cursor = cursor->next;
    }
    return NULL;
}

uint8_t md_free_list(md_linkedlist* alist)
{
    md_node *head, *cursor, *tmp;
    head = alist->head;
    if (head != NULL)
    {
        cursor = head->next;
        head->next = NULL;
        while (cursor != NULL)
        {
            tmp = cursor->next;
            free(cursor);
            cursor = tmp;
        }
    }
    free(alist);
    return 0;
}

md_linkedlist *md_import_from_text(char* qdict_filename, char* statev_filename)
{
    // create a new linked list object
    md_linkedlist *mylist = (md_linkedlist*)malloc(sizeof(md_linkedlist));
    mylist->elemsize = sizeof(md_node);
    mylist->length = 0;
    mylist->head = NULL;

    size_t dummy;
    md_node *cursor;

    FILE *qdict_txt_file = fopen(qdict_filename,"r");
    FILE *statevisits_txt_file = fopen(statev_filename,"r");

    //remove first line
    dummy = fscanf(qdict_txt_file, "%*[^\n]");
    dummy = fscanf(statevisits_txt_file, "%*[^\n]");

    char akey[30];
    float f, l, r;//, T;
    int f_v, l_v, r_v;// T_v;
    while (!feof(qdict_txt_file)) {

        // read values from file and store in cache variable
        dummy = fscanf(qdict_txt_file, "%s %f %f %f", akey, &f, &l, &r);
        dummy = fscanf(statevisits_txt_file, "%s %d %d %d", akey, &f_v, &l_v,
                &r_v);

        // create new list element with key
        cursor = md_prepend_list(mylist,strdup(akey));

        // fill in other values
        cursor->f = f;
        cursor->l = l;
        cursor->r = r;
        // cursor->T = T;

        cursor->f_v = f_v;
        cursor->l_v = l_v;
        cursor->r_v = r_v;
        // cursor->T_v = T_v;

//         printf("%s %6.2f %6.2f %6.2f %4d %4d %4d \n",cursor->key, cursor->f, cursor->l,
//                 cursor->r, cursor->f_v, cursor->l_v, cursor->r_v);
    }
    fclose(qdict_txt_file);
    fclose(statevisits_txt_file);
    return mylist;
}

md_linkedlist *md_import_from_dat(char* qdictkeys_filename,
        char* qdictvalues_filename, char* statevvalues_filename)
{
    md_linkedlist *mylist = (md_linkedlist*)malloc(sizeof(md_linkedlist));
    mylist->elemsize = sizeof(md_node);
    mylist->length = 0;
    mylist->head = NULL;

    size_t dummy;
    md_node *cursor;

    //safety feature about testing if "myqdict" already exists is unimplemented
    printf("\n ===LoadingQDICTFromDat=== \n");

    FILE *qdictkeys_file = fopen(qdictkeys_filename,"rb");
    FILE *qdictvalues_file = fopen(qdictvalues_filename,"rb");
    FILE *statevisitsvalues_file = fopen(statevvalues_filename,"rb");
//     fwrite(valvisitsarr,sizeof(uint16_t),sizeof(valvisitsarr)/sizeof(uint16_t),statevisitsvalues_file);

    char akey[30];
    float f, l, r;//, T;
    int f_v, l_v, r_v;//, T_v;

    if ((qdictkeys_file != NULL) && (qdictvalues_file != NULL) && (statevisitsvalues_file != NULL)) {
        while (!feof(qdictkeys_file)) {
            // Read from the dat file
            dummy = fread(&akey,sizeof(char[30]),1,qdictkeys_file);

            dummy = fread(&f,sizeof(float),1,qdictvalues_file);
            dummy = fread(&l,sizeof(float),1,qdictvalues_file);
            dummy = fread(&r,sizeof(float),1,qdictvalues_file);
            // dummy = fread(&T,sizeof(float),1,qdictvalues_file);
            dummy = fread(&f_v,sizeof(int),1,statevisitsvalues_file);
            dummy = fread(&l_v,sizeof(int),1,statevisitsvalues_file);
            dummy = fread(&r_v,sizeof(int),1,statevisitsvalues_file);
            // dummy = fread(&T_v,sizeof(int),1,statevisitsvalues_file);

            // create new list element with key
            cursor = md_prepend_list(mylist,strdup(akey));

            // fill in other values
            cursor->f = f;
            cursor->l = l;
            cursor->r = r;
            // cursor->T = T;

            cursor->f_v = f_v;
            cursor->l_v = l_v;
            cursor->r_v = r_v;
            // cursor->T_v = T_v;

//             printf("%s %6.2f %6.2f %6.2f %4d %4d %4d \n",cursor->key, cursor->f, cursor->l,
//                     cursor->r, cursor->f_v, cursor->l_v, cursor->r_v);
        }
        printf("Done\n");
        printf("===============\n");
    }
    else {
        printf("\n NO FILE TO READ \n");
    }
    fclose(qdictkeys_file);
    fclose(qdictvalues_file);
    fclose(statevisitsvalues_file);

    return mylist;
}

uint8_t md_export_to_text(md_linkedlist* mylist, char* qdict_filename, char* statev_filename)
{
    FILE *qdict_txt_file = fopen(qdict_filename,"w");
    FILE *statevisits_txt_file = fopen(statev_filename,"w");

    md_node *cursor = mylist->head;

    fprintf(qdict_txt_file,"State \t\t\t | For \t\t | Lef \t\t | Rig \n");
    fprintf(statevisits_txt_file,"State \t\t\t | For \t\t | Lef \t\t | Rig \n");

    char akey[30];
    float f, l, r;//, T;
    int f_v, l_v, r_v;//, T_v;
    while(cursor != NULL)
    {
        strcpy(akey,cursor->key);

        f = cursor->f;
        l = cursor->l;
        r = cursor->r;
        // T = cursor->T;

        f_v = cursor->f_v;
        l_v = cursor->l_v;
        r_v = cursor->r_v;
        // T_v = cursor->T_v;

        fprintf(qdict_txt_file, "%s %03.3f %03.3f %03.3f \n", (char *)akey, f, l, r);
        fprintf(statevisits_txt_file, "%s %4u %4u %4u \n", (char *)akey, f_v, l_v, r_v);
        cursor = cursor->next;
    }

    fclose(qdict_txt_file);
    fclose(statevisits_txt_file);

    return 0;
}

uint8_t md_export_to_dat(md_linkedlist* mylist, char* qdictkeys_filename,
        char* qdictvalues_filename, char* statevvalues_filename)
{
    FILE *qdictkeys_file = fopen(qdictkeys_filename,"wb");
    FILE *qdictvalues_file = fopen(qdictvalues_filename,"wb");
    FILE *statevisitsvalues_file = fopen(statevvalues_filename,"wb");

    // uint16_t dictsize = mylist->length;
    // uint16_t rowi = 0;

//     float valarr[dictsize][3];
//     uint16_t valvisitsarr[dictsize][3];
//     char keyarr[dictsize][30];

    md_node *cursor = mylist->head;
    // printf("ExportToDat1\n");

    char akey[30];
    float f, l, r;//, T;
    int f_v, l_v, r_v;//, T_v;

    while(cursor != NULL)
    {
        // printf("%f %f %f %d %d %d\n",cursor->f,cursor->l,cursor->r,cursor->f_v,cursor->l_v,cursor->r_v);
        // printf("ExportToDat3\n");
        /*
        strcpy(keyarr[rowi],cursor->key);
        valarr[rowi][0] = cursor->f;
        valarr[rowi][1] = cursor->l;
        valarr[rowi][2] = cursor->r;
        // printf("ExportToDat4\n");

        valvisitsarr[rowi][0] = cursor->f_v;
        valvisitsarr[rowi][1] = cursor->l_v;
        valvisitsarr[rowi][2] = cursor->r_v;
        // printf("ExportToDat5\n");

        // printf("\n");
        rowi++;

        */
        fwrite(akey,sizeof(char),sizeof(akey)/sizeof(char),qdictkeys_file);

        fwrite(&f,sizeof(float),1,qdictvalues_file);
        fwrite(&l,sizeof(float),1,qdictvalues_file);
        fwrite(&r,sizeof(float),1,qdictvalues_file);
        // fwrite(T,sizeof(float),1,qdictvalues_file);

        fwrite(&f_v,sizeof(uint16_t),1,statevisitsvalues_file);
        fwrite(&l_v,sizeof(uint16_t),1,statevisitsvalues_file);
        fwrite(&r_v,sizeof(uint16_t),1,statevisitsvalues_file);
        // fwrite(T_v,sizeof(uint16_t),1,statevvalues_file);

        cursor = cursor->next;
        // printf("ExportToDat6\n");
    }
    // printf("ExportToDat2\n");

    /*
    fwrite(keyarr,sizeof(char),sizeof(keyarr)/sizeof(char),qdictkeys_file);
    fwrite(valarr,sizeof(float),sizeof(valarr)/sizeof(float),qdictvalues_file);
    fwrite(valvisitsarr,sizeof(uint16_t),sizeof(valvisitsarr)/sizeof(uint16_t),statevisitsvalues_file);
    */

    fclose(qdictkeys_file);
    fclose(qdictvalues_file);
    fclose(statevisitsvalues_file);

    return 0;
}
