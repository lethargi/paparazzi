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

#include "modules/my_visrl/mydict.h"

//IMPORT AND EXPORTS HAVE NOT BEEN ADAPTED FOR EXTRA ACTION; ie option of
//turn till color is seen


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
    for (int i = 0; i < 4; i++){
        if (cursor->values[i] > cur_best_val) {
            cur_best_val = cursor->values[i];
            cursor->best_action = i;
        }
    }
}

md_node* md_create(char *key, md_node *next)
{
    md_node *new_node = (md_node*)malloc(sizeof(md_node));
    if(new_node == NULL)
    {
        printf("\nError creating node\n");
        return 0;
        // exit(0);
    }
    strcpy(new_node->key,key);

    // value and visits functions initialized at 0
    for(int i = 0; i<4; i++) {
        new_node->values[i] = 0;
        new_node->visits[i] = 0;
    }

    new_node->next = next;

    return new_node;
}

md_node* md_prepend_list(md_linkedlist* mylist, char *key)
{
    md_node* new_node = md_create(key, mylist->head);
    mylist->head = new_node;
    mylist->length++;
    return new_node;
}

md_node* md_search(md_linkedlist* alist, char *key)
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

uint8_t md_import_from_text(md_linkedlist *mylist, char* qdict_filename,
        char* statev_filename)
{
    printf("\n Importing from txt \n");

    size_t dummy;
    md_node *cursor;

    FILE *qdict_txt_file = fopen(qdict_filename,"r");
    FILE *statevisits_txt_file = fopen(statev_filename,"r");

    //remove first line
    dummy = fscanf(qdict_txt_file, "%*[^\n]");
    dummy = fscanf(statevisits_txt_file, "%*[^\n]");

    char akey[30];
    float val[3];//, T;
    int vis[3];// T_v;
    while (!feof(qdict_txt_file)) {

        // read values from file and store in cache variable
        dummy = fscanf(qdict_txt_file, "%s %f %f %f", akey, &val[0], &val[1], &val[2]);
        dummy = fscanf(statevisits_txt_file, "%s %d %d %d", akey, &vis[0], &vis[1], &vis[2]);

        // create new list element with key
        cursor = md_prepend_list(mylist,strdup(akey));

        // fill in other values
        for (int i = 0; i < 3; i++){
            cursor->values[i] = val[i];
            cursor->visits[i] = vis[i];
        }
    }
    fclose(qdict_txt_file);
    fclose(statevisits_txt_file);

    printf("\n Done \n");
    return 0;
}

uint8_t md_import_from_dat(md_linkedlist *mylist, char* keys_filename,
        char* values_filename, char* visits_filename)
{
    printf("\n Importing from dat \n");

    size_t dummy;
    md_node *cursor;

    //safety feature about testing if "myqdict" already exists is unimplemented

    FILE *keys_file = fopen(keys_filename,"rb");
    FILE *values_file = fopen(values_filename,"rb");
    FILE *visits_file = fopen(visits_filename,"rb");
//     fwrite(valvisitsarr,sizeof(uint16_t),sizeof(valvisitsarr)/sizeof(uint16_t),statevisitsvalues_file);

    char akey[30];

    if ((keys_file != NULL) && (values_file != NULL) && (visits_file != NULL)) {
        while (!feof(keys_file)) {
            // Read from the dat file
            dummy = fread(&akey,sizeof(char[30]),1,keys_file);

            cursor = md_prepend_list(mylist,strdup(akey));

            dummy = fread(cursor->values,sizeof(float),3,values_file);
            dummy = fread(cursor->visits,sizeof(int),3,visits_file);
        }
    }
    else {
        printf("\n NO FILE TO READ \n");
    }
    fclose(keys_file);
    fclose(values_file);
    fclose(visits_file);

    printf("\n Done \n");

    return 0;
}

uint8_t md_export_to_text(md_linkedlist* mylist, char* qdict_filename, char* visits_filename)
{
    printf("\n Exporting to txt \n");

    FILE *qdict_txt_file = fopen(qdict_filename,"w");
    FILE *visits_txt_file = fopen(visits_filename,"w");

    md_node *cursor = mylist->head;

    fprintf(qdict_txt_file,"State \t\t\t | For \t\t | Lef \t\t | Rig \n");
    fprintf(visits_txt_file,"State \t\t\t | For \t\t | Lef \t\t | Rig \n");

    char akey[30];
    float f, l, r;//, T;
    int f_v, l_v, r_v;//, T_v;
    while(cursor != NULL)
    {
        strcpy(akey,cursor->key);

        f = cursor->values[0];
        l = cursor->values[1];
        r = cursor->values[2];

        f_v = cursor->visits[0];
        l_v = cursor->visits[1];
        r_v = cursor->visits[2];

        fprintf(qdict_txt_file, "%s %03.3f %03.3f %03.3f \n", (char *)akey, f, l, r);
        fprintf(visits_txt_file, "%s %4u %4u %4u \n", (char *)akey, f_v, l_v, r_v);
        cursor = cursor->next;
    }

    fclose(qdict_txt_file);
    fclose(visits_txt_file);

    printf("\n Done \n");

    return 0;
}

uint8_t md_export_to_dat(md_linkedlist* mylist, char* keys_filename,
        char* values_filename, char* visits_filename)
{
    printf("\n Exporting to dat \n");
    FILE *keys_file = fopen(keys_filename,"wb");
    FILE *values_file = fopen(values_filename,"wb");
    FILE *visits_file = fopen(visits_filename,"wb");

    md_node *cursor = mylist->head;

    while(cursor != NULL)
    {
        fwrite(cursor->key,sizeof(char),sizeof(cursor->key)/sizeof(char),keys_file);
        fwrite(cursor->values,sizeof(float),3,values_file);
        fwrite(cursor->visits,sizeof(int),3,visits_file);

        cursor = cursor->next;
    }

    fclose(keys_file);
    fclose(values_file);
    fclose(visits_file);

    printf("\n Done \n");

    return 0;
}
