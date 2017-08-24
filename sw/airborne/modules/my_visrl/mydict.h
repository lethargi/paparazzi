
#ifndef MYDICT_H
#define MYDICT_H

struct md_node {
    struct md_node *next;

    char *key;
    float values[3];
    // using int here is inefficient in terms of memory
    int visits[3];

    uint8_t best_action;
} ;

struct md_linkedlist {
    struct md_node *head;
    // node* tail;
    uint16_t length;
    size_t elemsize;
};

// can be done with the structure definition
typedef struct md_linkedlist md_linkedlist;
typedef struct md_node md_node;


md_linkedlist *md_init_linkedlist(void);
void md_node_best_action(md_node *cursor);
md_node* md_create(char *key, md_node *next);
md_node* md_prepend_list(md_linkedlist* mylist, char *key);
md_node* md_search(md_linkedlist* alist, char *key);
uint8_t md_free_list(md_linkedlist* alist);

uint8_t md_import_from_text(md_linkedlist *mylist, char* qdict_filename,
        char* statev_filename);
uint8_t md_import_from_dat(md_linkedlist *mylist, char* keys_filename,
        char* values_filename, char* visits_filename);

uint8_t md_export_to_text(md_linkedlist* mylist, char* qdict_filename, char* visits_filename);
uint8_t md_export_to_dat(md_linkedlist* mylist, char* keys_filename,
        char* values_filename, char* visits_filename);

#endif
