struct md_node {
    struct md_node *next;
    char *key;

    float f, l, r, T;

    // note this implies max 65k visits per state before buffer overflow
    uint16_t f_v, l_v, r_v, T_v;
//     int f_v;
//     int l_v;
//     int r_v;
//     int T_v;

    uint8_t best_act;
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
md_node* search(md_linkedlist* alist, char *key);
uint8_t md_free_list(md_linkedlist* alist);

md_linkedlist *md_import_from_text(char* qdict_filename, char* statev_filename);
md_linkedlist *md_import_from_dat(char* qdictkeys_filename,
        char* qdictvalues_filename, char* statevvalues_filename);

uint8_t md_export_to_text(md_linkedlist* mylist, char* qdict_filename, char* statev_filename);
uint8_t md_export_to_dat(md_linkedlist* mylist, char* qdictkeys_filename,
        char* qdictvalues_filename, char* statevvalues_filename);
