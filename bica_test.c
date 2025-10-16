#include "bica.h"

int num_tests = 0;
int num_passed = 0;

/* ################ */
/* HELPER FUNCTIONS */
/* ################ */
void test_start(int test_no, char* test_name){
    num_tests++;
    printf("------------------------\nTEST %d: %s\n\n", test_no, test_name);
}

void test_end(int success){
    if(success) num_passed++;
    printf("\nTEST %s | %d/%d Running Total\n\n", success? "PASS":"FAIL", num_passed, num_tests);
}

void print_bica_arr(unsigned char * arr, int size){
    printf("BICAMSG{");
    for(int i = 0; i < size; i++)
        printf("%02x%s", arr[i], (i == size-1)?"": " ");
    printf("}\n");
}

/* ################# */
/* TESTING FUNCTIONS */
/* ################# */
// test_blank message generation (and lookup)
void test_1(){
    test_start(1, "M_TEST_DUMMY Create");
    bica_func function;
    function = bica_get_function(BICAM_TEST_DUMMY, BICAT_CREATE);

    if(function == nullptr){ 
        test_end(0);
    }else{
        unsigned char buf[BICA_BUFFER_LEN];
        function(buf, BICA_BUFFER_LEN, NULL);
        print_bica_arr(buf, BICA_BUFFER_LEN);
        for (int i = 1; i< BICA_BUFFER_LEN; i++){
            if(buf[i] != i){
                test_end(0);
                return;
            }
        }
        if(BICA_BUFFER_LEN>0){
            if(buf[0]!=0xFF){
                test_end(0);
                return;
            }
        }
        test_end(1);
    }
}

//Fail to find result
void test_2(){
    test_start(2, "M_TEST_BLANK FAIL TO FIND");
    bica_func function;
    function = bica_get_function(BICAM_TEST_NULLPTR, BICAT_PROCESS);

    test_end(function == nullptr);
}

//Buffer allocation
void test_3(){
    test_start(3, "bica_allocate_buffer()");
    unsigned char * buf = bica_allocate_buffer();
    print_bica_arr(buf, BICA_BUFFER_LEN);

    test_end(buf != nullptr);
    if(buf!=nullptr) free(buf);
}

//Adding a function and retrieving it
int dummy_var = 0;
int dummy_func(unsigned char* buffer, int buffer_len, void* data){
    dummy_var = 1;
    return 1;
}

void test_4(){
    test_start(4, "Modify BICA Table");
    bica_set_hook(BICAM_TEST_DUMMY, BICAT_PROCESS, &dummy_func);
    dummy_var = 0;

    bica_func function;
    function = bica_get_function(BICAM_TEST_DUMMY, BICAT_PROCESS);
    unsigned char buf[BICA_BUFFER_LEN];
    if(function == nullptr) printf("Bad Function Lookup\n");
    else function(buf, BICA_BUFFER_LEN, NULL);
    printf("Dummy Variable: %s\n",dummy_var?"Success":"Fail");

    test_end(dummy_var);
    dummy_var = 0;
    bica_set_hook(BICAM_TEST_DUMMY, BICAT_PROCESS, nullptr);
}

//Indexing/binary search test
void test_5(){
    test_start(5, "Check All Lookups");
    int pass = 1;
    for (int i = 0; i < BICA_NUM_MESSAGE_IDS; i++){
        int index_pass = _bica_get_index(_bica_m_lookup_table[i].message_id)==i;
        printf("0x%02x: %s\n", _bica_m_lookup_table[i].message_id, index_pass?"PASS":"FAIL");
        if(!index_pass)
            pass = 0;
        
    }
    test_end(pass);
}

//Overwriting on_nullptr
int dummy_var_2 = 0;
void dummy_func_2(unsigned char message_id, int type, int index_found){
    printf("Called new nullptr func, %02x\n", message_id);
    dummy_var_2 = 1;
}

void test_6(){
    test_start(6, "set bica_on_nullptr");
    bica_get_function(BICAM_TEST_DUMMY, BICAT_PROCESS);
    printf("Overwrite on_nullptr\n");
    bica_on_nullptr = dummy_func_2;
    bica_get_function(BICAM_TEST_DUMMY, BICAT_PROCESS);

    test_end(dummy_var_2);
    dummy_var_2 = 0;
    bica_on_nullptr = _bicad_on_nullptr;
}

int main(int argc, char* argv[]){
    test_1();
    test_2();
    test_3();
    test_4();
    test_5();
    test_6();
}